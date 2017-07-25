/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Application for ARREST project.
 *
 *              RSSI dumping over uart and sound ranging features.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "board.h"
#include "thread.h"
#include "mutex.h"
#include "msg.h"
#include "random.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/netdev.h"
#include "periph/uart.h"
#include "hdlc.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "periph/adc.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifndef UART_STDIO_DEV
#define UART_STDIO_DEV          (UART_UNDEF)
#endif

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define DISPATCHER_PRIO         (THREAD_PRIORITY_MAIN - 1)
#define RSSI_DUMP_PRIO          (THREAD_PRIORITY_MAIN - 1) 

/* from CC2538's uart.c */
#undef BIT
#define BIT(n) ( 1 << (n) )
/* Bit field definitions for the UART Line Control Register: */
#define FEN   BIT( 4) /**< Enable FIFOs */
#define UART_CTL_HSE_VALUE    0

static msg_t main_msg_queue[8];
/* larger queue for fast incoming RSSI packets */
static msg_t rssi_dump_msg_queue[16];

static char hdlc_stack[THREAD_STACKSIZE_MAIN];
static char dispatcher_stack[THREAD_STACKSIZE_MAIN];
static char rssi_dump_stack[THREAD_STACKSIZE_MAIN];

static void _dump(gnrc_pktsnip_t *pkt)
{
    int snips = 0;
    int size = 0;
    int raw_rssi =0;
    gnrc_netif_hdr_t *hdr;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_NETIF)
        {
            hdr = (gnrc_netif_hdr_t *) (snip->data);
            raw_rssi=hdr->rssi;
            raw_rssi -=73;
            printf("rssi: %d\n", raw_rssi);            
        }
        ++snips;
        size += snip->size;
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}

/* Note, the use of a mutex may slow down RSSI readings */

static void *_rssi_dump(void *arg) 
{
    //hdlc pid
    kernel_pid_t hdlc_pid = (kernel_pid_t) (uintptr_t) arg;
    msg_t msg, msg2;
    //Don't know yet 
    bool is_rssi_pkt = false;
    bool hdlc_snd_locked = false;
    hdlc_pkt_t *recv_pkt;
    gnrc_netif_hdr_t *netif_hdr;
    uint8_t *dst_addr;
    msg_init_queue(rssi_dump_msg_queue, sizeof(rssi_dump_msg_queue));
    gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               thread_getpid());

    hdlc_entry_t rssi_dump_thr = { .next = NULL, .port = RSSI_DUMP_PORT, 
                                         .pid = thread_getpid() };
    hdlc_register(&rssi_dump_thr);
    
    char send_data[UART_PKT_HDR_LEN + 1];
    hdlc_pkt_t send_pkt = { send_data, UART_PKT_HDR_LEN + 1 };
    char rssi_data[UART_PKT_HDR_LEN + 1];
    hdlc_pkt_t rssi_pkt = { rssi_data, UART_PKT_HDR_LEN + 1 };

    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t rssi_uart_hdr;
    printf("completed\n" );

    while(1)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                recv_pkt = msg.content.ptr;
                uart_pkt_hdr_t recv_uart_hdr;
                uart_pkt_parse_hdr(&recv_uart_hdr, recv_pkt->data, recv_pkt->length);
                DEBUG("_rssi_dump: Packet type : %d \n",recv_uart_hdr.pkt_type);

            case HDLC_RESP_RETRY_W_TIMEO:
                DEBUG("_rssi_dump : retry frame \n");
                if (is_rssi_pkt) {
                    /* don't bother resending */
                    is_rssi_pkt = false;
                    hdlc_snd_locked = false;
                } else {
                    xtimer_usleep(msg.content.value);
                    DEBUG("rssi_dump: resend UART packet\n");
                    msg_send(&msg2, hdlc_pid);
                }

                break;
            case HDLC_RESP_SND_SUCC:
                // DEBUG("_rssi_dump : sent frame \n");
                hdlc_snd_locked = false;
                break;
            case GNRC_NETAPI_MSG_TYPE_RCV:
                // DEBUG("_rssi_dump : received a beacon \n");

                if (!hdlc_snd_locked) { 
                    _dump(msg.content.ptr);
                    }
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* just in case */
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            default:
                /* error */
                DEBUG("rssi_dump: invalid packet\n");
                break;
            }
        }    
    

    /* should be never reached */
    DEBUG("Error: Reached Exit!");
    return NULL;
}


int main(void)
{
    msg_t msg, msg_snd_pkt;
    int hdlc_send_locked = 0;
    int range_req = 0;
    uint32_t sound_rf_tdoa;
    uint32_t range_req_time;
    kernel_pid_t rssi_pid;
    hdlc_pkt_t *recv_pkt;
    gnrc_pktsnip_t *gnrc_pkt;
    gnrc_pktsnip_t *snip;
    ipv6_addr_t sound_rf_sender_ip;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_pkt = { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    uart_pkt_hdr_t uart_hdr;
    hdlc_entry_t main_thr = { NULL, GET_SET_RANGING_THR_PORT, thread_getpid() };

    msg_init_queue(main_msg_queue, sizeof(main_msg_queue));

    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
        "hdlc", UART_DEV(1));
    rssi_pid = thread_create(rssi_dump_stack, sizeof(rssi_dump_stack), RSSI_DUMP_PRIO, NULL,
                  _rssi_dump, (void *) (uint32_t) hdlc_pid, "rssi_dump");
    static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);
    uint32_t port=RSSI_DUMP_PORT;
    server.target.pid = rssi_pid;
    server.demux_ctx = port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);

    hdlc_register(&main_thr);

    
    /* this thread handles set tx power, set channel, and ranging requests */
    while(1)
    {
        LED3_ON;
        break;
        xtimer_usleep(10000);
    }


    /* should be never reached */
    return 0;
}
