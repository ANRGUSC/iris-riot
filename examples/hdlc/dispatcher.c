/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 * Pradipta Ghosh
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
 * @file        dispatcher.c
 * @brief       dispatcher thread. handles all incoming packets (hdlc, radio, etc.).
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "hdlc.h"
#include "yahdlc.h"
#include "fcs16.h"
#include "dispatcher.h"
#include "uart_pkt.h"
#include "main-conf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* TODO: merge dispatcher into hdlc? */

static msg_t _dispatcher_msg_queue[16];

static std::map <char, Mail<msg_t, HDLC_MAILBOX_SIZE>*> mailbox_list;
static int registered_thr_cnt = 0;
Mutex thread_cnt_mtx;
static int thread_cnt = 0;

/**
 * @brief Thread IDs for dispatching
 */
static kernel_pid_t ultrasound_ranging_pid;

typedef struct dispatcher_entry {
    struct dispatcher_entry *next;
    uint16_t port;
    kernel_pid_t pid;
} dispatcher_entry_t;

static dispatcher_entry_t *dispatcher_reg;

void dispatcher_register(kernel_pid_t thread_id, dispatcher_entry_t *entry)
{
    LL_PREPEND(dispatcher_reg, entry);
}

void dispatcher_unregister(kernel_pid_t thread_id, dispatcher_entry_t *entry)
{
    LL_DELETE(dispatcher_reg, entry);
}

/**
 * @brief Dispatch or handle incoming hdlc messages. Get and set commands should
 *        be handled here. 
 *
 * @param[in] arg       PID of hdlc thread
 *
 */
static void *_dispatcher(void *arg)
{
    kernel_pid_t hdlc_thread_pid = (kernel_pid_t)arg;
    kernel_pid_t *sender_pid;

    msg_t *msg, *msg2;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t send_pkt = { .data = send_data, .length = 0};

    msg.type = HDLC_MSG_REG_DISPATCHER;
    msg.content.value = (uint32_t) 0;
    msg_send(&msg, hdlc_thread_pid);

    /* subscribe this thread to udp packets on DISPATCHER_UDP_PORT */
    gnrc_netreg_entry_t dispatcher_server = { NULL, (uint32_t) DISPATCHER_UDP_PORT, thread_getpid() };
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &dispatcher_server);

    while(1)
    {
        msg_receive(&msg);

        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                hdlc_pkt_t *recv_pkt = msg.content.ptr;   
                if(_handle_pkt(&send_pkt, recv_pkt->data, recv_pkt->length)) {
                    msg.type = HDLC_MSG_SND;
                    msg.content.ptr = &send_pkt;
                    msg_send(&msg, hdlc_pid);
                }
                break;
            case GNRC_NETAPI_MSG_TYPE_RCV:
                /* received radio packet. */
                gnrc_pktsnip_t *recv_gnrc_pkt = msg.content.ptr;
                if (_handle_pkt(&send_pkt, recv_gnrc_pkt->data, recv_gnrc_pkt->size)) {
                    /* need to fwd packet */  
                    msg.type = HDLC_MSG_SND;
                    msg.content.ptr = &send_pkt;
                } else {
                    gnrc_pktbuf_release(recv_gnrc_pkt);
                }
            default:
                /* error */
                DEBUG("Dispatcher: invalid packet\n");
                break;
        }    
    }

    DEBUG("Error: Reached Exit!");
    /* should be never reached */
}

static int _handle_pkt(hdlc_pkt_t *send_pkt, uint8_t *data, size_t length) 
{
    uart_pkt_hdr_t rcv_hdr;
    uart_pkt_parse_hdr(&rcv_hdr, data, length);

    switch (rcv_hdr.pkt_type)
    {
        /* TODO: free any radio packets received */
        case RADIO_SET_CHAN:
            uart_pkt_hdr_t hdr;
            hdr.dst_port = rcv_hdr.src_port; 
            hdr.src_port = rcv_hdr.dst_port; 
            uart_pkt_insert_hdr(send_pkt->data, HDLC_MAX_PKT_SIZE, &hdr);
            uint16_t channel = data[UART_PKT_DATA_FIELD];
            kernel_pid_t ifs[GNRC_NETIF_NUMOF];
            gnrc_netif_get(ifs);

            if(gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t)) < 0) {
                send_pkt->data[UART_PKT_TYPE_FIELD] = RADIO_SET_CHAN_FAIL;
            } else {
                send_pkt->data[UART_PKT_TYPE_FIELD] = RADIO_SET_CHAN_SUCCESS;
            }

            send_pkt->data[UART_PKT_DATA_FIELD] = (uint8_t)channel;
            send_pkt->length = UART_PKT_HDR_LEN + 1;
            return 1; /* need to send packet */
        case RADIO_SET_POWER:
            char send_data[2];
            hdlc_pkt_t *pkt = { send_data, sizeof(send_data) };
            int16_t power = (uint16_t)data[1];
            if(gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &power, sizeof(uint16_t)) < 0) {
                send_data[0] = RADIO_SET_POWER_FAIL;
            } else {
                send_data[0] = RADIO_SET_POWER_SUCCESS;
            } 
            send_data[1] = data[1];
            break;
        case SOUND_RANGE_REQ:
            ultrasound_range_recv();
            break;
        case SOUND_RANGE_X10_REQ:
            ultrasound_range_x10_recv();
            break;
        case FWD_TO_MBED:
            break;
        default:
            DEBUG("Unknown packet type.\n");
            return 1;
    }
    return 0
}

kernel_pid_t dispacher_init(char *stack, int stacksize, char priority, const char *name, kernel_pid_t hdlc_thread_pid)
{
    kernel_pid_t res;

    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST, 
            _dispatcher, (void *) hdlc_thread_pid, name);

    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}
