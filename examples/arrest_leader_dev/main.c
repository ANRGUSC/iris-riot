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

#include "board.h"
#include "thread.h"
#include "mutex.h"
#include "msg.h"
#include "random.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/netdev.h"
#include "periph/uart.h"
#include "../mbed_riot/main-conf.h"
#include "periph/adc.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifndef UART_STDIO_DEV
#define UART_STDIO_DEV          (UART_UNDEF)
#endif

#define SOUNDRF_SENDER_PRIO     (THREAD_PRIORITY_MAIN) 
#define MAX_ADDR_LEN            (8U)

/* from CC2538's uart.c */
#undef BIT
#define BIT(n) ( 1 << (n) )
/* Bit field definitions for the UART Line Control Register: */
#define FEN   BIT( 4) /**< Enable FIFOs */
#define UART_CTL_HSE_VALUE    0

static msg_t main_msg_queue[16];
/* larger queue for fast incoming RSSI packets */
static msg_t soundrf_sender_queue[16];

static char soundrf_sender_stack[THREAD_STACKSIZE_MAIN];

/* Holds the current main radio channel. */
static uint16_t main_channel = ARREST_DATA_CHANNEL;

/* TODO: need to program leader-follower IP discovery */

/**
 * Set short hardware address of radio. Works only if only one netif exists.
 * @param target_hwaddr_str Null terminated string in format similar to "ff:ff". 
 */
static int _set_hwaddr_short(const char *target_hwaddr_str)
{
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs);
    uint8_t target_hwaddr_short[2];
    uint8_t target_hwaddr_short_len;
    if (numof == 1) {
        target_hwaddr_short_len = gnrc_netif_addr_from_str(target_hwaddr_short,
                                    sizeof(target_hwaddr_short), target_hwaddr_str);
        return gnrc_netapi_set(ifs[0], NETOPT_ADDRESS, 0, target_hwaddr_short, 
                               target_hwaddr_short_len);
    }

    return -1; /* fail */
}

static int _set_hwaddr_long(const char *target_hwaddr_str)
{
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    uint8_t target_hwaddr_long[MAX_ADDR_LEN];
    size_t target_hwaddr_long_len;
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        target_hwaddr_long_len = gnrc_netif_addr_from_str(target_hwaddr_long, 
                                  sizeof(target_hwaddr_long), target_hwaddr_str);
        return gnrc_netapi_set(ifs[0], NETOPT_ADDRESS_LONG, 0, target_hwaddr_long, 
                               target_hwaddr_long_len);
    }

    return -1; /* fail */
}


/**
 * Set channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static int _set_channel(uint16_t channel)
{
    if (channel < 11 || channel > 26) {
        return -1; /* invalid channel number */
    }
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        return gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    }

    return -1; /* fail */
}

/**
 * Set radio transmit power. CC2538 specific. Works only if one netif exists.
 * @param  power Power  in dBm within the range -24 to 7.
 * @return       0 on success. Implementation defined negative number on failure.
 */
static int _set_tx_power(uint16_t power)
{
    if (power < -24 || power > 7)
    {
        return -1; /* fail */
    }

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        return gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &power, sizeof(uint16_t));
    }

    return -1; /* fail */
}

/**
 * Only supports having one netif.
 * @param  rcvr_ip        IP address of soundrf ping receiving node.
 * @param  rcvr_port      Port of soundrf ping receiving thread.
 * @param  sender_port    Port of soundrf ping sending thread.
 * @param  sender_node_id Node ID of soundrf ping sender.
 * @return                [description]
 */
static int _soundrf_sender_rdy(ipv6_addr_t *rcvr_ip, uint16_t rcvr_port, 
                               uint16_t sender_port, uint8_t sender_node_id)
{
    char buf[2] = { RANGE_RDY_FLAG, sender_node_id };
    gnrc_pktsnip_t *payload, *udp, *ip;

    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        DEBUG("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, sender_port, rcvr_port);
    if (udp == NULL) {
        DEBUG("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, rcvr_ip);
    if (ip == NULL) {
        DEBUG("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
        DEBUG("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    return 0;
}

/**
 * Only supports having one netif.
 * @param  rcvr_hwaddr     [description]
 * @param  rcvr_hwaddr_len [description]
 * @param  sender_node_id  [description]
 * @return                 [description]
 */
static int _soundrf_send_pings(uint8_t *rcvr_hwaddr, size_t rcvr_hwaddr_len, 
                                    uint8_t sender_node_id)
{
    gnrc_pktsnip_t *pkt, *hdr;
    msg_t msg;
    gnrc_netif_hdr_t *nethdr;
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    uint8_t flags = 0x00;

    /* turn off UART temporarily because of HDLC*/
    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 0;
    /* depending on implementation, turn off any other interrupts as needed */

    char buf[3] = { RANGE_FLAG_BYTE0, RANGE_FLAG_BYTE1, sender_node_id };

    size_t numof = gnrc_netif_get(ifs);

    if(numof != 1) {
        DEBUG("Error: more than 1 netif\n");
        return -1;
    }

    pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
    if (pkt == NULL) {
        DEBUG("Error: unable to copy data to packet buffer");
        return -1;
    }

    hdr = gnrc_netif_hdr_build(NULL, 0, rcvr_hwaddr, rcvr_hwaddr_len);
    if (hdr == NULL) {
        DEBUG("Error: packet buffer full\n");
        gnrc_pktbuf_release(pkt);
        return -1;
    }

    LL_PREPEND(pkt, hdr);
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags =  flags; /* this is used mainly to differentiate bcast pkts */

    if (gnrc_netapi_send(ifs[0], pkt) < 1) {
        DEBUG("Error: unable to send\n");
        gnrc_pktbuf_release(pkt);
        return -1;
    }

    /* after gnrc_netapi_send(), the RF packet + sound ping should be sent when
    the function returns. Manually reset UART to resume normal operation
    (see CC2538's uart.c) */

    UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
    UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
    UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
    UART1->cc2538_uart_dr.ECR = 0xFF;
    UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
    UART1->cc2538_uart_lcrh.LCRH |= FEN;
    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;

    return 0;
}

/* This thread deals with sound ranging */
static void *_soundrf_sender(void *arg) 
{
    msg_t msg;
    uint8_t rcvr_hwaddr[MAX_ADDR_LEN];
    size_t rcvr_hwaddr_len;
    ipv6_addr_t soundrf_rcvr_ip;
    gnrc_pktsnip_t *gnrc_pkt, *snip;
    gnrc_netreg_entry_t sender_server = {NULL, ARREST_LEADER_SOUNDRF_PORT, thread_getpid()};

    msg_init_queue(soundrf_sender_queue, sizeof(soundrf_sender_queue));
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &sender_server);

    /* Turn off MB13XX ultrasonic sensor using the following pin */ 
    if(gpio_init(GPIO_PA4, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    gpio_clear(GPIO_PA4);

    /* max out tx power */
    _set_tx_power(7);


    if (ipv6_addr_from_str(&soundrf_rcvr_ip, ARREST_FOLLOWER_IPV6_ADDR) == NULL) {
        DEBUG("Error: unable to parse destination address");
        return NULL;
    }

    rcvr_hwaddr_len = gnrc_netif_addr_from_str(rcvr_hwaddr, sizeof(rcvr_hwaddr), 
                                               ARREST_FOLLOWER_SHORT_HWADDR);

    if (rcvr_hwaddr_len == 0) {
        DEBUG("error: invalid address given\n");
        return 1;
    }

    while(1)
    {
        msg_receive(&msg);

        switch (msg.type)
        {

            case GNRC_NETAPI_MSG_TYPE_RCV:
                gnrc_pkt = (gnrc_pktsnip_t *)msg.content.ptr;
                snip = gnrc_pktsnip_search_type(gnrc_pkt, GNRC_NETTYPE_UNDEF);
                if ( RANGE_REQ_FLAG == ((uint8_t *)snip->data)[0] && 
                        ARREST_LEADER_SOUNDRF_ID == ((uint8_t *)snip->data)[1] ) {
                    DEBUG("Got REQ. Sending 'RDY' pkt now!\n");
                    gnrc_pktbuf_release(gnrc_pkt);  
                    _soundrf_sender_rdy(&soundrf_rcvr_ip, GET_SET_RANGING_THR_PORT,
                        ARREST_LEADER_SOUNDRF_PORT, ARREST_LEADER_SOUNDRF_ID);
                } else if ( RANGE_GO_FLAG == ((uint8_t *)snip->data)[0] && 
                        ARREST_LEADER_SOUNDRF_ID == ((uint8_t *)snip->data)[1] ) {
                    DEBUG("Got GO. Time to send sound/rf ping!\n");
                    gnrc_pktbuf_release(gnrc_pkt);  
                    range_tx_init(GPIO_PA4);
                    _soundrf_send_pings(rcvr_hwaddr, rcvr_hwaddr_len, 
                                        ARREST_LEADER_SOUNDRF_ID);
                    range_tx_off();
                    DEBUG("RF and ultrasound pings sent!\n");
                } else {
                    gnrc_pktbuf_release(gnrc_pkt);  
                    DEBUG("soundrf_sender: invalid pkt!\n");
                }

                
                break;
            default:
                /* error */
                DEBUG("soundrf_sender: unknown msg_t!\n");
                break;
        }    
    }

    /* should be never reached */
    DEBUG("Error: Reached Exit!");
    return NULL;
}

int main(void)
{
    msg_t msg;
    gnrc_pktsnip_t *snip;
    gnrc_netif_hdr_t *hdr;
    uint8_t *dst_addr;
    gnrc_netreg_entry_t rmt_ctrl_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, thread_getpid() };

    msg_init_queue(main_msg_queue, 8);

    /* hwaddr for reference assumign _set_hwaddr_short() is successful below */
    uint8_t my_hwaddr_short[2];
    gnrc_netif_addr_from_str(my_hwaddr_short, sizeof(my_hwaddr_short), ARREST_LEADER_SHORT_HWADDR);

    _set_hwaddr_short(ARREST_LEADER_SHORT_HWADDR);
    // _set_hwaddr_long(ARREST_LEADER_LONG_HWADDR);
    _set_channel(ARREST_DATA_CHANNEL);

    thread_create(soundrf_sender_stack, sizeof(soundrf_sender_stack), SOUNDRF_SENDER_PRIO, 0,
                  _soundrf_sender, (void *) 0, "soundrf_sender");

    gnrc_netreg_register(1, &rmt_ctrl_serv);

    DEBUG("Starting main thread...\n");

    while (1) {
        uint8_t *c_ptr = NULL;
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("Remote control received packet.\n");
                /* content of message holds a linked list of all the packet contents*/
                snip = msg.content.ptr;
                /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
                if(snip->size == 2 && *(uint8_t *)snip->data == (uint8_t)REMOTE_CTRL_FLAG) {
                    c_ptr = snip->data + 1;
                }

                snip = snip->next;

                /* for correct packets, the next snip in the is the l2 header
                   holding a short addr */
                if (snip->type == GNRC_NETTYPE_NETIF ) {
                    hdr = (gnrc_netif_hdr_t *)snip->data;
                    dst_addr = gnrc_netif_hdr_get_dst_addr(hdr); 
                    if( hdr->dst_l2addr_len == 2 && dst_addr[0] == my_hwaddr_short[0] 
                        && dst_addr[1] == my_hwaddr_short[1]) {
                        uart_write(UART_DEV(0), c_ptr, 1);
                        fflush(stdout);
                        DEBUG("\n");
                    } 
                }
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            default:
                // gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                DEBUG("msg.type = %d", msg.type);
                DEBUG("remote_ctrl: unknown msg_t!\n");
                break;
        }
    }

    /* should be never reached */
    return 0;
}
