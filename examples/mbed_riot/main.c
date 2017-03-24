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
#include "periph/uart.h"
#include "hdlc.h"
#include "dispatcher.h"
#include "uart_pkt.h"
#include "main-conf.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifndef UART_STDIO_DEV
#define UART_STDIO_DEV          (UART_UNDEF)
#endif

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define DISPATCHER_PRIO         (THREAD_PRIORITY_MAIN - 1)

static msg_t main_msg_queue[8];
/* larger queue for fast incoming RSSI packets */
static msg_t rssi_dump_msg_queue[16];

static char hdlc_stack[THREAD_STACKSIZE_MAIN];
static char dispatcher_stack[512];
static char rssi_dump_stack[THREAD_STACKSIZE_MAIN];

/* Holds the current main radio channel. */
static uint16_t main_channel = DEFAULT_CHANNEL;

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
    gnrc_netif_get(ifs);
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
    gnrc_netif_get(ifs);
    if (numof == 1) {
        return gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &power, sizeof(uint16_t));
    }

    return -1; /* fail */
}

/* Note, the use of a mutex may slow down RSSI readings */
static void *_rssi_dump(void *arg) 
{
    kernel_pid_t hdlc_thread_pid = (kernel_pid_t)arg;
    kernel_pid_t *sender_pid;
    msg_t *msg;
    gnrc_netreg_entry_t rssi_dump_server = {NULL, RSSI_DUMP_PORT, thread_getpid()};


    msg_init_queue(rssi_dump_msg_queue, sizeof(rssi_dump_msg_queue));

    dispatcher_entry_t rssi_dump_thr = { .dispatcher_entry = NULL, 
        .port = RSSI_DUMP_PORT, .pid = thread_getpid() };
    dispatcher_register(&rssi_dump_thr);

    _set_hwaddr_short(ARREST_LEADER_SHORT_HWADDR);

    mutex_t send_pkt_mtx;
    char send_data[UART_PKT_HDR_LEN + 1];
    hdlc_pkt_t send_pkt = { .data = send_data, .length = UART_PKT_HDR_LEN + 1 };
    uart_pkt_hdr_t uart_hdr = {
        .src_port = RSSI_DUMP_PORT,
        .dst_port = 0,               /* no dst_port needed for mbed's dispatcher */
        .pkt_type = RSSI_DATA_PKT
    };
    /* same header for all outgoing packets */
    uart_pkt_insert_hdr(send_pkt.data, UART_PKT_HDR_LEN + 1, &uart_hdr); 

    while(1)
    {
        msg_receive(&msg);

        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                hdlc_pkt_t *recv_pkt = msg.content.ptr;
                uart_pkt_hdr_t uart_hdr;
                uart_pkt_parse_hdr(&uart_hdr, recv_pkt->data, recv_pkt->length);
                switch (uart_hdr.pkt_type) 
                {
                    case RSSI_DUMP_START:
                        /* check for custom channel request */
                        if (recv_pkt->length > UART_PKT_HDR_LEN) {
                            _set_channel(recv_pkt->[UART_PKT_DATA_FIELD]);
                        } else {
                            /* default localization channel */
                            _set_channel(RSSI_LOCALIZATION_CHAN);
                        }
                        gnrc_netreg_register(1, &rssi_dump_server);
                        break;
                    case RSSI_DUMP_STOP:
                        gnrc_netreg_unregister(1, &rssi_dump_server);
                        _set_channel(DEFAULT_CHANNEL);
                        break;
                    default:
                        DEBUG("rssi_dump: invalid packet!\n");
                        break;
                }
                hdlc_pkt_release(recv_pkt);
                break;
            case HDLC_RESP_RETRY_W_TIMEO:
                /* don't bother retrying */
                mutex_unlock(&send_pkt_mtx);
                break;
            case HDLC_RESP_SND_SUCC:
                mutex_unlock(&send_pkt_mtx);
                break;
            case GNRC_NETAPI_MSG_TYPE_RCV:
                if (mutex_trylock(&send_pkt_mtx)) {
                    gnrc_netif_hdr_t *netif_hdr = ((gnrc_pktsnip_t *)msg.content.ptr)->data;
                    uint8_t *dst_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
                    /* need to subtract 73 from raw RSSI (do on mbed side) to get dBm value */
                    uint8_t raw_rssi = netif_hdr->rssi;

                    if (netif_hdr->dst_l2addr_len == 2 && !memcmp(dst_addr, target_hwaddr_short, 2)) {
                        send_pkt.length = uart_pkt_cpy_data(send_pkt.data, 
                            UART_PKT_HDR_LEN + 1, &raw_rssi, 1); 

                        msg.type = HDLC_MSG_SND;
                        msg.content.ptr = &send_pkt;
                        msg_send(&msg, hdlc_pid);
                    }
                } /* else { do nothing, wait for next packet } */

                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
            default:
                /* error */
                DEBUG("rssi_dump: invalid packet\n");
                break;
        }    
    }

    /* should be never reached */
    DEBUG("Error: Reached Exit!");
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
            /* TODO */
            break;
        case SOUND_RANGE_REQ:
            ultrasound_range_recv();
            /* initialize sound ranging */
            /* do sound ranging */
            /* once done, send packet containing TDoA to mbed */
            break;
        case SOUND_RANGE_X10_REQ:
            /* TODO */
            break;
        default:
            DEBUG("Unknown packet type.\n");
            return 1;
    }
    return 0
}

int main(void)
{
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
        "hdlc", UART_DEV(1));
    kernel_pid_t dispatcher_pid = dispacher_init(dispatcher_stack, 
        sizeof(dispatcher_stack), DISPATCHER_PRIO, "dispatcher", hdlc_pid);
    kernel_pid_t rssi_dump_pid = thread_create(rssi_dump_stack, 
        sizeof(rssi_dump_stack), RSSI_DUMP_PRIO, "rssi_dump", hdlc_pid);

    msg_init_queue(main_msg_queue, 8);

    msg_t msg, msg_snd_pkt;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_pkt = { .data = send_data, .length = HDLC_MAX_PKT_SIZE };

    dispatcher_entry_t main_thr = { .next = NULL, .port = GET_SET_RANGING_THR_PORT, 
        .pid = thread_getpid() };
    dispatcher_register(&main_thr);

    send_hdlc_lock = 0;

    /* this thread handles set tx power, set channel, and ranging requests */
    while(1)
    {
        while(!send_hdlc_lock)
        {
            msg_receive(&msg);

            switch (msg.type)
            {
                case HDLC_RESP_SND_SUCC:
                    send_hdlc_lock = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg.content.value);
                    msg_send(&msg_snd_pkt, hdlc_pid);
                    break;
                case HDLC_PKT_RDY:
                    hdlc_pkt_t *recv_pkt = msg.content.ptr;
                    uart_pkt_hdr_t uart_hdr;
                    uart_pkt_parse_hdr(&uart_hdr, recv_pkt->data, recv_pkt->length);
                    switch (uart_hdr.pkt_type) 
                    {
                        case RADIO_SET_CHAN:
                            _set_channel(recv_pkt->[UART_PKT_DATA_FIELD]);
                            /* TODO: respond to mbed via RADIO_SET_CHAN_X msg */
                            break;
                        case RADIO_SET_POWER:
                            _set_tx_power(recv_pkt->[UART_PKT_DATA_FIELD]);
                            /* TODO: respond to mbed via RADIO_SET_POWER_X msg */
                            break;
                        case SOUND_RANGE_REQ:
                            sound_rf_ping_req();
                            break;
                        case SOUND_RANGE_X10_REQ:
                            /* TODO */
                            break;
                        default:
                            DEBUG("rssi_dump: invalid packet!\n");
                            break;
                    }
                    hdlc_pkt_release(recv_pkt);
                    break;
                case GNRC_NETAPI_MSG_TYPE_RCV:
                    gnrc_pktsnip_t *gnrc_pkt = msg.content.ptr;
                    gnrc_pktbuf_release(gnrc_pkt);
                    break;

                default:
                    /* error */
                    LED3_ON;
                    break;
            }
        }

    }

    /* should be never reached */
    return 0;
}

static int sound_rf_ping_req()
{
    /* register thread for response */

}

static int sound_rf_ping_go()
{
    /* turn off UART temporarily */
    /* turn off any other interrupts if needed */
    /* unregister thread from zigbee packets */
    /* range_rx_init() */
    /* send GO udp pkt */
    /* xtimer_msg_receive_timeout -- to be woken up by network layer externally*/
    /* on timeout or packet receive, reset UART just in case */
    /* manually reset cc2538 uart */

}



