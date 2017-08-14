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
#include "net/hdlc.h"
#include "net/uart_pkt.h"
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

/* Note, the use of a mutex may slow down RSSI readings */
static void *_rssi_dump(void *arg) 
{
    kernel_pid_t hdlc_pid = (kernel_pid_t) (uintptr_t) arg;
    msg_t msg, msg2;
    bool is_rssi_pkt = false;
    bool hdlc_snd_locked = false;
    hdlc_pkt_t *recv_pkt;
    gnrc_netif_hdr_t *netif_hdr;
    uint8_t *dst_addr;
    gnrc_netreg_entry_t rssi_dump_server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, {thread_getpid()} };


    msg_init_queue(rssi_dump_msg_queue, sizeof(rssi_dump_msg_queue));

    hdlc_entry_t rssi_dump_thr = { .next = NULL, .port = RSSI_DUMP_PORT, 
                                         .pid = thread_getpid() };
    hdlc_register(&rssi_dump_thr);

    /* assuming _set_hwaddr_short above was successful */
    uint8_t my_hwaddr_short[2];
    gnrc_netif_addr_from_str(my_hwaddr_short, sizeof(my_hwaddr_short), ARREST_FOLLOWER_SHORT_HWADDR);

    char send_data[UART_PKT_HDR_LEN + 1];
    hdlc_pkt_t send_pkt = { send_data, UART_PKT_HDR_LEN + 1 };
    char rssi_data[UART_PKT_HDR_LEN + 1];
    hdlc_pkt_t rssi_pkt = { rssi_data, UART_PKT_HDR_LEN + 1 };

    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t rssi_uart_hdr;

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

                switch (recv_uart_hdr.pkt_type) 
                {
                    case RSSI_DUMP_START:
                        /* TODO: check for custom channel request */
                        /* default localization channel */
                        _set_channel(RSSI_LOCALIZATION_CHAN);

                        if(!hdlc_snd_locked) {
                            hdlc_snd_locked = true; 
                            uart_hdr.dst_port = recv_uart_hdr.src_port;
                            uart_hdr.src_port = RSSI_DUMP_PORT;
                            uart_hdr.pkt_type = RSSI_SCAN_STARTED;
                            uart_pkt_insert_hdr(send_pkt.data, UART_PKT_HDR_LEN + 1,
                                &uart_hdr);
                            send_pkt.length = UART_PKT_HDR_LEN;

                            msg2.type = HDLC_MSG_SND;
                            msg2.content.ptr = &send_pkt;
                            msg_send(&msg2, hdlc_pid);
                        } else {
                            /* requeue the msg_t */
                            msg_send_to_self(&msg);
                        }

                        DEBUG("rssi_dump : dump STARTING... \n");

                        gnrc_netreg_register(1, &rssi_dump_server);
                        break;
                    case RSSI_DUMP_STOP:
                        gnrc_netreg_unregister(1, &rssi_dump_server);
                        _set_channel(main_channel);
                        DEBUG("rssi_dump: dump STOPPED.\n");

                        if (!hdlc_snd_locked) {
                            hdlc_snd_locked = true; 
                            uart_hdr.dst_port = recv_uart_hdr.src_port;
                            uart_hdr.src_port = RSSI_DUMP_PORT;
                            uart_hdr.pkt_type = RSSI_SCAN_STOPPED;
                            uart_pkt_insert_hdr(send_pkt.data, UART_PKT_HDR_LEN + 1,
                                &uart_hdr);
                            send_pkt.length = UART_PKT_HDR_LEN;

                            msg2.type = HDLC_MSG_SND;
                            msg2.content.ptr = &send_pkt;
                            msg_send(&msg2, hdlc_pid);
                            DEBUG("_rssi_dump : Dumping stopped \n");
                        } else {
                            /* requeue the msg_t */
                            msg_send_to_self(&msg);
                        }

                        /* TODO: send RSSI_SCAN_STOPPED to mbed */
                        break;
                    default:
                        DEBUG("rssi_dump: invalid packet!\n");
                        break;
                }
                hdlc_pkt_release(recv_pkt);
                break;
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
                    // DEBUG("rssi_dump: getting RSSI and sending to mbed...\n"); 

                    netif_hdr = ((gnrc_pktsnip_t *)msg.content.ptr)->next->data;
                    dst_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
                    /* need to subtract 73 from raw RSSI (do on mbed side) to get dBm value */
                    uint8_t raw_rssi = netif_hdr->rssi;
                    if(dst_addr == 0){/* do nothing*/}
                    if (netif_hdr->dst_l2addr_len == 2 ) {//&& !memcmp(dst_addr, my_hwaddr_short, 2)) {
                        hdlc_snd_locked = true;
                        rssi_uart_hdr.src_port = RSSI_DUMP_PORT;
                        rssi_uart_hdr.dst_port = 0xFFFF; /* taken care of by dispatcher */
                        rssi_uart_hdr.pkt_type = RSSI_DATA_PKT;
                        uart_pkt_insert_hdr(rssi_pkt.data, UART_PKT_HDR_LEN + 1,
                            &rssi_uart_hdr);
                        rssi_pkt.length = uart_pkt_cpy_data(rssi_pkt.data, 
                            UART_PKT_HDR_LEN + 1, &raw_rssi, sizeof(raw_rssi)); 

                        is_rssi_pkt = true;
                        msg2.type = HDLC_MSG_SND;
                        msg2.content.ptr = &rssi_pkt;

                        // DEBUG("rssi_dump: sending RSSI over uart...\n");

                        msg_send(&msg2, hdlc_pid);
                    }
                } /* else { do nothing, wait for next packet } */

                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
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

static int _sound_rf_ping_req(uint16_t rcvr_port, 
    ipv6_addr_t *sender_ip, uint16_t sender_port, uint8_t sender_node_id)
{
    char buf[2] = { RANGE_REQ_FLAG, sender_node_id };
    gnrc_pktsnip_t *payload, *udp, *ip;

    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, rcvr_port, sender_port);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, sender_ip);
    if (ip == NULL) {
        puts("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
        puts("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    return 0;
}

static uint32_t _sound_rf_ping_go(uint16_t rcvr_port, 
    ipv6_addr_t *sender_ip, uint16_t sender_port, uint8_t sender_node_id){

    msg_t msg;
    gnrc_pktsnip_t *payload, *udp, *ip;
    uint32_t retval;

    /* turn off UART temporarily because of HDLC*/
    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 0;
    /* depending on implementation, turn off any other interrupts as needed */

    /* range_rx_init() */
    /* send GO udp pkt */
    /* send "GO" packet */
    char buf[2] = { RANGE_GO_FLAG, sender_node_id };
    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 0;
    }

    udp = gnrc_udp_hdr_build(payload, rcvr_port, sender_port);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 0;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, sender_ip);
    if (ip == NULL) {
        puts("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 0;
    }

    adc_init(AD5_PIN);
    range_rx_init(sender_node_id, DEFAULT_ULTRASOUND_THRESH, AD5_PIN, 
        ADC_RES_7BIT, MAX_SOUND_SAMPLES);

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, 
        ip)) {
        puts("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 0;
    }

    DEBUG("PID of req is %d\n", thread_getpid());

    /* there should no be other messages being sent to this thread at this point */
    /* wait 500ms */
    if(xtimer_msg_receive_timeout(&msg, 500000) < 0) {
        /* timed out */
        DEBUG("netdev ranging timed out\n");
        retval = 0;
    } else {
        DEBUG("TDoA: %ld\n", (uint32_t) msg.content.value);
        retval = ((uint32_t) msg.content.value);
    }

    /* on timeout or packet receive, manually reset UART just in case 
    see CC2538's uart.c) */
    UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
    UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
    UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
    UART1->cc2538_uart_dr.ECR = 0xFF;
    UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
    UART1->cc2538_uart_lcrh.LCRH |= FEN;
    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;

    return retval;
}

int main(void)
{
    msg_t msg, msg_snd_pkt;
    int hdlc_send_locked = 0;
    int range_req = 0;
    uint32_t sound_rf_tdoa;
    uint32_t range_req_time;
    hdlc_pkt_t *recv_pkt;
    gnrc_pktsnip_t *gnrc_pkt;
    gnrc_pktsnip_t *snip;
    ipv6_addr_t sound_rf_sender_ip;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_pkt = { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    uart_pkt_hdr_t uart_hdr;
    gnrc_netreg_entry_t main_thr_server = { NULL, GET_SET_RANGING_THR_PORT, {thread_getpid()} };
    hdlc_entry_t main_thr = { NULL, GET_SET_RANGING_THR_PORT, thread_getpid() };

    msg_init_queue(main_msg_queue, sizeof(main_msg_queue));

    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
        "hdlc", UART_DEV(1));
    thread_create(rssi_dump_stack, sizeof(rssi_dump_stack), RSSI_DUMP_PRIO, NULL,
                  _rssi_dump, (void *) (uint32_t) hdlc_pid, "rssi_dump");

    hdlc_register(&main_thr);

    _set_hwaddr_short(ARREST_FOLLOWER_SHORT_HWADDR);
    _set_channel(main_channel);

    /* this thread handles set tx power, set channel, and ranging requests */
    while(1)
    {
        if(range_req) {
            uint32_t timeout = (uint32_t) (range_req_time + RANGE_REQ_TIMEO_USEC) 
                          - (uint32_t) xtimer_now().ticks32;
            if(timeout < 0) {
                range_req = 0;
                DEBUG("Ranging REQ timeout!\n");
                if(!hdlc_send_locked) {
                    uart_hdr.src_port = GET_SET_RANGING_THR_PORT;
                    uart_hdr.dst_port = ARREST_FOLLOWER_RANGE_THR_PORT;
                    uart_hdr.pkt_type = SOUND_RANGE_DONE;
                    uart_pkt_insert_hdr(hdlc_pkt.data, HDLC_MAX_PKT_SIZE, &uart_hdr);
                    hdlc_pkt.length = uart_pkt_cpy_data(hdlc_pkt.data, 
                        HDLC_MAX_PKT_SIZE, &sound_rf_tdoa, sizeof(uint32_t));
                    sound_rf_tdoa = 0;

                    msg_snd_pkt.type = HDLC_MSG_SND;
                    msg_snd_pkt.content.ptr = &hdlc_pkt;
                    msg_send(&msg_snd_pkt, hdlc_pid);
                    hdlc_send_locked = 1;
                    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &main_thr_server);
                } else {
                    DEBUG("Error: Already sending SOUND_RANGE_DONE.\n");
                    LED3_ON;
                }

                msg_receive(&msg);
            } else {
                if(0 > xtimer_msg_receive_timeout(&msg, timeout)) {
                    range_req = 0;
                    DEBUG("Ranging REQ timeout!\n");
                    uart_hdr.src_port = GET_SET_RANGING_THR_PORT;
                    uart_hdr.dst_port = ARREST_FOLLOWER_RANGE_THR_PORT;
                    uart_hdr.pkt_type = SOUND_RANGE_DONE;
                    uart_pkt_insert_hdr(hdlc_pkt.data, HDLC_MAX_PKT_SIZE, &uart_hdr);
                    hdlc_pkt.length = uart_pkt_cpy_data(hdlc_pkt.data, 
                        HDLC_MAX_PKT_SIZE, &sound_rf_tdoa, sizeof(uint32_t));
                    sound_rf_tdoa = 0;

                    msg_snd_pkt.type = HDLC_MSG_SND;
                    msg_snd_pkt.content.ptr = &hdlc_pkt;
                    msg_send(&msg_snd_pkt, hdlc_pid);
                    hdlc_send_locked = 1;
                    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &main_thr_server);
                }
            }
        } else {
            msg_receive(&msg);
        }
        DEBUG("received a msg!\n");

        switch (msg.type)
        {
            case HDLC_RESP_SND_SUCC:
                hdlc_send_locked = 0;
                break;
            case HDLC_RESP_RETRY_W_TIMEO:
                xtimer_usleep(msg.content.value);
                msg_send(&msg_snd_pkt, hdlc_pid);
                break;
            case HDLC_PKT_RDY:
                recv_pkt = (hdlc_pkt_t *)msg.content.ptr;
                uart_pkt_hdr_t recv_uart_hdr;
                uart_pkt_parse_hdr(&recv_uart_hdr, recv_pkt->data, recv_pkt->length);
                switch (recv_uart_hdr.pkt_type) 
                {
                    case RADIO_SET_CHAN:
                        DEBUG("Set channel request received\n");
                        _set_channel(recv_pkt->data[UART_PKT_DATA_FIELD]);
                        /* TODO: respond to mbed via RADIO_SET_CHAN_X msg */
                        break;
                    case RADIO_SET_POWER:
                        DEBUG("Set tx power request received.\n");
                        _set_tx_power(recv_pkt->data[UART_PKT_DATA_FIELD]);
                        /* TODO: respond to mbed via RADIO_SET_POWER_X msg */
                        break;
                    case SOUND_RANGE_REQ:
                        DEBUG("ranging: SOUND_RANGE_REQ received!\n");

                        gnrc_netreg_register(GNRC_NETTYPE_UDP, &main_thr_server);
                        if (ipv6_addr_from_str(&sound_rf_sender_ip, ARREST_LEADER_SOUNDRF_IPV6_ADDR) == NULL) {
                            DEBUG("Error: unable to parse destination address");
                            return 1;
                        }
                        range_req = 1;
                        range_req_time = xtimer_now().ticks32;
                        _sound_rf_ping_req(GET_SET_RANGING_THR_PORT, 
                            &sound_rf_sender_ip, ARREST_LEADER_SOUNDRF_PORT, 
                            ARREST_LEADER_SOUNDRF_ID);
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
                gnrc_pkt = msg.content.ptr;
                snip = gnrc_pktsnip_search_type(gnrc_pkt, GNRC_NETTYPE_UNDEF);
                if ( RANGE_RDY_FLAG == ((uint8_t *) snip->data)[0] && 
                     ARREST_LEADER_SOUNDRF_ID == ((uint8_t *)snip->data)[1] ) {
                    DEBUG("Got init msg. Turning on ranging mode.");
                    /* no need for anymore UDP packets. unregister. */
                    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &main_thr_server);
                    if (ipv6_addr_from_str(&sound_rf_sender_ip, ARREST_LEADER_SOUNDRF_IPV6_ADDR) == NULL) {
                        DEBUG("Error: unable to parse destination address");
                        return 1;
                    }
                    sound_rf_tdoa = _sound_rf_ping_go(GET_SET_RANGING_THR_PORT, 
                        &sound_rf_sender_ip, ARREST_LEADER_SOUNDRF_PORT, 
                        ARREST_LEADER_SOUNDRF_ID);
                    range_req = 0;  /* turnoff range req timeo */
                    range_rx_stop();

                    /* WARNING: if the time between two sound range requests to 
                    the openmote is too short, your TDoA values may be unusually
                    low. This may be because the ADC is not settling properly. */
                    
                    DEBUG("ranging: 'GO' done\n");
                } else {
                    DEBUG("Unknown packet.");
                    gnrc_pktbuf_release(gnrc_pkt);
                    break;
                }

                if(!hdlc_send_locked) {
                    DEBUG("ranging: sending SOUND_RANGE_DONE over UART\n");
                    uart_hdr.src_port = GET_SET_RANGING_THR_PORT;
                    uart_hdr.dst_port = ARREST_FOLLOWER_RANGE_THR_PORT;
                    uart_hdr.pkt_type = SOUND_RANGE_DONE;
                    uart_pkt_insert_hdr(hdlc_pkt.data, HDLC_MAX_PKT_SIZE, &uart_hdr);
                    hdlc_pkt.length = uart_pkt_cpy_data(hdlc_pkt.data, 
                        HDLC_MAX_PKT_SIZE, &sound_rf_tdoa, sizeof(uint32_t));
                    sound_rf_tdoa = 0;

                    msg_snd_pkt.type = HDLC_MSG_SND;
                    msg_snd_pkt.content.ptr = &hdlc_pkt;
                    msg_send(&msg_snd_pkt, hdlc_pid);
                    hdlc_send_locked = 1;
                } else {
                    DEBUG("Error: Already sending SOUND_RANGE_DONE.\n");;
                    LED3_ON;
                }

                gnrc_pktbuf_release(gnrc_pkt);
                break;
            default:
                /* error */
                LED3_ON;
                break;
        }
    }


    /* should be never reached */
    return 0;
}
