/**
 * Copyright (c) 2018, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran 
 * Pradipta Ghosh
 * Yutong Gu
 * Richard Kim
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
 * @brief       RIOT side slave code for chain follower demo.
 *
 * This program acts as the slave to a program running on another device.
 * This slave program relays incoming radio packets to another device and sends
 * radio packets upon requests. This slave program also handles TDoA ranging 
 * requests and TDoA beaconing requests from the master device. While the master
 * device and software on the master device can be arbitrary, we have built 
 * the master code and all supporting libraries/services in MBED-OS.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Yutong Gu <yutonggu@usc.edu>
 * @author      Richard Kim <richartk@usc.edu>
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "xtimer.h"
#include "periph/uart.h"
#include "net/hdlc.h"
#include "net/uart_pkt.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
// #include "net/gnrc/ipv6/hdr.h"
#include "range.h"
#include "dac.h"
#include "app-conf.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

/* HDLC thread priority should be higher than normal */ 
#define HDLC_PRIO           (THREAD_PRIORITY_MAIN - 1)
#define NETWORK_SLAVE_PRIO  (THREAD_PRIORITY_MAIN)
#define RANGE_SLAVE_PRIO    (THREAD_PRIORITY_MAIN)
#define BEACONER_SLAVE_PRIO (THREAD_PRIORITY_MAIN)

/* HDLC port numbers */
#define MAIN_THR_PORT       5000
#define NET_SLAVE_PORT      5001 
#define RANGE_SLAVE_PORT    5002
#define RANGE_BEACONER_PORT 5003
#define MBED_MAIN_PORT      6000
#define TX_PIN              GPIO_PIN(3, 2)
#define RANGE_RX_HW_ADDR    "ff:ff"

#define RANGE_BEACON_START  31
#define RANGE_BEACON_STOP   32
/* all UDP packets will be sent to port 8000 for this application */
#define FORWARD_TO_MBED_MAIN_PORT  8000

/* see openmote-cc2538's periph_conf.h for second UART pin config */

//setting the message queue with message structs
static msg_t main_msg_queue[16];
static msg_t network_slave_msg_queue[16];
static msg_t range_slave_msg_queue[16];

/* statically allocated thread stacks */
static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];
static char network_slave_stack[THREAD_STACKSIZE_MAIN];
static char range_slave_stack[THREAD_STACKSIZE_MAIN];

/**
 * @brief      function to send udp packets to a chosen port
 *
 * @param      addr_str  The address string
 * @param      port_str  The port string
 * @param      data      The data
 */
static int _net_slave_send(const char *addr_str, uint16_t port, void *data, size_t data_len)
{
    ipv6_addr_t addr;
    gnrc_pktsnip_t *payload, *udp, *ip;
        
    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        DEBUG("Error: unable to parse destination address");
        return -1;
    }

    /* parse port */
    if (port == 0) {
        DEBUG("Error: unable to parse destination port");
        return -1;
    }

    /* allocate payload */
    payload = gnrc_pktbuf_add(NULL, data, data_len, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        DEBUG("Error: unable to copy data to packet buffer");
        return -1;
    }

    /* allocate UDP header, set source port := destination port */
    udp = gnrc_udp_hdr_build(payload, port, port);
    if (udp == NULL) {
        DEBUG("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    /* allocate IPv6 header */
    ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
    if (ip == NULL) {
        DEBUG("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    /* send packet */
    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
        DEBUG("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    return 0;
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
 * Get channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static uint16_t _get_channel(void)
{
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    uint16_t channel;
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        gnrc_netapi_get(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
        return channel;
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


/* network slave thread used to support network requests (send/receive) from 
an mbed device */
static void *_network_slave(void *arg)
{
    msg_init_queue(network_slave_msg_queue, sizeof(network_slave_msg_queue));

    /* start network slave server which listens on port */
    gnrc_netreg_entry_t network_slave_server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);
    network_slave_server.target.pid = thread_getpid();
    network_slave_server.demux_ctx = (uint16_t) FORWARD_TO_MBED_MAIN_PORT;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &network_slave_server);

    /* pointer to the range_params recieved from the mbed */
    range_params_t *range_params;

    //Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in the mbed received pkt
    char *mbed_rcv_ptr;

    /* set hdlc_pid from input arg */
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;

    //regsiter network slave thread to HDLC 
    hdlc_entry_t network_slave = { NULL, NET_SLAVE_PORT, thread_getpid() };
    hdlc_register(&network_slave);

    msg_t msg_snd, msg_rcv;

    //stores the payload of incoming udp packets due to RIOT's link list structure
    //for incoming packets. using the entire buffer length would NOT be a good 
    //idea because you need space for other items
    char udp_payload[HDLC_MAX_PKT_SIZE];
    unsigned int udp_payload_size;

    /* initialize hdlc pkt buffer */
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = 0 };
    hdlc_pkt_t *hdlc_rcv_pkt;
    //creating two uart hdr structs
    uart_pkt_hdr_t uart_snd_hdr;
    uart_pkt_hdr_t uart_rcv_hdr;

    gnrc_pktsnip_t *gnrc_rcv_pkt;

    size_t data_payload_len;

    while(1)
    { 
        DEBUG("network_slave: waiting for a message\n");
        msg_receive(&msg_rcv);

        switch (msg_rcv.type)
        {
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* just in case */
                DEBUG("network_slave: release packet just in case\n");
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg_rcv.content.ptr);
                break;

            case GNRC_NETAPI_MSG_TYPE_RCV: 
                /* if UDP packet, pass to MBED */
                DEBUG("network_slave: recvd network packet. forwarding to"
                      "master mbed device.\n");

                gnrc_rcv_pkt = msg_rcv.content.ptr;

                //make a copy of the payload first
                memcpy(udp_payload, gnrc_rcv_pkt->data, gnrc_rcv_pkt->size);
                udp_payload_size = gnrc_rcv_pkt->size;

                printf("udp_payload: %d\n", udp_payload[0]);

                uart_snd_hdr.src_port = NET_SLAVE_PORT; 
                uart_snd_hdr.dst_port = MBED_MAIN_PORT; 
                uart_snd_hdr.pkt_type = NET_SLAVE_RECEIVE; 
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, 
                                    &uart_snd_hdr);

                /* we can't use uart_pkt_cpy_data() here, so we'll do this manually */

                /* insert ipv6 addr in string form to hdlc pkt buffer */
                char ipv6_addr[IPV6_ADDR_MAX_STR_LEN];
                gnrc_rcv_pkt = gnrc_pktsnip_search_type(gnrc_rcv_pkt, GNRC_NETTYPE_IPV6);
                ipv6_hdr_t *ipv6_hdr = gnrc_rcv_pkt->data;
                ipv6_addr_to_str(ipv6_addr, &ipv6_hdr->src, IPV6_ADDR_MAX_STR_LEN);

                /* insert the ipv6 addr string WITH the null character */
                memcpy(hdlc_snd_pkt.data + UART_PKT_DATA_FIELD, ipv6_addr, strlen(ipv6_addr) + 1);

                /* now we can insert the actual payload of the packet */
                gnrc_rcv_pkt = gnrc_pktsnip_search_type(gnrc_rcv_pkt, 
                                                        GNRC_NETTYPE_UNDEF);
                memcpy(hdlc_snd_pkt.data + UART_PKT_DATA_FIELD + (strlen(ipv6_addr) + 1), 
                       udp_payload, udp_payload_size);
                /* size of the hdlc packet payload will be the sum of all parts */
                hdlc_snd_pkt.length = UART_PKT_HDR_LEN + (strlen(ipv6_addr) + 1)
                                      + udp_payload_size;

                /* finally, send it to the hdlc thread it send it to the mbed */
                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;         
                printf("network_slave: packet forwarding to MBED\n");

                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("network_slave: HDLC msg queue full!\n");
                    // xtimer_usleep(HDLC_RTRY_TIMEO_USEC);
                }
                printf("network_slave: packet forwarding to MBED\n");

                gnrc_pktbuf_release(msg_rcv.content.ptr);
                break;

            case HDLC_RESP_SND_SUCC:
                // DEBUG("network_slave: sent frame_no %d!\n", frame_no++);                    
                break;

            case HDLC_RESP_RETRY_W_TIMEO:
                xtimer_usleep(msg_rcv.content.value);
                // DEBUG("network_slave: retrying frame_no %d\n", frame_no);
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("network_slave: HDLC msg queue full! retrying...\n");
                    xtimer_usleep(HDLC_RTRY_TIMEO_USEC);
                }
                break;

            case HDLC_PKT_RDY:
                /* received message from mbed */
                // DEBUG("network_slave: pkt recvd from mbed\n");
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;   
                uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;

                switch (uart_rcv_hdr.pkt_type)
                {
                    case NET_SEND_UDP:
                        DEBUG("network_slave: NET_SEND_UDP recvd from mbed\n");

                        /* count down bytes to determine actual data payload length */
                        data_payload_len = hdlc_rcv_pkt->length - UART_PKT_HDR_LEN;

                        /* the hdlc message from the mbed should have a 
                        null-terminated string at the beginning of the data payload */
                        char *ipv6_addr_str = mbed_rcv_ptr; //null-terminated string embedded message
                        // DEBUG("network_slave: %s\n",ipv6_addr_str);

                        /* continue counting down bytes: string length, null 
                        char, and the two bytes that hold the port number */
                        data_payload_len -= strlen(ipv6_addr_str) + 1 + 2; 

                        /* move pointer to port number */
                        while(*(mbed_rcv_ptr++) != '\0') {}

                        /* concatenate next 2 bytes (little endian) for port number */
                        uint16_t port = *mbed_rcv_ptr + (*(mbed_rcv_ptr+1) << 8); 
                        mbed_rcv_ptr += 2;

                        /* finally, send the udp/ipv6 packet */
                        _net_slave_send(ipv6_addr_str, port, mbed_rcv_ptr, data_payload_len);
                        break;
                    default:
                        LED2_ON;
                        break;
                } 

                hdlc_pkt_release(hdlc_rcv_pkt);
                break;

            default:
                //error 
                LED3_ON;
                break;
        } /* outer switch */
    } /* while */

    /* should be never reached */
    return 0;
    
}

void *_range_slave(void *arg)
{
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;
    msg_init_queue(range_slave_msg_queue, sizeof(range_slave_msg_queue));

    uart_pkt_hdr_t uart_hdr;
    hdlc_pkt_t *hdlc_pkt;
    range_params_t *range_params;
    uint16_t old_channel;
    msg_t  msg;

    //register range slave thread to HDLC 
    hdlc_entry_t range_slave = { NULL, RANGE_SLAVE_PORT, thread_getpid() };
    hdlc_register(&range_slave);

    /* no reply messages over hdlc are handled in this function's context. see
    range_and_send() to see the response via hdlc to range requests */
    while(1)
    { 
        DEBUG("range_slave: waiting for a range request\n");
        msg_receive(&msg);

        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                /* received message from mbed */
                DEBUG("range_slave: pkt recvd from mbed\n");
                hdlc_pkt = (hdlc_pkt_t *) msg.content.ptr;   
                uart_pkt_parse_hdr(&uart_hdr, hdlc_pkt->data, hdlc_pkt->length);

                if (uart_hdr.pkt_type == SOUND_RANGE_REQ) {
                    range_params = (range_params_t *)uart_pkt_get_data(hdlc_pkt->data, hdlc_pkt->length);

                    switch(range_params->ranging_mode)
                    {
                        case ONE_SENSOR_MODE:
                            DEBUG("******************ONE SENSOR MODE*******************\n");
                            break;
                        case TWO_SENSOR_MODE:
                            DEBUG("******************TWO SENSOR MODE*******************\n");
                            break;
                        case XOR_SENSOR_MODE:
                            DEBUG("******************XOR SENSOR MODE*******************\n");
                            break;
                        case OMNI_SENSOR_MODE:
                            DEBUG("******************OMNI SENSOR MODE******************\n");
                            break;
                        default:
                            DEBUG("Recieved an invalid ranging mode\n");
                            hdlc_pkt_release(hdlc_pkt);
                            continue;
                    }
                    old_channel = _get_channel();
                    DEBUG("Switching from channel %d to %d\n",old_channel, RSSI_LOCALIZATION_CHAN);
                    _set_channel(RSSI_LOCALIZATION_CHAN);
                    range_and_send(range_params, hdlc_pid, RANGE_SLAVE_PORT, uart_hdr.src_port);
                    _set_channel(old_channel);
                    DEBUG("Switching from channel %d to %d\n",RSSI_LOCALIZATION_CHAN, old_channel);
                    DEBUG("******************RANGING COMPLETED******************\n");
                } else {
                    LED2_ON;
                    DEBUG("range_slave: invalid HDLC packet\n");
                }

                hdlc_pkt_release(hdlc_pkt);
                break;

            default:
                LED3_ON; //error
                break;
        } 
    } /* while */

    /* this should never be reached */
}


/**
 * Only supports having one netif.
 * @param  rcvr_hwaddr     [description]
 * @param  rcvr_hwaddr_len [description]
 * @param  sender_node_id  [description]
 * @return                 [description]
 */
static int _send_beacons(uint8_t *rcvr_hwaddr, size_t rcvr_hwaddr_len, 
                                    uint8_t sender_node_id)
{
    gnrc_pktsnip_t *pkt, *hdr;
    msg_t msg;
    gnrc_netif_hdr_t *nethdr;
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    uint8_t flags = 0x00;

    /* turn off UART temporarily because of HDLC*/
    // UART1->cc2538_uart_ctl.CTLbits.UARTEN = 0;
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

    // UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
    // UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
    // UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
    // UART1->cc2538_uart_dr.ECR = 0xFF;
    // UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
    // UART1->cc2538_uart_lcrh.LCRH |= FEN;
    // UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;

    return 0;
}

static msg_t beaconer_slave_queue[16];
static char beaconer_slave_stack[THREAD_STACKSIZE_MAIN];
/* This thread deals with sound ranging */
static void *_beaconer_slave(void *arg) 
{
    msg_t msg;

    uint16_t old_channel;
    msg_init_queue(beaconer_slave_queue, sizeof(beaconer_slave_queue));
    hdlc_entry_t range_beaconer = { NULL, RANGE_BEACONER_PORT, thread_getpid() };
    hdlc_pkt_t *hdlc_pkt;
    hdlc_register(&range_beaconer);


    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);    size_t rcvr_hwaddr_len;
    if (hw_addr_len == 0) {
        DEBUG("error: invalid address given\n");
        return 1;
    }

    /* Turn off MB13XX ultrasonic sensor using the following pin */ 
    if(gpio_init(TX_PIN, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    gpio_clear(TX_PIN);

    /* max out tx power */
    _set_tx_power(7);

    uart_pkt_hdr_t uart_hdr;
       
    bool beaconing = false;    
    uint8_t beacon_node_id;
    uint32_t timeout_usec = 5000000;

    while(1)
    {
        if(xtimer_msg_receive_timeout(&msg,timeout_usec)<0){
            if(beaconing){
                DEBUG("Sending next beacon\n");
                old_channel = _get_channel();
                DEBUG("Switching from channel %d to %d\n",old_channel, RSSI_LOCALIZATION_CHAN);
                _set_channel(RSSI_LOCALIZATION_CHAN);
                range_tx_init(TX_PIN);
                _send_beacons(hw_addr, hw_addr_len, beacon_node_id);
                range_tx_off();
                _set_channel(old_channel);
                DEBUG("Switching from channel %d to %d\n",RSSI_LOCALIZATION_CHAN, old_channel);    
            }
            continue;            
        }

        switch (msg.type)
        {

            case HDLC_PKT_RDY:
                /* received message from mbed */
                DEBUG("range_slave: pkt recvd from mbed\n");
                hdlc_pkt = (hdlc_pkt_t *) msg.content.ptr;   
                uart_pkt_parse_hdr(&uart_hdr, hdlc_pkt->data, hdlc_pkt->length);

                if (uart_hdr.pkt_type == RANGE_BEACON_START){
                    DEBUG("Got GO. Time to send sound/rf ping!\n");
                    beaconing = true;
                    beacon_node_id = hdlc_pkt->data[UART_PKT_DATA_FIELD]; // TODO: Check whether this is correct
                    old_channel = _get_channel();
                    DEBUG("Switching from channel %d to %d\n",old_channel, RSSI_LOCALIZATION_CHAN);
                    _set_channel(RSSI_LOCALIZATION_CHAN);
                    range_tx_init(TX_PIN);
                    _send_beacons(hw_addr, hw_addr_len, beacon_node_id);
                    range_tx_off();
                    _set_channel(old_channel);
                    DEBUG("Switching from channel %d to %d\n",RSSI_LOCALIZATION_CHAN, old_channel);
                    
                }
                else if (uart_hdr.pkt_type == RANGE_BEACON_STOP){
                    beaconing = false;
                    DEBUG("Time to Stop Beaconing!\n");

                }
                else{
                    DEBUG("Unknown Packet type!\n");
                }
                hdlc_pkt_release(hdlc_pkt);
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
    uint8_t hwaddr_long[8];

    DEBUG("starting DAC\n");
    if (init_dac(DEFAULT_DAC_CS, SPI_CLK_400KHZ) == SPI_OK) {
        DEBUG("Setting voltage at %d%%\n", DEFAULT_SENSOR_THRESH * 100 / 255);
        set_voltage((uint8_t)DEFAULT_SENSOR_THRESH, DAC_GAIN_1);
        stop_dac();
    } else {
        DEBUG("SPI failed\n");
    }

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    gnrc_netif_get(ifs);  //get hardware address
    int res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, hwaddr_long, 
                          sizeof(hwaddr_long));
    char hwaddr_long_str[res * 3];
    gnrc_netif_addr_to_str(hwaddr_long_str, sizeof(hwaddr_long_str), 
                           hwaddr_long, res);
    
    /* init main thread's queue for rpc messages */
    msg_init_queue(main_msg_queue, sizeof(main_msg_queue));

    /* register main thread to hdlc */
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, thread_getpid() };
    hdlc_register(&main_thr);

    /* start hdlc thread */
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(1));
    
    /* start network slave thread */
    thread_create(network_slave_stack, sizeof(network_slave_stack), NETWORK_SLAVE_PRIO, 
            THREAD_CREATE_STACKTEST, _network_slave, (void *)hdlc_pid, "network_slave");

    /* start range slave thread */
    thread_create(range_slave_stack, sizeof(range_slave_stack), RANGE_SLAVE_PRIO, 
            THREAD_CREATE_STACKTEST, _range_slave, (void *)hdlc_pid, "range_slave");
    
    /* start the beaconer slave thread */
    thread_create(beaconer_slave_stack, sizeof(beaconer_slave_stack), BEACONER_SLAVE_PRIO, 0,
                  _beaconer_slave, (void *) 0, "beaconer_slave");

    msg_t msg_snd, msg_rcv;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    uart_pkt_hdr_t uart_hdr;


    /* main thread: serves as the beaconer slave thread */
    int exit = 0;

    while(1)
    {        
        while(1)
        {
            msg_receive(&msg_rcv);
            DEBUG("Waiting for a message\n");
            switch (msg_rcv.type)
            {
                case HDLC_RESP_SND_SUCC:
                    DEBUG("main_thr: sent frame_no %d!\n", frame_no++);
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    DEBUG("main_thr: retrying frame_no %d\n", frame_no);
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("main_thr: HDLC msg queue full!\n");
                        /* put back into thread's queue */
                        msg_send_to_self(&msg_rcv);
                    }
                    break;
                case HDLC_PKT_RDY:
                    // hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                    // DEBUG("The \n packet \n has \n been received\n");  
                    // main_mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;
                    // main_rcv_pkt = (mqtt_pkt_t *)main_mbed_rcv_ptr;
                    // DEBUG("The data received is %s \n", main_rcv_pkt->data);
                    // DEBUG("The topic received is %s \n", main_rcv_pkt->topic);
                    // hdlc_pkt_release();
                    break;
                default:
                    //error
                    LED3_ON;
                    break;
            }

            if(exit) {
                exit = 0;
                break;
            }
        }


    }
    //should be never reached 
    
    return 0;
}

