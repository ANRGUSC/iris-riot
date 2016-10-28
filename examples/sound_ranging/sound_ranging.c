/*
 * Copyright (C) 2016 Autonomous Networks Research Group
 *                     University of Southern California
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Simple program example to test ultrasound ranging.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "net/netdev2.h"
#include "timex.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/adc.h"

#include "thread.h"
#include "msg.h"

#define TX_POWER            7
#define TX_NODE_IPV6_ADDR   "fe80::212:4b00:433:ed81"
#define RX_NODE_IPV6_ADDR   "fe80::212:4b00:433:ed4f"
#define CLIENT_PORT         8000
#define SERVER_PORT         8888

#define MAX_ADDR_LEN        (8U)
#define RANGE_RX_HW_ADDR    "ff:ff"
#define QUEUE_SIZE          8
#define CC2538_RSSI_OFFSET  73

#define RANGE_REQ_FLAG      0x12
#define RANGE_RDY_FLAG      0x34
#define RANGE_GO_FLAG       0x56
#define TX_NODE_ID 0x00

static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
                                        KERNEL_PID_UNDEF};

/* this is used to unregister the thread from receiving UDP packets sent to the 
   port in the "server" struct */
static void _unregister_thread(void)
{
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.pid = KERNEL_PID_UNDEF;
}
/*----------------------------------------------------------------------------*/
int range_rx(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <ultrasound_thresh>\n", argv[0]);
        return 1;
    }

    if (atoi(argv[1]) <= 0)
    {
        puts("error: please input value greater than 0");
        return 1;
    }

    int ultrasound_thresh = atoi(argv[1]);
    unsigned long time_diff = 0;
    char *tx_node_addr_str = TX_NODE_IPV6_ADDR;
    ipv6_addr_t tx_node_ip_addr;
    gnrc_pktsnip_t *pkt, *snip;
    int16_t tx_power = TX_POWER;
    gnrc_pktsnip_t *payload, *udp, *ip;
    char buf[2] = {0x00, 0x00};

    /* register this thread to the chosen UDP port */
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    if (ipv6_addr_from_str(&tx_node_ip_addr, tx_node_addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }

    /* send ultrasound ranging request */
    buf[0] = RANGE_REQ_FLAG;
    buf[1] = TX_NODE_ID;
    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, CLIENT_PORT, SERVER_PORT);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, &tx_node_ip_addr);
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

    /* wait for "RDY" packet */
    while(1) {   
        puts("Waiting for RDY pkt.");
        msg_receive(&msg);

        if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
            pkt = msg.content.ptr;

            /* get snip containing packet data where we put the packet number */
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
            if ( RANGE_RDY_FLAG == ((uint8_t *) snip->data)[0] && 
                 TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                puts("Got init msg. Turning on ranging mode.");
                break;
            } else {
                puts("Unknown packet.");
            }

            gnrc_pktbuf_release(pkt);
        } 
    } /* while */

    /* send "GO" packet */
    buf[0] = RANGE_GO_FLAG;
    buf[1] = TX_NODE_ID;
    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, CLIENT_PORT, SERVER_PORT);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, &tx_node_ip_addr);
    if (ip == NULL) {
        puts("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    range_rx_init(TX_NODE_ID, ultrasound_thresh, AD4_PIN, ADC_RES_7BIT, 2000);

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, 
        ip)) {
        puts("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    xtimer_sleep(1);

    if( (time_diff = range_rx_stop()) > 0 ) {
        printf("TDoA = %lu\n", time_diff);
    } else {
        puts("Ranging failed.");
    }

    _unregister_thread();

    return 0;
}
/*----------------------------------------------------------------------------*/
int range_tx(int argc, char **argv)
{
    /* for sending L2 pkt */
    kernel_pid_t dev;
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    gnrc_pktsnip_t *pkt, *hdr, *snip;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;    

    /* for sending udp pkt */
    char *rx_node_addr_str = RX_NODE_IPV6_ADDR;
    char buf[3] = {0x00, 0x00, 0x00};
    ipv6_addr_t rx_node_ip_addr;
    int16_t tx_power = TX_POWER;
    gnrc_pktsnip_t *payload, *udp, *ip;

    /* enable output on Port D pin 3 */
    if(gpio_init(GPIO_PD3, GPIO_OUT) < 0) {
        puts("Error initializing GPIO_PIN.");
        return 1;
    }

    gpio_clear(GPIO_PD3);

    /* register this thread to the chosen UDP port */
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        /* FIXME: mising error checks for both */
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    if (ipv6_addr_from_str(&rx_node_ip_addr, rx_node_addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }

    /* ultrasound transmitter is always ready for request (infinite loop) */
    while(1) {

start: 

        /* wait until for ranging request packet */
        while(1) {
            msg_receive(&msg);

            if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
                pkt = msg.content.ptr;

                /* get snip containing packet data where we put the packet number */
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
                if ( RANGE_REQ_FLAG == ((uint8_t *)snip->data)[0] && 
                        TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                    puts("Got REQ. Sending 'RDY' pkt now!");
                    break;
                } else {
                    puts("Not a ranging request packet.");
                }

                gnrc_pktbuf_release(pkt);
            }
        }

        /* send "READY" message */
        buf[0] = RANGE_RDY_FLAG;
        buf[1] = TX_NODE_ID;
        payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return 1;
        }

        udp = gnrc_udp_hdr_build(payload, (uint16_t) CLIENT_PORT, (uint16_t) SERVER_PORT);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return 1;
        }

        ip = gnrc_ipv6_hdr_build(udp, NULL, &rx_node_ip_addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return 1;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return 1;
        }

        /* wait for "GO" packet */
        int retries = 0;

        while(1){
            msg_receive(&msg);

            if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
                pkt = msg.content.ptr;
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);

                if ( RANGE_GO_FLAG == ((uint8_t *)snip->data)[0] && 
                        TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                    puts("Got 'GO' pkt. Time to send RF/Ultrasound ping!");
                    break;
                } else if(retries > 2) {
                    /* for testing purposes */
                    puts("No valid packets. Resetting...");
                    gnrc_pktbuf_release(pkt);
                    goto start;
                } else {
                    puts("Not a ranging request packet.");
                }

                gnrc_pktbuf_release(pkt);
            }

            ++retries;
        }

        range_tx_init(GPIO_PD3);

        /** Send L2 Packet **/
        /* network interface */
        dev = ifs[0];
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);

        if (hw_addr_len == 0) {
            if (strcmp(argv[2], "bcast") == 0) {
                flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
            } else {
                puts("error: invalid address given");
                return 1;
            }
        }

        /* put packet together */
        buf[0] = RANGE_FLAG_BYTE0;
        buf[1] = RANGE_FLAG_BYTE1;
        buf[2] = TX_NODE_ID;
        pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            puts("error: packet buffer full");
            return 1;
        }
       
        hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
        if (hdr == NULL) {
            puts("error: packet buffer full");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        LL_PREPEND(pkt, hdr);
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        /* ready to send */

        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(dev, pkt) < 1) {
            puts("error: unable to send");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        
        range_tx_off(); //turn off just in case
        puts("RF and ultrasound pings sent");        
    } /* end while (infinite loop) */

    /* should never reach here! */
    _unregister_thread();
    return 0; 
}

