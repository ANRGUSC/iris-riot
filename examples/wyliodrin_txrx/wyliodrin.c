
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
 * @brief       Simple program example to test channel statistics
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
#include "timex.h"
#include "xtimer.h"

#include "thread.h"
#include "msg.h"

/* define this so you can compile and test via the macros below instead of
   manually typing it into the RIOT shell everytime */ 
// #define TEST_MODE

/* hard code settings here so you do not have to type it in everytime! */
#ifdef TEST_MODE 
/* valid range (dBm): -27 to 7 */
#define TX_POWER 7
/* valid range: 11 to 26 */ 
#define DEFAULT_CHAN 12
#define IPV6_ADDR "fe80::212:4b00:433:ed81" // hard coded address?
#define PORT "8888"
#endif

#define QUEUE_SIZE 8
#define CC2538_RSSI_OFFSET 73

static gnrc_netreg_entry_t receiver = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
                                        {KERNEL_PID_UNDEF}};

/* this is used to unregister the thread from receiving UDP packets sent to the 
   port in the "receiver" struct */
static void _unregister_thread(void)
{
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &receiver);
    //receiver.pid = KERNEL_PID_UNDEF;
    receiver.target.pid = KERNEL_PID_UNDEF;
}

int wyliodrin_tx(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: %s <num_pkts> <interval_in_us> [<ipv6_addr> <port>]\n", 
            argv[0]);
        return 1;
    }

    if (atoi(argv[1]) <= 0 || atoi(argv[2]) <= 0)
    {
        printf("error: please input values greater than 0");
        return 1;
    }

    unsigned int num_pkts = atoi(argv[1]);
    uint32_t interval = (uint32_t)atoi(argv[2]);
    ipv6_addr_t addr;
    uint16_t port;

    char *addr_str;
    char *port_str;

#ifdef TEST_MODE
    if (argc > 3) {
        printf("usage: %s <num_pkts> <interval in us>\n", argv[0]);
        return 1;
    }
    addr_str = IPV6_ADDR;
    port_str = PORT;
    int16_t tx_power = TX_POWER;
    uint16_t channel = DEFAULT_CHAN;

    /* this is the best way I could find setting network iface properties */
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 
    /* there should be only one network interface on the board */
    if (numof == 1) {
        /* FIXME: mising error checks for both */
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
        gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    }
#else
    addr_str = argv[3];
    port_str = argv[4];
#endif

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }
    /* parse port */
    port = (uint16_t)atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return 1;
    }

    uint8_t pkt_num = 1;

    for (unsigned int i = 0; i < num_pkts; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        unsigned payload_size;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, &pkt_num, sizeof(pkt_num), GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return 1;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return 1;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
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
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        printf("Success: send %u byte to [%s]:%u\n", payload_size, addr_str,
               port);

        pkt_num++;
        xtimer_usleep(interval);
    }

    printf("Done!\n");

    return 0; 
}

/* the rx mode only uses <num_pkts> and <interval_in_us> values for calculation */
int wyliodrin_rx(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <num_pkts> <interval_in_us> <port>\n", argv[0]);
        return 1;
    }

    if (atoi(argv[1]) <= 0 || atoi(argv[2]) <= 0)
    {
        printf("error: please input values greater than 0");
        return 1;
    }

    unsigned int num_pkts = atoi(argv[1]);
    unsigned int interval = atoi(argv[2]);
    uint32_t port = atoi(argv[3]);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    if (port == 0) {
        puts("Error: unable to parse destination port");
        return 1;
    }

    /* register this thread to the chosen UDP port */
    receiver.next = NULL;
    receiver.demux_ctx = port; 
    //receiver.pid = thread_getpid();
    receiver.target.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &receiver);
    gnrc_pktsnip_t *pkt;
    gnrc_pktsnip_t *snip;
    gnrc_netif_hdr_t *hdr;

    /* variables for rssi statistics */
    float rssi_mean = 0;
    float delta = 0;
    float diff_squared_sum = 0;

    printf("wyliodrin_rx: waiting for first packet...\n");
    /* this function blocks. use msg_try_receive(&msg) for non-blocking. */
    msg_receive(&msg);

    unsigned int pkt_cnt = 0;
    for (pkt_cnt = 1; pkt_cnt <= num_pkts; pkt_cnt++) {
        if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
            pkt = msg.content.ptr;
            puts("wyliodrin_rx: data received:");

            /* get snip containing packet data where we put the packet number */
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
            /* For some reason, RIOT sometimes prints a large number here such
               such as 
            printf("received packet number %u\n", *(unsigned int *) snip->data); 
            */
            /* get snip containing network interface header (includes rssi)*/
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
            hdr = snip->data;

            /* Note: the RSSI value printed here is the platform specific raw 
               value from the netif. Make sure to adjust the RSSI value to get a
               more meaningful number (see below for CC2538-specific info). 
               Note: there is a bug with the ARM compiler where "PRIu8" is 
               translated to "hhu" during compilation. Thus, to see values 
               instead of just "hu" when the netif header is printed, go to
               gnrc_netif_hdr_print.c and change the rssi and lqi printf's to
               printf("rssi: %u ", hdr->rssi);
               printf("lqi: %u"\n", hdr->lqi);
               These are left unchanged in this branch because the ARM compiler
               may be fixed overtime...
             */
            gnrc_netif_hdr_print(hdr);

            /* You can also get/print ipv6 data via ipv6_hdr_print() similar to 
               gnrc_netif_hdr_print(). See gnrc_pktdump.c as an example.
             */

            /* The current CC2538 RF driver uses rfcore_read_byte() to read the
               RSSI value which returns an uint_fast8_t when the RSSI register 
               is an 8-bit signed value. Casting an unsigned to a signed is 
               technically undefined in the C standard, but often works with
               most compilers. Assuming this is safe to do, all you need is to
               subtract an offset of 73 from the value read to get the the RSSI
               in dBm */
            /* online algorithm for variance and mean calculation (Welford):
               https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
             */
            delta = ((float) hdr->rssi - CC2538_RSSI_OFFSET) - rssi_mean;
            rssi_mean += delta / (float) pkt_cnt;
            diff_squared_sum += delta * (((float) hdr->rssi - CC2538_RSSI_OFFSET) - rssi_mean);
            printf("delta = %f, rssi_mean (dBm) = %f, diff_squared_sum = %f\n", delta, rssi_mean, diff_squared_sum);

            /* Tell GNRC you are done with this packet so it can release the memory */
            gnrc_pktbuf_release(pkt);

            /* Block while waiting for a packet with timeout based on experiment */
            if (xtimer_msg_receive_timeout(&msg, (uint32_t) ((num_pkts - pkt_cnt + 1) * interval)) < 0) {
                printf("wyliodrin_rx: timeout, experiment stopped\n");
                printf("rssi: mean = %f dBm, variance = %f\n", rssi_mean, diff_squared_sum / (float) (pkt_cnt - 1));
                printf("packet reception rate: %f \n", (float) pkt_cnt / (float) num_pkts);
                _unregister_thread();
                return 1;
            }
        }
        else {
            puts("wyliodrin_rx: received something unexpected");
        }
    }

    /* Stop this thread from receiving anymore packets */
    _unregister_thread();

    /* Note: there is no error checking here */
    printf("wyliodrin_rx: experiment complete\n");
    printf("rssi: mean = %f, variance = %f\n", rssi_mean, diff_squared_sum / (float) (pkt_cnt - 1));
    printf("packet reception rate: %f\n", (float) pkt_cnt / (float) num_pkts);

    return 0;
}