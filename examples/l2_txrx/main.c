/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
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
 * @brief       Application to control ARREST leader (m3pi).
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/netapi.h"
#include "../mbed_riot/main-conf.h"
#include "xtimer.h"


#define SHELL_BUFSIZE       (128U)
#define UART_BUFSIZE        (128U)

#define PRINTER_PRIO        (THREAD_PRIORITY_MAIN - 1)

/* for the tx, keeping debug mode enabled is probably a good idea */
#define ENABLE_DEBUG        (1)
#include "debug.h"


#define MAX_TX_POWER         7       //valid range: -27 to 7
/* set default chan in makefile */

#define DEST_ADDR /*"00:12:4b:00:04:33:ed:2a"*/"df:2f"

static msg_t msg_queue[8];
static msg_t rx_msg_queue[16];

static char rx_thread_stack[THREAD_STACKSIZE_MAIN];

void *rx_thread(void *arg)
{
    DEBUG("Thread entered\n");
    msg_t msg;
    gnrc_pktsnip_t *pkt;
    gnrc_pktsnip_t *snip;
    gnrc_netif_hdr_t *hdr;
    msg_init_queue(rx_msg_queue, 16);
    uint8_t *src_addr;
    uint8_t src_len;
    gnrc_netreg_entry_t rx_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL,
        {thread_getpid()} };
    gnrc_netreg_register(1, &rx_serv);

    while(1)
    {
        msg_receive(&msg);
        DEBUG("Message received\n");
        if(msg.type == GNRC_NETAPI_MSG_TYPE_RCV)
        {
            pkt = msg.content.ptr;
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
            hdr = (gnrc_netif_hdr_t *)snip->data;
            src_addr = gnrc_netif_hdr_get_src_addr(hdr);
            DEBUG("%u %u\n", src_addr[0], src_addr[1]);
            src_len = hdr->src_l2addr_len;
            char addr_str[3*src_len];
            printf("Source: %s\n", gnrc_netif_addr_to_str(addr_str, 3*src_len,
                src_addr, src_len));
            gnrc_pktbuf_release(pkt);
        }
        else
            puts("Received something unexpected\n");
    }

    //Shouldn't be reached
    return NULL;
}

int main(void)
{
    //Needed so can see all of debug statements
    xtimer_sleep(5);
    int res;
    uint8_t hwaddr_long[8];
    uint8_t hwaddr_src[2];
    int hwaddr_src_len;
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];

    /* setup IPC queue for main thread*/
    msg_init_queue(msg_queue, 8);

    //Start up receiver thread
    thread_create(rx_thread_stack, sizeof(rx_thread_stack),
        THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, rx_thread, NULL,
        "rx_thread");

    /* get hwaddr and print for reference */
    size_t numof = gnrc_netif_get(ifs);
    if(numof != 1)
        DEBUG("Too many interfaces\n");
    DEBUG("PID: %d\n", (int)ifs[0]);
    res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, hwaddr_long, sizeof(hwaddr_long));
    char hwaddr_long_str[res * 3];
    DEBUG(" hwaddr_long: ");
    DEBUG("%s\n", gnrc_netif_addr_to_str(hwaddr_long_str, sizeof(hwaddr_long_str),
                                            hwaddr_long, res));

    hwaddr_src_len = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS, 0, hwaddr_src,
        sizeof(hwaddr_src));

    char hwaddr_src_str[hwaddr_src_len * 3];
    DEBUG("short addr: %s\n", gnrc_netif_addr_to_str(hwaddr_src_str, 
        sizeof(hwaddr_src_str), hwaddr_src, sizeof(hwaddr_src)));

    uint16_t channel = ARREST_DATA_CHANNEL;
    uint16_t tx_power = MAX_TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));

    uint8_t hwaddr_dest[2];
    size_t hwaddr_dest_len;
    gnrc_pktsnip_t *pkt, *hdr;

    hwaddr_dest_len = gnrc_netif_addr_from_str(hwaddr_dest, 
                       sizeof(hwaddr_dest), DEST_ADDR);
    //Use for debug
    /*gnrc_netreg_entry_t rx_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL,
        {thread_getpid()} };
    gnrc_netreg_register(1, &rx_serv);
    msg_t msg;
    gnrc_pktsnip_t *snip;
    gnrc_netif_hdr_t *hddr;
    uint8_t *src_addr;
    while(1)
    {
        msg_receive(&msg);
        printf("Message received\n");
        if(msg.type == GNRC_NETAPI_MSG_TYPE_SND)
        {
            printf("Received proper message\n");
            pkt = msg.content.ptr;
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
            hddr = (gnrc_netif_hdr_t *)snip->data;
            src_addr = gnrc_netif_hdr_get_src_addr(hddr);
            DEBUG("%u %u\n", src_addr[0], src_addr[1]);
            src_addr = gnrc_netif_hdr_get_dst_addr(hddr);
            DEBUG("%u %u\n", src_addr[0], src_addr[1]);
        }
    }*/
    /*while(1)
    {
        xtimer_sleep(1);
    }*/

    while (1) 
    {
        char c = 0xAA;

        xtimer_usleep(100000);
        
        /* put packet together */
        pkt = gnrc_pktbuf_add(NULL, &c, sizeof(c), GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            DEBUG("error: packet buffer full");
            continue;
        }

        hdr = gnrc_netif_hdr_build(hwaddr_src, hwaddr_src_len, hwaddr_dest,
            hwaddr_dest_len);
        if (hdr == NULL) {
            DEBUG("error: packet buffer full");
            gnrc_pktbuf_release(pkt);
            continue;
        }

        LL_PREPEND(pkt, hdr);

        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(ifs[0], pkt) < 1) {
            DEBUG("error: unable to send");
            gnrc_pktbuf_release(pkt);
            continue;
        }
        DEBUG("Sent packet!\n");
        /* RIOT CC2538 drivers mishandles extremely fast packets. Beaconer needs
        to slow down its rate a little. 200us is minimum, but still causes 
        errors. Currently set at 1200us to be safe. */
    }

    /* this should never be reached */
    return 0;
}