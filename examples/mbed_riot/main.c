/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
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
 * @file       tdma_master.c
 * @brief      Implements the master node for initializing TDMA-based localization anchors. 
 *
 * @author     Jason A. Tran <jasontra@usc.edu>
 * @author     Richard Kim <richartk@usc.edu>
 */

#include <inttypes.h>

#include "board.h"
#include "msg.h"
#include "net/gnrc.h"
#include "timex.h"
#include "xtimer.h"
#include "main-conf.h"
#include "anchor_node_table.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"

#define MAX_TX_POWER             7       //valid range: -27 to 7

#define MAIN_QUEUE_SIZE         (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

int main(void)
{
    msg_t msg;
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    gnrc_netif_get(ifs);
    gnrc_pktsnip_t *send_pkt, *recv_pkt, *hdr;
    uint8_t *src_l2addr;
    int src_l2addr_len;
    char l2_addr_str[GNRC_NETIF_HDR_L2ADDR_PRINT_LEN];
    int anchor_node_id = 1;
    uint8_t buf[6];      // Document up top = { 2byte flag, 1byte node_id, 2byte slot time (ms), 1byte tot_num_anchors }
    uint16_t tdma_slot_time_msec = (uint16_t) (TDMA_SLOT_TIME_USEC / 1000);
    // uint8_t total_num_anchors = TDMA_TOTAL_ANCHOR_NODES;
    uint8_t total_num_anchors = 3;

    gnrc_netreg_entry_t tdma_master_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, {thread_getpid()} };

    msg_init_queue(_main_msg_queue, sizeof(_main_msg_queue));

    uint16_t channel = TDMA_BOOTSTRAP_CHANNEL;
    uint16_t tx_power = MAX_TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));

    /* register for all l2 packets */
    gnrc_netreg_register(1, &tdma_master_serv);

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("Received pkt.\n");
                recv_pkt = msg.content.ptr;

                /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
                if (((uint8_t *)recv_pkt->data)[0] == ((TDMA_ANCHOR_ID_REQ_U16_FLAG >> 8) & 0xFF) &&
                    ((uint8_t *)recv_pkt->data)[1] == (TDMA_ANCHOR_ID_REQ_U16_FLAG & 0xFF)) {
                    recv_pkt = recv_pkt->next;
                    src_l2addr_len = gnrc_netif_hdr_get_srcaddr(recv_pkt, &src_l2addr);

                    // Use this if you have particular openmotes you want to use.
                    //------------------------------------------------------------------------//
                    // anchor_node_id = anchor_id_lookup(src_l2addr, (unsigned int) src_l2addr_len);

                    // if (anchor_node_id < 0) {
                    //     DEBUG("Anchor node unrecognized\n");
                    //     gnrc_pktbuf_release(recv_pkt);
                    //     break;
                    // }
                    //------------------------------------------------------------------------//
                    
                    xtimer_sleep(1); // did this for testing, goes too fast to check otherwise.
                    
                    DEBUG("Assigning %s anchor id %d\n", 
                          gnrc_netif_addr_to_str(l2_addr_str, sizeof(l2_addr_str),
                                                 src_l2addr, src_l2addr_len),
                          anchor_node_id);

                    buf[0] = TDMA_ANCHOR_ID_RESP_U16_FLAG >> 8 & 0xFF;
                    buf[1] = TDMA_ANCHOR_ID_RESP_U16_FLAG      & 0xFF; 
                    buf[2] = anchor_node_id;
                    buf[3] = tdma_slot_time_msec >> 8 & 0xFF;
                    buf[4] = tdma_slot_time_msec      & 0xFF;
                    buf[5] = total_num_anchors;

                    send_pkt = gnrc_pktbuf_add(NULL, &buf, sizeof(buf), GNRC_NETTYPE_UNDEF);
                    if (send_pkt == NULL) {
                        DEBUG("error: packet buffer full\n");
                        return 1;
                    }

                    /* TODO: sending the l2addr directly hasn't been tested. Not
                    sure if this works. Usually we convert a string into RIOT's
                    structure, whatever that is. Make sure this works first! */
                    // It works - Richard - 1/22/2018
                    hdr = gnrc_netif_hdr_build(NULL, 0, src_l2addr, src_l2addr_len);
                    if (hdr == NULL) {
                        DEBUG("error: packet buffer full\n");
                        gnrc_pktbuf_release(send_pkt);
                        return 1;
                    }

                    LL_PREPEND(send_pkt, hdr);

                    if (gnrc_netapi_send(ifs[0], send_pkt) < 1) {
                        DEBUG("error: unable to send\n");
                        gnrc_pktbuf_release(send_pkt);
                    }

                    // Otherwise, use this (it doesn't matter what the node is, since only
                    // anchor nodes will pass the tdma anchor id req flag test.)
                    // Just makes sure to change how many nodes you're using!
                    //------------------------------------------------------------------------//
                    anchor_node_id++;
                    //------------------------------------------------------------------------//
                }

                /* done with src_l2addr */
                gnrc_pktbuf_release(recv_pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* This thread will get all send l2 send requests, even if it's
                coming from this thread. Discard it */
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            default:
                DEBUG("msg.type = %d\n", msg.type);
                DEBUG("tdma_master: unknown msg_t!\n");
                break;
        }
    }

    /* should be never reached */
    return 0;
}
