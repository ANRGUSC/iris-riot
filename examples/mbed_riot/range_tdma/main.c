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
 * Yutong Gu
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
 * @file       range.c
 * @brief      Implements the follower nodes for TDMA-based localization anchors. 
 *
 * @author     Jason A. Tran <jasontra@usc.edu>
 * @author     Richard Kim <richartk@usc.edu>
 */
#include "range_param.h"
#include "range.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define MAX_ADDR_LEN        (8U)
#define TX_PIN              GPIO_PIN(3, 2)
#define MAIN_QUEUE_SIZE     (8)
#define MAX_TX_POWER         7

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

// int range_tx_tdma(void)
int main(void)
{
    DEBUG("Running range_tx_tdma.\n");
// SETUP
    // Radio
    //------------------------------------------------------------------------//
    msg_t msg;
    msg_init_queue(_main_msg_queue, sizeof(_main_msg_queue));
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 
    gnrc_pktsnip_t *send_pkt, *recv_pkt, *hdr;
    uint8_t anchor_node_id;
    // Document up top = { 2byte flag, 1byte node_id, 2byte slot time (ms), 1byte tot_num_anchors }
    uint8_t buf[3]; // buf is size 6 in master
    uint16_t tdma_slot_time_msec = 0;
    uint8_t total_num_anchors;
    gnrc_netreg_entry_t tdma_slave_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, thread_getpid() };
    uint16_t channel = TDMA_BOOTSTRAP_CHANNEL;
    uint16_t tx_power = TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t)); // check channel
    /* there should be only one network interface on the board */
    if (numof == 1) 
    {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }
    /* register for all l2 packets */
    gnrc_netreg_register(1, &tdma_slave_serv);
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
    gnrc_netif_hdr_t *nethdr;
    // Broadcasting flag setup.
    uint8_t flags = 0x00;    
    //------------------------------------------------------------------------//

    // Ultrasonic sensor
    //------------------------------------------------------------------------//
    /* enable output on Port D pin 3 */
    if (gpio_init(TX_PIN, GPIO_OUT) < 0) 
    {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    // Clearing output for the ultrasonic sensor.
    gpio_clear(TX_PIN);
    //------------------------------------------------------------------------//
    
    // Miscellaneous
    //------------------------------------------------------------------------//
    bool wait_for_id = true;
    xtimer_ticks32_t go_time; // go_time = time at which openmote sends ranging.
    uint32_t id_delay_time;
    uint8_t incoming_rank;
    // NETOPT_ADDRESS; // this device's hw_addr 
    //------------------------------------------------------------------------//

    DEBUG("Sending REQ packet...");
    // NODE sends REQ packet to LEADER.
    //------------------------------------------------------------------------//
    buf[0] = TDMA_ANCHOR_ID_REQ_U16_FLAG >> 8 & 0xFF;
    buf[1] = TDMA_ANCHOR_ID_REQ_U16_FLAG & 0xFF;
    buf[2] = 0x00;

    send_pkt = gnrc_pktbuf_add(NULL, &buf, sizeof(buf), GNRC_NETTYPE_UNDEF);
    if (send_pkt == NULL) 
    {
        DEBUG("error: packet buffer full\n");
        return 1;
    }

    hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
    if (hdr == NULL) 
    {
        DEBUG("error: packet buffer full\n");
        gnrc_pktbuf_release(send_pkt);
        return 1;
    }

    LL_PREPEND(send_pkt, hdr);

    flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags = flags;

    if (gnrc_netapi_send(ifs[0], send_pkt) < 1) 
    {
        DEBUG("error: unable to send\n");
        gnrc_pktbuf_release(send_pkt);
    }
    //------------------------------------------------------------------------//
    DEBUG("Done.\n");

    DEBUG("Waiting for ID packet...\n");
    // NODE A waits for ID packet from LEADER.
    //------------------------------------------------------------------------//
    wait_for_id = true;
    while (wait_for_id)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            // NODE receives ID packet from LEADER.
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("Received RCV pkt.\n");
                recv_pkt = msg.content.ptr;

                /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
                if (((uint8_t *)recv_pkt->data)[0] == ((TDMA_ANCHOR_ID_RESP_U16_FLAG >> 8) & 0xFF) &&
                    ((uint8_t *)recv_pkt->data)[1] == (TDMA_ANCHOR_ID_RESP_U16_FLAG & 0xFF) )
                {
                    DEBUG("Was ID packet.\n");
                    // NODE records its ranking.
                    anchor_node_id = ((uint8_t *)recv_pkt->data)[2];
                    DEBUG("Rank: ");
                    DEBUG("%d\n", anchor_node_id);
                    // NODE syncs with the LEADER.
                    tdma_slot_time_msec &= 0x00;
                    tdma_slot_time_msec |= ((uint8_t *)recv_pkt->data)[3];
                    tdma_slot_time_msec = tdma_slot_time_msec << 8;
                    tdma_slot_time_msec |= ((uint8_t *)recv_pkt->data)[4];

                    // NODE records total number of nodes (useful for NODE 1).
                    total_num_anchors = ((uint8_t *)recv_pkt->data)[5];

                    // NODE A received the data correctly and now breaks out of the loop.
                    wait_for_id = false;
                }
                gnrc_pktbuf_release(recv_pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* This thread will get all send l2 send requests, even if it's
                coming from this thread. Discard it */
                DEBUG("Received SND pkt (1).\n");
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            default:
                // pkt received didn't match any of these.
                DEBUG("msg.type = %d\n", msg.type);
                DEBUG("Waiting for ID packet from LEADER. PKT was none of these.\n");
                break;
        }
    }
    //------------------------------------------------------------------------//

// MAIN LOOP
    while (true)
    {
        DEBUG("Waiting to go.\n");
        // NODE A received ID packet from LEADER and is now waiting to go.
        //--------------------------------------------------------------------//
        DEBUG("Start time: %u\n", xtimer_now().ticks32);
        go_time.ticks32 = xtimer_now().ticks32 + anchor_node_id * tdma_slot_time_msec;
        DEBUG("Go time: %u\n", go_time.ticks32);
        while (xtimer_now().ticks32 < go_time.ticks32)
        {
            msg_receive(&msg); // make this a thread? or make the timer a thread?
            switch (msg.type)
            {
                case GNRC_NETAPI_MSG_TYPE_RCV:
                    DEBUG("Received pkt.\n");
                    recv_pkt = msg.content.ptr;
                    if ( ((uint8_t *)recv_pkt->data)[0] == RANGE_FLAG_BYTE0 &&
                         ((uint8_t *)recv_pkt->data)[1] == RANGE_FLAG_BYTE1 ) 
                    {
                        DEBUG("Another anchor node ranging packet.\n");
                        incoming_rank = ((uint8_t *)recv_pkt->data)[1];
                        // NODE's rank is 1 and listening for the last NODE.
                        if ( (incoming_rank == total_num_anchors) && (anchor_node_id == 1))
                        {
                            DEBUG("last rank to first.\n");
                            go_time.ticks32 = xtimer_now().ticks32 + (uint32_t)tdma_slot_time_msec;
                        }
                        // NODE's rank is higher than the incoming NODE's transmission.
                        if (incoming_rank < anchor_node_id)
                        {
                            DEBUG("lower rank to higher.\n");
                            id_delay_time = (uint32_t) ((anchor_node_id - incoming_rank) * tdma_slot_time_msec);
                            go_time.ticks32 = xtimer_now().ticks32 + id_delay_time;
                        }
                    }
                    gnrc_pktbuf_release(recv_pkt);
                    break;
                case GNRC_NETAPI_MSG_TYPE_SND:
                    /* This thread will get all send l2 send requests, even if it's
                    coming from this thread. Discard it */
                    DEBUG("Received SND pkt (2).\n");
                    gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                    break;
                default:
                    DEBUG("Not a ranging packet.\n");
                    break;
            }
        }
        //--------------------------------------------------------------------//
        DEBUG("Waited to go.\n");

        DEBUG("Sending signal...");
        // ACUTALLY SENDING THE SIGNAL GOD FINALLY
        // TODO: Modify to not block the interrupts
        //--------------------------------------------------------------------//
        /** Send L2 Packet **/
        /* network interface */
        // need to change this to hit anything
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);

        /* put packet together */
        buf[0] = RANGE_FLAG_BYTE0;
        buf[1] = RANGE_FLAG_BYTE1;
        buf[2] = anchor_node_id;

        send_pkt = gnrc_pktbuf_add(NULL, &buf, sizeof(buf), GNRC_NETTYPE_UNDEF);
        if (send_pkt == NULL) {
            DEBUG("error: packet buffer full\n");
            return 1;
        }

        hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
        if (hdr == NULL) {
            DEBUG("error: packet buffer full\n");
            gnrc_pktbuf_release(send_pkt);
            return 1;
        }

        LL_PREPEND(send_pkt, hdr);

        flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        /* ready to send */
        
        // change channel to 26 for the receiving node.
        channel = 26;
        gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));

        // initializing the ultrasonic speaker
        range_tx_init(TX_PIN);

        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(ifs[0], send_pkt) < 1) {
            DEBUG("error: unable to send\n");
            gnrc_pktbuf_release(send_pkt);
            return 1;
        }   
        DEBUG("RF and ultrasound pings sent\n");
        xtimer_usleep(100000); // Delay window of 100 ms ()
        /*
  [Node 1 RF]          [Node 1 ping]                           [Node 1 ping end]
        |--------------------|----------------------------------------|----------...
        |--------------------|----------------------------------------|----------...|
                20 ms                           45 ms                      55 ms     (100 ms total)
        */
        //--------------------------------------------------------------------//
    }
    range_tx_off(); //turn off just in case
    //end of while loop, code should never reach here, etc.
    return 0;
}
