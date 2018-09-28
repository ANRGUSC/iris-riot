/**
 * Copyright (c) 2018, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
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
 * @file       listener_tdma/main.c
 * @brief      Implements a listener node for debugging TDMA-based localization anchors. 
 *
 * @author     Richard Kim <richartk@usc.edu>
 */
#define ENABLE_DEBUG (1) // This HAS to be before the include statement
#include "debug.h"

#include "range_param.h"
#include "range.h"

#define MAX_ADDR_LEN        (8U)
#define TX_PIN              GPIO_PIN(3, 2)
#define MAIN_QUEUE_SIZE     (8)

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

int main(void)
{
    DEBUG("Listening for RF and ultrasonic pings.\n");
// SETUP
    // Radio
    //------------------------------------------------------------------------//
    msg_t msg;
    msg_init_queue(_main_msg_queue, sizeof(_main_msg_queue));
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs);
    gnrc_pktsnip_t *send_pkt, *recv_pkt, *hdr;
    uint8_t *src_l2addr;
    int src_l2addr_len;
    // Document up top = { 2byte flag, 1byte node_id, 2byte slot time (ms), 1byte tot_num_anchors }
    uint8_t buf[3]; // buf is size 6 in master
    gnrc_netreg_entry_t tdma_slave_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, thread_getpid() };
    uint16_t channel = TDMA_BOOTSTRAP_CHANNEL;
    uint16_t tx_power = TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t)); // check channel
    /* there should be only one network interface on the board */
    if (numof == 1) 
    {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    // test code for getting the mac address
    uint8_t my_hw_addr[MAX_ADDR_LEN];
    size_t my_hw_addr_len;
    my_hw_addr_len = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS, 0, my_hw_addr, sizeof(my_hw_addr));
    char my_hw_addr_str[my_hw_addr_len * 3];
    gnrc_netif_addr_to_str(my_hw_addr_str, sizeof(my_hw_addr_str), my_hw_addr, my_hw_addr_len);
    DEBUG("HW_addr: %s\n", my_hw_addr_str);
    // end test code

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
    uint8_t anchor_node_id = 1;
    uint16_t tdma_slot_time_msec = 0;
    uint8_t total_num_anchors = 1;
    uint8_t incoming_id = 1;        // The id of the node that sent this ping. 
    uint32_t timeout_usec = 300000;
    uint8_t range_mode = 0x60;
    uint16_t num_samples = 10;
    range_data_t* data;
    //------------------------------------------------------------------------//

    // Preprocessing.
    //------------------------------------------------------------------------//
    // Change channel to 26 for the receiving node.
    channel = 26;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    //------------------------------------------------------------------------//

    data = range_rx(timeout_usec, range_mode, num_samples);

// use gpio read, check often enough to catch the pings

// MAIN LOOP
    while (true)
    {
        // msg_receive(&msg);
        if(msg_try_receive(&msg) > 0)
        {
            switch (msg.type)
            {
                case GNRC_NETAPI_MSG_TYPE_RCV:
                    DEBUG("Received RCV pkt.\n");
                    recv_pkt = msg.content.ptr;

                    // from node, to leader, on channel 11, the node init pkt
                    /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
                    if (((uint8_t *)recv_pkt->data)[0] == ((TDMA_ANCHOR_ID_REQ_U16_FLAG >> 8) & 0xFF) &&
                        ((uint8_t *)recv_pkt->data)[1] == (TDMA_ANCHOR_ID_REQ_U16_FLAG & 0xFF)) 
                    {
                        recv_pkt = recv_pkt->next;
                        src_l2addr_len = gnrc_netif_hdr_get_srcaddr(recv_pkt, &src_l2addr);

                        DEBUG("init pkt from anchor node %s\n", 
                              gnrc_netif_addr_to_str(l2_addr_str, sizeof(l2_addr_str), 
                                                     src_l2addr, src_l2addr_len));
                    }

                    // from leader, to node, on channel 11, the node id pkt
                    /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
                    if (((uint8_t *)recv_pkt->data)[0] == ((TDMA_ANCHOR_ID_RESP_U16_FLAG >> 8) & 0xFF) &&
                        ((uint8_t *)recv_pkt->data)[1] == (TDMA_ANCHOR_ID_RESP_U16_FLAG & 0xFF) )
                    {
                        // 2.2 NODE records its ID.
                        anchor_node_id = ((uint8_t *)recv_pkt->data)[2];
                        DEBUG("ID: %d\n", anchor_node_id);
                        
                        // 2.3. NODE syncs with the LEADER.
                        tdma_slot_time_msec &= 0x00;
                        tdma_slot_time_msec |= ((uint8_t *)recv_pkt->data)[3];
                        tdma_slot_time_msec = tdma_slot_time_msec << 8;
                        tdma_slot_time_msec |= ((uint8_t *)recv_pkt->data)[4];
                        DEBUG("Time slot: %d ms\n", tdma_slot_time_msec);

                        // 2.4. NODE records total number of nodes.
                        total_num_anchors = ((uint8_t *)recv_pkt->data)[5];
                        DEBUG("Total anchors: %d\n", total_num_anchors);
                    }

                    // from node, to anyone, on channel 26, the ranging packet
                    // #TODO get range_rx or however yutong gets the data
                    if ( ((uint8_t *)recv_pkt->data)[0] == RANGE_FLAG_BYTE0 &&
                         ((uint8_t *)recv_pkt->data)[1] == RANGE_FLAG_BYTE1 ) 
                    {
                        incoming_id = ((uint8_t *)recv_pkt->data)[2];
                        DEBUG("Heard %d\n", incoming_id);

                        DEBUG("Range data: %s", data);
                    }
                    break;
                case GNRC_NETAPI_MSG_TYPE_SND:
                    /* This thread will get all send l2 send requests, even if it's
                    coming from this thread. Discard it */
                    DEBUG("Received SND pkt.\n");
                    gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                    break;
                default:
                    DEBUG("Not a ranging packet.\n");
                    gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                    break;
            }
        }
    }
    DEBUG("-------------------------------------\n\n");
    //end of while loop, code should never reach here, etc.
    return 0;
}
