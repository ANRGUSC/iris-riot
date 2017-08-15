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
#define MAIN_QUEUE_SIZE     (8)

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


#define MAXSAMPLES_ONE_PIN            18000

#define RX_ONE_PIN                    GPIO_PIN(3, 3) //aka GPIO_PD3 - maps to DIO0
#define RX_TWO_PIN                    GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1

#define RX_LOGIC_PIN                  GPIO_PIN(3, 1) //aka GPIO_PD1 - maps to DIO2

#define TX_PIN                        GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1 //for usb openmote
//#define TX_PIN                      GPIO_PIN(3, 0) //aka GPIO_PD0 - maps to DIO3  //for regular openmote

#define ULTRSND_TIMEOUT               99000 //usec

static range_data_t* time_diffs;

static gpio_rx_line_t gpio_lines = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_LOGIC_PIN};



int range_rx(void)
{
    // TODO:
    // given arguments, need to set as arguments or hardcode.
    uint32_t timeout_usec;
    uint8_t range_mode;
    uint16_t num_samples;

    uint8_t mode = range_mode;
    //number of iterations in the gpio polling loop before calling it a timeout
    uint32_t maxsamps = MAXSAMPLES_ONE_PIN;

    if(gpio_init(TX_PIN, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    // clearing output for the ultrasonic sensor
    gpio_clear(TX_PIN);

    

    time_diffs = malloc(sizeof(range_data_t)*num_samples);
    
    uint32_t timeout = timeout_usec;
    if(timeout <= 0){
        DEBUG("timeout must be greater than 0");
        return 1;
    }


    sample1 = gpio_read(rx_line_array[0]);
    DEBUG("%d ", sample1);
    if(sample1 != 0){
        last2 = xtimer_now_usec();
        DEBUG("%lu - %lu ",last, last2);
        time_diffs.tdoa = last2 - last;
        range_rx_successful_stop();
        break;
    }





// some other code
    int i;
    for(i = 0; i < num_samples; i++){


        range_rx_init(TX_NODE_ID, thread_getpid(), gpio_lines, maxsamps, mode);


        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("RF ping missed\n");
            range_rx_stop();
            time_diffs[i] = (range_data_t) {0, 0, RF_MISSED};
            continue;
        }

        if(msg.type == RF_RCVD){
            if(xtimer_msg_receive_timeout(&msg, ULTRSND_TIMEOUT) < 0){
                DEBUG("Ultrsnd ping missed\n");
                range_rx_stop();
                time_diffs[i] = (range_data_t) {0, 0, ULTRSND_MISSED};
                continue;
            }
            if(msg.type == ULTRSND_RCVD){
                time_diffs[i] = *(range_data_t*) msg.content.ptr;
            } else{
                range_rx_stop();
                i--;
                continue;
            }

        }
    }

    DEBUG("tdoa: %d, orient_diff: %d, status: %d\n", 
            time_diffs.tdoa, time_diffs.orient_diff, time_diffs.status);
    return 0;
//some other code ends 






    // Radio
    //------------------------------------------------------------------------//
    msg_t msg;
    msg_init_queue(_main_msg_queue, sizeof(_main_msg_queue));
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 
    gnrc_pktsnip_t *recv_pkt, *hdr;
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
    //------------------------------------------------------------------------//

    while (1)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                // NODE receives ID packet from LEADER.
                DEBUG("Received RCV pkt.\n");
                recv_pkt = msg.content.ptr;
                // for(int x = 0; x < sizeof(recv_pkt->data); x++)
                // {
                //     DEBUG( ((uint8_t *)recv_pkt->data)[x] );
                // }
                DEBUG("%d, %d, %d", ((uint8_t *)recv_pkt->data)[0], ((uint8_t *)recv_pkt->data)[1], ((uint8_t *)recv_pkt->data)[2]);
                gnrc_pktbuf_release(recv_pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* This thread will get all send l2 send requests, even if it's
                coming from this thread. Discard it */
                DEBUG("Received GNRC_NETAPI_MSG_TYPE_SND pkt.\n");
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            default:
                // pkt received didn't match any of these.
                DEBUG("msg.type = %d\n", msg.type);
                break;
        }
    } //end of while loop, code should never reach here, etc.
    return 0;
}
