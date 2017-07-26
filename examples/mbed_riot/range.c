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
 * @author      Yutong Gu <yutonggu@usc.edu>
 */
#include "range.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
/*----------------------------------------------------------------------------*/
int range_tx(void)
{
// SETUP
    // Radio
    //------------------------------------------------------------------------//
    msg_t msg;
    msg_init_queue(main_msg_queue, sizeof(_main_msg_queue));
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 
    gnrc_pktsnip_t *send_pkt, *recv_pkt, *hdr;
    uint8_t *src_l2addr;
    int src_l2addr_len;
    char l2_addr_str[GNRC_NETIF_HDR_L2ADDR_PRINT_LEN];
    int anchor_node_id;
    // Document up top = { 2byte flag, 1byte node_id, 2byte slot time (ms), 1byte tot_num_anchors }
    uint8_t buf[3]; // buf is size 6 in master
    uint16_t tdma_slot_time_msec = 0;
    uint8_t total_num_anchors;

    gnrc_netreg_entry_t tdma_slave_serv = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, thread_getpid() };
    
    uint16_t channel = TDMA_BOOTSTRAP_CHANNEL;
    uint16_t tx_power = MAX_TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t)); // check channel
 
    /* there should be only one network interface on the board */
    if (numof == 1) 
    {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }
    /* register for all l2 packets */
    gnrc_netreg_register(1, &tdma_slave_serv);
	// ------------------------------------------------------------------
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), ARREST_LEADER_SHORT_HWADDR);
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

    range_tx_init(TX_PIN);
    //------------------------------------------------------------------------//
    
    // Miscellaneous
    //------------------------------------------------------------------------//
	bool wait_for_id = true;
    xtimer_ticks32_t start_time, wait_time, ranking_time, go_time;
    int incoming_rank;
    // NETOPT_ADDRESS; // this device's hw_addr 
    //------------------------------------------------------------------------//

// MAIN LOOP
 	while (true)
 	{
		// NODE sends REQ packet to LEADER.
	    //--------------------------------------------------------------------//
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
	    //--------------------------------------------------------------------//

    	// NODE A waits for ID packet from LEADER.
	    //--------------------------------------------------------------------//
    	wait_for_id = true;
        while (wait_for_id)
        {
	        msg_receive(&msg);
	        // pkt = msg.content.ptr;

			/* get snip containing packet data where we put the packet number */
	        // snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
	        switch (msg.type)
	        {
				// NODE receives ID packet from LEADER.
	            case GNRC_NETAPI_MSG_TYPE_RCV:
	                DEBUG("Received pkt.\n");
	                recv_pkt = msg.content.ptr;

	                /* first snip should be of type GNRC_NETTYPE_UNDEF carrying the data */
					if (((uint8_t *)recv_pkt->data)[0] == ((TDMA_ANCHOR_ID_RESP_U16_FLAG >> 8) & 0xFF) &&
                    	((uint8_t *)recv_pkt->data)[1] == (TDMA_ANCHOR_ID_RESP_U16_FLAG & 0xFF) )
                    {
			            // NODE records its ranking.
			            anchor_node_id = (int)(((uint8_t *)recv_pkt->data)[2]);

			            // NODE syncs with the LEADER.
	                    tdma_slot_time_msec & 0x00;
	                    ( tdma_slot_time_msec | ((uint8_t *)recv_pkt->data)[3] ) << 8;
	                      tdma_slot_time_msec | ((uint8_t *)recv_pkt->data)[4];

	                    // NODE records total number of nodes (useful for NODE 1).
	                    total_num_anchors = ((uint8_t *)recv_pkt->data)[5];

	                    // NODE A received the data correctly and now breaks out of the loop.
	                    wait_for_id = false;
	                }
	                break;
                case GNRC_NETAPI_MSG_TYPE_SND:
	                /* This thread will get all send l2 send requests, even if it's
	                coming from this thread. Discard it */
	                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                	break
				default:
	                // pkt received didn't match any of these.
	                DEBUG("msg.type = %d\n", msg.type);
	                DEBUG("Waiting for ID packet from LEADER. PKT was none of these.");
	                break;
	        }
	    }
	    //--------------------------------------------------------------------//

	    // NODE A received ID packet from LEADER and is now waiting to go.
	    // TODO: check if GNRC_NETAPI_MSG_TYPE_RCV is correct
	    //--------------------------------------------------------------------//
        start_time = xtimer_now();
        wait_time = xtimer_now() - start_time;
        go_time = anchor_node_id * tdma_slot_time_msec;
        ranking_time = go_time;
        while ( (wait_time < go_time) && (wait_time < ranking_time) )
        {
	        msg_receive(&msg);
	        switch (msg.type)
	        {
	            case GNRC_NETAPI_MSG_TYPE_RCV:
	                DEBUG("Received pkt.\n");
	                recv_pkt = msg.content.ptr;
	                if ( ((uint8_t *)recv_pkt->data)[0] == RANGE_FLAG_BYTE0 &&
	                     ((uint8_t *)recv_pkt->data)[1] == RANGE_FLAG_BYTE1 ) 
	                {
						incoming_rank = ((uint8_t *)recv_pkt->data)[1];
						// NODE's rank is 1 and listening for the last NODE.
						if ( (incoming_rank == total_num_anchors) && (anchor_node_id == 1))
						{
							ranking_time = 1 * tdma_slot_time_msec;
						}
						// NODE's rank is higher than the incoming NODE's transmission.
						if (incoming_rank < anchor_node_id)
						{
							ranking_time = (anchor_node_id - incoming_rank) * tdma_slot_time_msec;
						}
	                }
	                break;
                case GNRC_NETAPI_MSG_TYPE_SND:
	                /* This thread will get all send l2 send requests, even if it's
	                coming from this thread. Discard it */
	                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                	break
	            default:
	            	DEBUG("Not a ranging packet.");
	            	break;
            }
            // Refresh the timer.
        	wait_time = xtimer_now() - rstx_start_time;
        }
	    //--------------------------------------------------------------------//

        //ACUTALLY SENDING THE SIGNAL GOD FINALLY
        // TODO: Modify to not block the interrupts
	    //--------------------------------------------------------------------//
        /** Send L2 Packet **/
        /* network interface */
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), LEADER_HW_ADDR);

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
        
        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(ifs[0], send_pkt) < 1) {
            DEBUG("error: unable to send\n");
            gnrc_pktbuf_release(send_pkt);
            return 1;
        }   
        gnrc_pktbuf_release(recv_pkt);
        range_tx_off(); //turn off just in case
        DEBUG("RF and ultrasound pings sent\n");  
	    //--------------------------------------------------------------------//
    }
    //end of while loop, code should never reach here etc.
    return 0;
}
/*----------------------------------------------------------------------------*/
static range_data_t* time_diffs;
range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, uint16_t num_samples){ 
    // Check correct argument usage.
    uint8_t mode = range_mode;
    uint32_t maxsamps; //number of iterations in the gpio polling loop before calling it a timeout
    gpio_rx_line_t lines = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_XOR_PIN};
    
    if(mode == TWO_SENSOR_MODE){
        maxsamps = MAXSAMPLES_TWO_PIN;
    } else {
        maxsamps = MAXSAMPLES_ONE_PIN;
    }

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
        return NULL;
    }

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

   
    int i;
    for(i = 0; i < num_samples; i++){


        range_rx_init(TX_NODE_ID, thread_getpid(), lines, maxsamps, mode);

block:
        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("RF ping missed\n");
            return NULL;
        }   

        if(msg.type == RF_RCVD){
            if(xtimer_msg_receive_timeout(&msg,timeout)<0){
                DEBUG("Ultrsnd ping missed\n");
                return NULL;
            }
            if(msg.type == ULTRSND_RCVD){
                time_diffs[i] = *(range_data_t*) msg.content.ptr;
            } else{
                goto block;
            }

        }
        if(time_diffs[i].tdoa > 0){
            printf("range: TDoA = %d\n", time_diffs[i].tdoa);
            switch (range_mode){
                case ONE_SENSOR_MODE:
                    break;

                case TWO_SENSOR_MODE:
                    if(time_diffs[i].error!=0){
                        printf("range: Missed pin %d\n", time_diffs[i].error);
                    } else{
                        printf("range: OD = %d\n", time_diffs[i].orient_diff);
                    }
                    break;

                case XOR_SENSOR_MODE:
                    printf("range: OD = %d\n", time_diffs[i].orient_diff);
                    break;
            }
        }
        else{
            printf("Ultrsnd ping missed\n");
        }


    }

    return time_diffs;
}
/*----------------------------------------------------------------------------*/
static range_data_t* time_diffs;
range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, uint16_t num_samples){ 
    // Check correct argument usage.
    uint8_t mode = range_mode;
    uint32_t maxsamps; //number of iterations in the gpio polling loop before calling it a timeout
                       //
    gpio_rx_line_t lines = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_XOR_PIN};
    
    if(mode == TWO_SENSOR_MODE){
        maxsamps = MAXSAMPLES_TWO_PIN;
    } else {
        maxsamps = MAXSAMPLES_ONE_PIN;
    }

    time_diffs = malloc(sizeof(range_data_t)*num_samples);
    
    uint32_t timeout = timeout_usec;
    if(timeout <= 0){
        DEBUG("timeout must be greater than 0");
        return NULL;
    }

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

   
    int i;
    for(i = 0; i < num_samples; i++){


        range_rx_init(TX_NODE_ID, thread_getpid(), lines, maxsamps, mode);

block:
        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("RF ping missed\n");
            return NULL;
        }

        if(msg.type == RF_RCVD){
            if(xtimer_msg_receive_timeout(&msg,timeout)<0){
                DEBUG("Ultrsnd ping missed\n");
                return NULL;
            }
            if(msg.type == ULTRSND_RCVD){
                time_diffs[i] = *(range_data_t*) msg.content.ptr;
            } else{
                goto block;
            }

        }
        printf("range: tdoa = %d\n", time_diffs[i].tdoa);
        switch (range_mode){
            case ONE_SENSOR_MODE:
                break;

            case TWO_SENSOR_MODE:
                if(time_diffs[i].error!=0){
                    printf("range: Missed pin %d\n", time_diffs[i].error);
                } else{
                    printf("range: odelay = %d\n", time_diffs[i].odelay);
                }
                break;

            case XOR_SENSOR_MODE:
                printf("range: odelay = %d\n", time_diffs[i].odelay);
                break;
        }
        if(i == num_samples-1){
            time_diffs[i].error += 10;
        }


    }

    return time_diffs;
}
