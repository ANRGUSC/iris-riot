/*````````````````````````````````````````````````````````````````````````````````````
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

#include "range.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

#define MAXSAMPLES_ONE_PIN            18000
#define MAXSAMPLES_TWO_PIN            18000

#define RX_ONE_PIN                    GPIO_PIN(3, 3) //aka GPIO_PD3 - maps to DIO0
#define RX_TWO_PIN                    GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1
#define RX_XOR_PIN                    GPIO_PIN(3, 1) //aka GPIO_PD1 - maps to DIO2
#define TX_PIN                        GPIO_PIN(3, 0) //aka GPIO_PD0 - maps to DIO3

// TODO: Comb code for memory leaks, figure out why openmote is freezing. Change for to while.
#define MAX_TX_RUNS                   2000 // Tested number of runs before the openmote freezes.
#define TIME_WINDOW                   1000 // Time window of one run, should be uint32_t.
#define NUM_OF_NODES                  3    // Number of nodes
#define LEADER_HW_ADDR                ff:ff// TODO: Replace with the correct MAC address of the leader.
//static unsigned int gpio_lines[]={GPIO_PIN(3, 3), GPIO_PIN(3, 2), GPIO_PIN(3, 1)};

#define FOLLOW_SYNC                     0x32    //pray that these aren't taken
#define FOLLOW_ASSIGN                   0X54
#define FOLLOW_GO                       0x76
#define LEAD_INFO                       0X98

#define BUFFER_SIZE_OF_PACKET           3       // size of buf that holds address and other data.
#define BUFFER_SIZE_OF_PACKET_LEADER    4       // size of buf that holds address and other data.


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
            if(i == num_samples-1){
                time_diffs[i].error += 10;
            }
        }
        else{
            printf("Ultrsnd ping missed\n");
        }


    }

    return time_diffs;
}
/*----------------------------------------------------------------------------*/
xtimer_ticks32_t sync_time(xtimer_ticks32_t leader_time)
{
    return leader_time - xtimer_now();
}
/*----------------------------------------------------------------------------*/
int range_tx(uint32_t delay_usec)
{
    // TODO: Check correct argument usage.

    // Broadcasting setup
    // ------------------------------------------------------------------------
    /* for sending L2 pkt */
    kernel_pid_t dev;
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;

    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;    
    char buf[3] = {0x00, 0x00, 0x00}; // buf used to transmit
                                      // - tx ranging data
                                      // - follower nodes' addresses
    char buf2[BUFFER_SIZE_OF_PACKET] = {0x00, 0x00, 0x00, 0x00, 0x00}; // buf2 used to transmit
                                                                       // times for sync

    int16_t tx_power = TX_POWER;

    /* setup the message queue */
    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 
    /* there should be only one network interface on the board */
    if (numof == 1) {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }
    // ------------------------------------------------------------------------

    // Ultrasonic sensor setup
    // ------------------------------------------------------------------------
    /* enable output on Port D pin 3 */
    if(gpio_init(GPIO_PD2, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.");
        return 1;
    }
    // clearing output for the ultrasonic sensor
    DEBUG("Clearing GPIO_PD2");
    gpio_clear(GPIO_PD2);
    // ------------------------------------------------------------------------

    // Miscellaneous setup
    // ------------------------------------------------------------------------
    bool no_signal;
    xtimer_ticks32_t rstx_start_time;
    uint8_t *hw_addr_nodes[NUM_OF_NODES]; // array of hw_addr of the nodes
    // NETOPT_ADDRESS; // this device's hw_addr 
    int place_on_list = 0; // counter to keep track of place on list 
                           // (also in case not all nodes contacted)

    // for determining when to broadcast.
    srand(time(NULL));   // should only be called once
    int r = rand() % NUM_OF_NODES;// returns a pseudo-random integer between 0 and NUM_OF_NODES-1
    
    // for processing the time from the leader node.
    uint32_t time_processing = 0x00000000;
    xtimer_ticks32_t offset_time;
    
    // for transmitting the address of this node.
    uint8_t tx_node_id1 = (uint8_t)(NETOPT_ADDRESS >> 8)
    uint8_t tx_node_id2 = (uint8_t)(NETOPT_ADDRESS)

    // TODO: finalize syncing
    // syncing:
    // xtimer_ticks32_t current_time, current_lead_time, sync_offset;
    // bool sync_offset_pos;

    // current_time = xtimer_now();
    // if(xtimer_less(current_time, current_lead_time))
    // {
    //     sync_offset = xtimer_diff(current_lead_time, current_time);
    //     sync_offset_pos = true;
    // }
    // else
    // {
    //     sync_offset = xtimer_diff(current_time, current_lead_time);
    //     sync_offset_pos = false;
    // }
    // 
    // if xtimer_ticks32_t can be negative:
    // sync_offset = current_lead_time = xtimer_now

    // ------------------------------------------------------------------------

    // check for leader, leader processing
    if(LEADER_HW_ADDR == NETOPT_ADDRESS) //or some matching
    {
        int current_list_size = 1;
        uint16_t list_of_hw_addr[NUM_OF_NODES+1];
        xtimer_ticks32_t cutoff_timer = 0
        xtimer_ticks32_t current_time = xtimer_now();
        
        // prepare sync packet
        // wait for a little bit for everyone to settle down
        // send SYNC
        
        // Broadcasting flag setup.
        flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
        
        /** Send L2 Packet **/
        /* network interface */
        dev = ifs[0];
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
        
        /* put packet together */
        buf2[0] = (uint8_t)current_time >> 24;
        buf2[1] = (uint8_t)current_time >> 16;
        buf2[2] = (uint8_t)current_time >> 8;
        buf2[3] = (uint8_t)current_time;
        buf2[4] = FOLLOW_SYNC;

        pkt = gnrc_pktbuf_add(NULL, &buf2, BUFFER_SIZE_OF_PACKET_LEADER, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            DEBUG("error: packet buffer full");
            return 1;
        }
       
        hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
        if (hdr == NULL) {
            DEBUG("error: packet buffer full");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        LL_PREPEND(pkt, hdr);
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        /* ready to send */

        // make sure no packets are to be sent!!
        if (gnrc_netapi_send(dev, pkt) < 1) {
            DEBUG("error: unable to send");
            gnrc_pktbuf_release(pkt);
            return 1;
        }   

        // prepare time (time is equal to NUM_OF_NODES)        
        current_time = xtimer_now();
        cutoff_timer = xtimer_now() - current_time;
        while(current_list_size < NUM_OF_NODES && cutoff_timer < (NUM_OF_NODES * TIME_WINDOW))
        {
            // set timer for 1 time window
            msg_receive(&msg);
            if (msg.type == LEAD_INFO) // ensure that they send an INFO signal or something
            {
                // unpack and get the hw_addr
                pkt = msg.content.ptr;
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);

                // getting the hw_addr
                uint16_t incoming_hw_addr = 0x0000;
                incoming_hw_addr | ((uint16_t)((uint8_t *)snip->data)[0]) << 8;
                incoming_hw_addr | ((uint8_t *)snip->data)[1];

                // add to the list: hw_addr_nodes
                hw_addr_nodes[current_list_size] = incoming_hw_addr;

                current_list_size++; // increment current_list_size
                cutoff_timer = 0; // reset the timer
            }
            // TODO: figure out how to get it to stop if nothing received.
            if(msg.type == NULL) // no message received or something
            {
                cutoff_timer = xtimer_now() - current_time;
            }
            cutoff_timer = xtimer_now() - current_time; // increment timer
        }

        // done with getting the list
        // now need to send out the list
        for(int x = 1; x < current_list_size + 1; x++)
        {
            // prepare and send packets with x and *(x + hw_addr_nodes) and ASSIGN flags.
            /** Send L2 Packet **/
            /* network interface */
            dev = ifs[0];
            // hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
            hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), hw_addr_nodes[x]);

            /* put packet together */
            buf[0] = 0x00; // unused
            buf[1] = (uint8_t)x;
            buf[2] = FOLLOW_ASSIGN;
            pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
            if (pkt == NULL) {
                DEBUG("error: packet buffer full");
                return 1;
            }
           
            hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
            if (hdr == NULL) {
                DEBUG("error: packet buffer full");
                gnrc_pktbuf_release(pkt);
                return 1;
            }
            LL_PREPEND(pkt, hdr);
            nethdr = (gnrc_netif_hdr_t *)hdr->data;
            nethdr->flags = flags;
            /* ready to send */

            // make sure no packets are to be sent!!
            if (gnrc_netapi_send(dev, pkt) < 1) {
                DEBUG("error: unable to send");
                gnrc_pktbuf_release(pkt);
                return 1;
            }   
        }
        while(true)
        {
            // send GO signal with timestamp
            // send SYNC
            
            // Broadcasting flag setup.
            flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
            
            /** Send L2 Packet **/
            /* network interface */
            dev = ifs[0];
            hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
            
            /* put packet together */
            buf2[0] = (uint8_t)current_time >> 24;
            buf2[1] = (uint8_t)current_time >> 16;
            buf2[2] = (uint8_t)current_time >> 8;
            buf2[3] = (uint8_t)current_time;
            buf2[0] = FOLLOW_GO;

            pkt = gnrc_pktbuf_add(NULL, &buf2, BUFFER_SIZE_OF_PACKET_LEADER, GNRC_NETTYPE_UNDEF);
            if (pkt == NULL) {
                DEBUG("error: packet buffer full");
                return 1;
            }
           
            hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
            if (hdr == NULL) {
                DEBUG("error: packet buffer full");
                gnrc_pktbuf_release(pkt);
                return 1;
            }
            LL_PREPEND(pkt, hdr);
            nethdr = (gnrc_netif_hdr_t *)hdr->data;
            nethdr->flags = flags;
            /* ready to send */

            // make sure no packets are to be sent!!
            if (gnrc_netapi_send(dev, pkt) < 1) {
                DEBUG("error: unable to send");
                gnrc_pktbuf_release(pkt);
                return 1;
            }   
            // wait current_list_size * TIME_WINDOW + 1
            xtimer_usleep( (current_list_size + 1) * TIME_WINDOW );
        }
    }
    // end leader processing
    // leader should never go beyond this
    
    // follower code starts here, should skip leader processing
    while(true)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            case FOLLOW_SYNC:
            	// FOLLOW_SYNC = {time[4], FOLLOW_SYNC}
                // - All nodes sync to timestamp.

                // Pull time from FOLLOW_SYNC from the leader node.
                pkt = msg.content.ptr;

                /* get snip containing packet data where we put the packet number */
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);

                // leader sends out pkt with FOLLOW_SYNC
                // should be 5 bytes big
                // bytes 0-3 should hold the time
                // byte 4 holds the identifier? flag? (that it IS a FOLLOW_SYNC signal) 
                // sloppy, should've used strcpy? or something to copy the bits over.
                for(int x = 0; x < 4; x++)
                {
                    part_of_time = ((uint8_t *) snip->data)[x];
                    time_processing = (time_processing << 8) | part_of_time;
                }
                gnrc_pktbuf_release(pkt);

                // syncing...
                offset_time = sync_time( (xtimer_ticks32_t)time_processing);

                // - All nodes randomly send hw_addr.
                // - If successful, node will listen for ASSIGN signal.
                // - If failed (collision), node will wait (# of nodes) * time_window
                xtimer_usleep(r * TIME_WINDOW);

                // TODO: Double check this.
                // TODO: code in collision handling.

                /** Send L2 Packet **/
                /* network interface */
                dev = ifs[0];
                hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), LEADER_HW_ADDR);
                
                /* put packet together */
                buf[0] = tx_node_id1;
                buf[1] = tx_node_id2;
                buf[2] = LEAD_INFO;

                pkt = gnrc_pktbuf_add(NULL, &buf, sizeof(buf), GNRC_NETTYPE_UNDEF);
                if (pkt == NULL) {
                    DEBUG("error: packet buffer full");
                    return 1;
                }
               
                hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
                if (hdr == NULL) {
                    DEBUG("error: packet buffer full");
                    gnrc_pktbuf_release(pkt);
                    return 1;
                }

                LL_PREPEND(pkt, hdr);
                nethdr = (gnrc_netif_hdr_t *)hdr->data;
                nethdr->flags = flags;
                /* ready to send */

                // make sure no packets are to be sent!!
                if (gnrc_netapi_send(dev, pkt) < 1) {
                    DEBUG("error: unable to send");
                    gnrc_pktbuf_release(pkt);
                    return 1;
                }   
                break;
            case FOLLOW_ASSIGN:
                // FOLLOW_ASSIGN = {0x00, place_on_list, FOLLOW_ASSIGN}
                // - Nodes remember place_on_list variable.
                
                pkt = msg.content.ptr;

                /* get snip containing packet data where we put the packet number */
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
                
                place_on_list = (int)((uint8_t *)snip->data)[1];

                gnrc_pktbuf_release(pkt);
                break;
            case FOLLOW_GO:
            	// FOLLOW_GO = {time[4], FOLLOW_GO}
                // syncing...
                // - All nodes sync to timestamp.

                // Pull time from FOLLOW_GO from the leader node.
                pkt = msg.content.ptr;

                /* get snip containing packet data where we put the packet number */
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);

                // leader sends out pkt with FOLLOW_GO
                // should be 5 bytes big
                // bytes 0-3 should hold the time
                // byte 4 holds the identifier? flag? (that it IS a FOLLOW_GO signal) 
                // sloppy, should've used strcpy? or something to copy the bits over.
                for(int x = 0; x < 4; x++)
                {
                    part_of_time = ((uint8_t *) snip->data)[x];
                    time_processing = (time_processing << 8) | part_of_time;
                }
                gnrc_pktbuf_release(pkt);

                // syncing...
                offset_time = sync_time( (xtimer_ticks32_t)time_processing);
                
                //ACUTALLY SENDING THE SIGNAL GOD FINALLY

                // TODO: Modify to not block the interrupts
                xtimer_usleep(place_on_list * TIME_WINDOW);

                // broadcast
                range_tx_init(GPIO_PD2);

                // Broadcasting flag setup.
                flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
                
                /** Send L2 Packet **/
                /* network interface */
                dev = ifs[0];
                // hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
                hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), LEADER_HW_ADDR);

                /* put packet together */
                buf[0] = RANGE_FLAG_BYTE0;
                buf[1] = RANGE_FLAG_BYTE1;
                buf[2] = TX_NODE_ID;
                pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
                if (pkt == NULL) {
                    DEBUG("error: packet buffer full");
                    return 1;
                }
               
                hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
                if (hdr == NULL) {
                    DEBUG("error: packet buffer full");
                    gnrc_pktbuf_release(pkt);
                    return 1;
                }
                LL_PREPEND(pkt, hdr);
                nethdr = (gnrc_netif_hdr_t *)hdr->data;
                nethdr->flags = flags;
                /* ready to send */

                // make sure no packets are to be sent!!
                if (gnrc_netapi_send(dev, pkt) < 1) {
                    DEBUG("error: unable to send");
                    gnrc_pktbuf_release(pkt);
                    return 1;
                }   
                range_tx_off(); //turn off just in case
                DEBUG("RF and ultrasound pings sent");  
                // should go back to waiting for a signal.
                break;
            default:
                DEBUG("Packet does not match SYNC, ASSIGN, or GO.\n");
                break;
    }

    // old main loop
    // while(true){
    // for(int x = 0; x < MAX_TX_RUNS; x++){

    //     xtimer_usleep(delay_usec);

    //     range_tx_init(GPIO_PD2);

    //     // Broadcasting flag setup.
    //     flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
        
    //     /** Send L2 Packet **/
    //     /* network interface */
    //     dev = ifs[0];
    //     hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
        
    //     /* put packet together */
    //     buf[0] = RANGE_FLAG_BYTE0;
    //     buf[1] = RANGE_FLAG_BYTE1;
    //     buf[2] = TX_NODE_ID;
    //     pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
    //     if (pkt == NULL) {
    //         DEBUG("error: packet buffer full");
    //         return 1;
    //     }
       
    //     hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
    //     if (hdr == NULL) {
    //         DEBUG("error: packet buffer full");
    //         gnrc_pktbuf_release(pkt);
    //         return 1;
    //     }
    //     LL_PREPEND(pkt, hdr);
    //     nethdr = (gnrc_netif_hdr_t *)hdr->data;
    //     nethdr->flags = flags;
    //     /* ready to send */

    //     // make sure no packets are to be sent!!
    //     if (gnrc_netapi_send(dev, pkt) < 1) {
    //         DEBUG("error: unable to send");
    //         gnrc_pktbuf_release(pkt);
    //         return 1;
    //     }   
    //     range_tx_off(); //turn off just in case
    //     DEBUG("RF and ultrasound pings sent");  
    // }

    return 0;
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
