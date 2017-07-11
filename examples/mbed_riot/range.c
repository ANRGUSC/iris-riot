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

static uint32_t time_diffs[3];

/*----------------------------------------------------------------------------*/
range_data_t* range_rx(uint32_t utimeout, uint32_t sys_flag){
    // Check correct argument usage.
    uint32_t flag = sys_flag;
    
    uint32_t maxsamps = 0;

    uint32_t timeout = utimeout;
    if(timeout <= 0){
        DEBUG("timeout must be greater than 0");
        return NULL;
    }

    // Message setup.
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.target.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

   
    if(flag == TWO_SENSOR_MODE){
        maxsamps = 30000;
    } else {
        maxsamps = 100000;
    }

    range_rx_init(TX_NODE_ID, thread_getpid(), gpio_lines, maxsamps, flag);

block:
    if(xtimer_msg_receive_timeout(&msg,timeout)<0){
        DEBUG("Ping missed");
         _unregister_thread();
        return NULL;
    }

    if(msg.type == 143){
        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("Ping missed #1");
            return NULL;
        }
        if(msg.type == 144){
            memcpy(time_diffs, msg.content.ptr, sizeof(uint32_t)*3);
        } else{
            goto block;
        }

    }
    _unregister_thread();

    printf("range: TDoA = %lu\n", time_diffs[0]);
    switch (sys_flag){
        case ONE_SENSOR_MODE:
            break;

        case TWO_SENSOR_MODE:
            if(time_diffs[2]!=0){
                printf("range: Missed pin %lu\n", time_diffs[2]);
            } else{
                printf("range: OD = %lu\n", time_diffs[1]);
            }
            break;

        case XOR_SENSOR_MODE:
            printf("range: OD = %lu\n", time_diffs[1]);
            break;
    }

    return time_diffs;
}

/*----------------------------------------------------------------------------*/
int range_tx(uint32_t udelay)
{
/*
    Overall structure:
    // main loop
    while(true){
        // setup variables before each run
        boolean no_signal = true
        time = 0
        // listen for any fellow broadcasting signals
        while (no_signal && not_time_yet (time < delay_time))
            check for signal
                if signal, no_signal = false
            time++
            if no_signal == false //signal received
                check if valid signal - if valid, 
                    calculate time to broadcast off of id of that signal
                    wait time
                    time = delay_time (effectively a break)
                else ignore invalid signals
                    error msg
                    no_signal = true
        if none detected within TIME_DELAY??? (time >= delay time)
            broadcast signal id + ping for however long
    }
*/
    // Check correct argument usage.

    // Broadcasting setup
    uint32_t TX_DELAY_TIME = udelay;

    /* for sending L2 pkt */
    kernel_pid_t dev;
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;    
    char buf[3] = {0x00, 0x00, 0x00};

    int16_t tx_power = TX_POWER;

    /* register this thread to the chosen UDP port */
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.target.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    /* enable output on Port D pin 3 */
    if(gpio_init(GPIO_PD2, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.");
        return 1;
    }
    // clearing output for the ultrasonic sensor
    DEBUG("Clearing GPIO_PD2");
    gpio_clear(GPIO_PD2);

    // Richard's code, mod the ipv6 into mac address checking, add to main loop.
    /*while(true){
        no_signal = true;
        rstx_start_time = xtimer_now();
        current_time = 0;
        
        while(no_signal && current_time < TX_DELAY_TIME){
            current_time = xtimer_now() - rstx_start_time;
            msg_receive(&msg);
            
                do processing to check ipv6 address, i.e., where it should
                belong on the chart/node list, etc.

                if(is_valid_addr(ipv6)){
                    no_signal = false;

                    uint32_t waiting_time = processing(ipv6_addr);
                    xtimer_sleep(waiting_time);
                }
                else{
                    DEBUG("Invalid ipv6 address.");
                    no_signal = true;
                }
            
            // how to check signal???
            // use mac address instead of ipv6
            
            if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
                pakt = msg.content.ptr;

                snip = gnrc_pktsnip_search_type(pakt, GNRC_NETTYPE_IPV6);

                tx_node_ip_addr = ((ipv6_hdr_t *)snip->data)->src;
                char ipv6_addr[IPV6_ADDR_MAX_STR_LEN];
                ipv6_addr_to_str(ipv6_addr, &tx_node_ip_addr, sizeof(ipv6_addr));
                for(int x = 0; x < IPV6_ADDRESS_TABLE_LENGTH; x++)
                {
                    if(strcmp(*IPV6_ADDRESS_TABLE[x], ipv6_addr) == 0)
                    {
                        no_signal = false;
                        
                        uint32_t waiting_time = processing(ipv6_addr);
                        xtimer_sleep(waiting_time);
                    }
                }
                gnrc_pktbuf_release(pakt);
                break;
            }
        }
        if(current_time > TX_DELAY_TIME){
            //broadcasting
            //how to broadcast???
            flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
            */
            /** Send L2 Packet **/
            /* network interface */
            /*dev = ifs[0];
            hw_addr = 0;
            hw_addr_len = 0;

            hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
            if (hdr == NULL) {
                DEBUG("error: packet buffer full");
                gnrc_pktbuf_release(pkt);
                return 1;
            }
            LL_PREPEND(pkt, hdr);
            nethdr = (gnrc_netif_hdr_t *)hdr->data;
            nethdr->flags = flags;
            */
            /* ready to send */

            //make sure no packets are to be sent!!
            /*if (gnrc_netapi_send(dev, pkt) < 1) {
                DEBUG("error: unable to send");
                gnrc_pktbuf_release(pkt);
                */
    // Run counter.
    // int i = 0;
    // main loop
    // while(true){
    for(int x = 0; x < 2000; x ++){
        //printf("%d: ", x);

        // Counting runs.
        // printf("%d: ", i);
        // i++;

        xtimer_usleep(TX_DELAY_TIME);

        range_tx_init(GPIO_PD2);
        // Broadcasting flag setup.
        flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
        
        /** Send L2 Packet **/
        /* network interface */
        dev = ifs[0];
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
        
            // Pinging (ignore.)
            //DEBUG("Pinging...");
            /* set pin to 1 for around 50uS */
            // gpio_set(GPIO_PD2);
            /* ultrasound ping should execute 20.5msec after gpio pin goes up */
            // xtimer_spin(100); //or however long it should be up
            // DEBUG("Clearing GPIO_PD2");
            // gpio_clear(GPIO_PD2);
            // DEBUG("RF and ultrasound pings sent");
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

        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(dev, pkt) < 1) {
            DEBUG("error: unable to send");
            gnrc_pktbuf_release(pkt);
            return 1;
        }   
        //gnrc_pktbuf_release(pkt);
        range_tx_off(); //turn off just in case
        DEBUG("RF and ultrasound pings sent");  
    }

    _unregister_thread();
    return 0;
}
