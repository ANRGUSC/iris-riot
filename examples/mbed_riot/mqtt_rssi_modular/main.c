/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Pradipta Ghosh
 * Daniel Dsouza
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
 * @brief       Full-duplex MQTT test
 *
 *
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      Daniel Dsouza <dmdsouza@usc.edu>
 * @}
 */

#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/netdev.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//Header to reboot the system
#include "periph/pm.h"

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "random.h"
#include "xtimer.h"
#include "periph/uart.h"
#include "net/hdlc.h"
#include "net/uart_pkt.h"

#include "net/gnrc.h"
#include "net/gnrc/netapi.h"
#include <stdlib.h>
#include "net/emcute.h"
#include "net/ipv6/addr.h"
#include "main-conf.h"
#include "mqtt.h"
#include "app-conf.h"
#include "mqtt_thread.h"



#define ENABLE_DEBUG (1)
#include "debug.h"
//setting the priority of the hdlc and thread2
#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define MQTT_CONTOL_PRIO            (THREAD_PRIORITY_MAIN)
#define RSSI_DUMP_PRIO          (THREAD_PRIORITY_MAIN - 1)
//setting the port of the main thread
//setting the port of thread2 

/* see openmote-cc2538's periph_conf.h for second UART pin config */
//setting the message queue with message structs
static msg_t main_msg_queue[16];
static msg_t rssi_dump_msg_queue[16];
//creating the stacks
static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];//16896
static char mqtt_control_thread_stack[THREAD_STACKSIZE_MAIN];//16384
static char rssi_dump_stack[THREAD_STACKSIZE_MAIN];//16384

static kernel_pid_t thread2_pid;
static kernel_pid_t rssi_pid;


/**
 * @brief      Gets the rssi value from the gnrc packet
 *
 * @param      pkt   The packet snip
 *
 * @return     { returns the rssi value of the udp message }
 */

static uint8_t rssi_val(gnrc_pktsnip_t *pkt)
{
    int                 snips = 0;
    int                 size = 0;
    uint8_t             raw_rssi =0;
    gnrc_netif_hdr_t    *hdr;
    gnrc_pktsnip_t      *snip = pkt;
    udp_hdr_t           *udp;
    ipv6_hdr_t           *ip;


    while (snip != NULL) {
        switch (snip->type){
            case GNRC_NETTYPE_NETIF:
                hdr = (gnrc_netif_hdr_t *) (snip->data);
                raw_rssi = hdr->rssi;            
                return raw_rssi;

            case GNRC_NETTYPE_UDP:
                udp = (udp_hdr_t *) (snip->data);
                // udp_hdr_print (udp);
                break;

            case GNRC_NETTYPE_IPV6:
                ip = (ipv6_hdr_t *) (snip->data);
                // ipv6_hdr_print (ip);
                break;

            default:
                break;
        }
        ++snips;
        size += snip->size;
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}

/**
 * @brief      function to send udp packets to a chosen port
 *
 * @param      addr_str  The address string
 * @param      port_str  The port string
 * @param      data      The data
 */
static int rssi_send(char *addr_str, uint16_t port, char *data)
{
    ipv6_addr_t     addr;
    unsigned int    num =1;
    gnrc_pktsnip_t *payload, *udp, *ip;
    unsigned payload_size;
        
    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        DEBUG("Error: unable to parse destination address");
        return -1;
    }

    /* parse port */
    if (port == 0) {
        DEBUG("Error: unable to parse destination port");
        return -1;
    }

    for (unsigned int i = 0; i < num; i++) {
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            DEBUG("Error: unable to copy data to packet buffer");
            return -1;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            DEBUG("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return 1;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            DEBUG("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return 1;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            DEBUG("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return 1;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        DEBUG("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
               port);               
        xtimer_usleep(10000);       
        return 0;
    }
}




/**
 * @brief      Main Thread
 *
 * @return     status
 */
int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming packets */
    msg_init_queue(rssi_dump_msg_queue, sizeof(rssi_dump_msg_queue));

    hdlc_entry_t rssi_dump_thr = { .next = NULL, .port = RSSI_RIOT_PORT, 
                                         .pid = thread_getpid() };
    hdlc_register(&rssi_dump_thr); 

    //setting the hdlc pid 
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(1));  
    kernel_pid_t mqtt_pid = mqtt_thread_init(mqtt_control_thread_stack, sizeof(mqtt_control_thread_stack), MQTT_CONTOL_PRIO, 
                                      "mqtt_control", (void*) hdlc_pid);

    //creates rssi_dump thread
    //Initializes and starts the udp server on RSSI_DUMP_PORT
    static gnrc_netreg_entry_t server1 = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);
    server1.target.pid = thread_getpid();
    server1.demux_ctx  = RSSI_DUMP_PORT;

    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server1);
    DEBUG("Success: started UDP server on port %" PRIu16 "\n", server1.demux_ctx);

    //The main thread DOES NOT send and receive messages in this example
    //setting up the two message structs 
    char  frame_no = 0;
    //create packets with max size 
    random_init(xtimer_now().ticks32);

    //DEBUG("Main Thread pid is %" PRIkernel_pid "\n", thread_getpid());
    int exit = 0;

    char udp_msg[6] = "hello";
    msg_t msg_send_to_rssi_dump;
    msg_t msg, msg_snd;
    uint8_t rssi_value;
    int rssi_go = 0;   
    int i = 0; 
    int c = 15;   
    char send_data[HDLC_MAX_PKT_SIZE];
    char ipv6_send_addr[25]="fe80::212:4b00:0000:0000";
    char rx_node_id[9];  

    gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               thread_getpid());
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t uart_rcv_hdr; 
    
    while (get_mqtt_state() != MQTT_MBED_INIT_DONE)
        xtimer_usleep(100000);

    DEBUG("rssi_thread: initialization complete\n" );

    while (1)
    {
        // DEBUG("rssi_thread: waiting for message\n");
        msg_receive(&msg);
        
        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                DEBUG("rssi_thread: message from hdlc\n"); 
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg.content.ptr;   
                uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                switch(uart_rcv_hdr.pkt_type){
                    case RSSI_SND:
                        DEBUG("rssi_thread: received the rssi send message\n");                        
                        memcpy(rx_node_id, hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD, sizeof(rx_node_id));
                        DEBUG("rssi_thread: The node to send to is %s\n", rx_node_id);
                        hdlc_pkt_release(hdlc_rcv_pkt);

                        c = 15;
                        for (i = 0 ; i < 8 ; i++){
                            if (c != 19)
                                ipv6_send_addr[c] = rx_node_id[i];
                            c = (c == 18) ? (c + 2) : (c + 1);
                        }
                        DEBUG("rssi_thread: The ipv6 is %s\n", ipv6_send_addr); 
                        //sending to the constructed ipv6 address at port 9000                                                                                          
                        if(rssi_send(ipv6_send_addr, RSSI_DUMP_PORT, udp_msg) == 0)
                            DEBUG("rssi_thread: udp message sent\n");
                            
                        break;  
                }
                break;

            case HDLC_RESP_RETRY_W_TIMEO:
                xtimer_usleep(msg.content.value);
                //DEBUG("mqtt_control_thread: retrying frame_no %d\n", frame_no);
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("rssi_thread: HDLC msg queue full!\n");
                    msg_send_to_self(&msg);
                }
                break;

            case HDLC_RESP_SND_SUCC:
                DEBUG("rssi_thread : sent frame \n");
                break;

            case GNRC_NETAPI_MSG_TYPE_RCV: 
                DEBUG("rssi_thread: RSSI packet received\n");             
                rssi_value = rssi_val(msg.content.ptr);                
                DEBUG("rssi_thread: rssi value %d\n", rssi_value);
                //sending information to rssi thread on the MBED side                
                uart_hdr.src_port = RSSI_RIOT_PORT; //PORT 220
                uart_hdr.dst_port = RSSI_MBED_DUMP_PORT; //PORT 9111
                uart_hdr.pkt_type = RSSI_DATA_PKT;
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, &rssi_value, sizeof(rssi_value));  

                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;                
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("rssi_thread: HDLC msg queue full\n");
                    continue;
                }     
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);          
                break;

            case GNRC_NETAPI_MSG_TYPE_SND:
                /* just in case */
                DEBUG("rssi_thread: gnrc_pkt send\n");
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;

            default:
                /* error */
                DEBUG("rssi_thread: invalid packet\n");
                break;
        }
        
    }    
    //should be never reached 
    
    return 0;
}

