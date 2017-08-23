/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran <jasontra@usc.edu>
 * Pradipta Ghosh <pradiptg@usc.edu>
 * Daniel Dsouza <dmdsouza@usc.edu>
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
 * @brief       ANRG Robot Testbed Alpha Release
 *
 *
 * @author      Jason A. Tran <jasontra@usc.ed>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @author      
 * @author      Daniel Dsouza <dmdsouza@usc.edu>

 * @}
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "xtimer.h"
#include "periph/uart.h"
#include "net/hdlc.h"
#include "net/uart_pkt.h"

#include "net/gnrc.h"
#include "net/gnrc/netapi.h"
#include "net/emcute.h"
#include "net/ipv6/addr.h"

#include "mqtt.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define HDLC_PRIO           (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO        (THREAD_PRIORITY_MAIN)
#define EMCUTE_PRIO         (THREAD_PRIORITY_MAIN - 1)

//setting the port of the main thread
//setting the port of thread2 
#define MAIN_THR_PORT   165
#define MQTT_THR_PORT   170
#define MBED_PORT       200

/* see openmote-cc2538's periph_conf.h for second UART pin config */

static msg_t mqtt_thread_queue[8];
static msg_t main_msg_queue[8];
static char hdlc_stack[THREAD_STACKSIZE_MAIN];
static char mqtt_thread_stack[THREAD_STACKSIZE_MAIN];

#define EMCUTE_PORT         (1883U)  
#define MQTT_SN_SERVER      ("fd00:dead:beef::1")
#define MQTT_SN_PORT        ("8888")
#define INIT_INFO_TOPIC     ("init_info")


#define MAX_NUM_SUBS        (16U)  //Define the maximum number of subscriptions
#define TOPIC_MAXLEN        (16U)

static char my_hwaddr_topic[9];
static char stack[THREAD_STACKSIZE_DEFAULT];
static emcute_sub_t subscriptions[MAX_NUM_SUBS];
static char topics[MAX_NUM_SUBS][TOPIC_MAXLEN];
static kernel_pid_t mqtt_thread_pid;
static bool mbed_initialization_flag = 0;

//Global variables 
char send_data[32];
char pub_server[32];
char send_topic[16];
mqtt_pkt_t mqtt_snd_pkt;

static void *emcute_thread(void *arg)
{
    DEBUG("Starting  MQTT thread \n ");
    mqtt_thread_pid = (kernel_pid_t)arg;
    emcute_run(EMCUTE_PORT, my_hwaddr_topic);
    return NULL;    /* should never be reached */
}

/**
 * @brief      callback function for subscibing to a topic. When some data is published in that topic, this function is called.
 *
 * @param[in]  topic     The topic
 * @param      data      The data
 * @param[in]  data_len  The data length0
 */
static void on_pub_mbed(const emcute_topic_t *topic, void *data, size_t data_len)
{
    int topic_len;
    topic_len = strlen(topic->name);

    msg_t msg_to_mqtt_thr;

    memcpy((void *)&send_data, data, data_len);
    send_data[data_len]='\0';
    memcpy((char *)&send_topic, topic->name, topic_len);
    send_topic[topic_len]='\0';

    /* TO DO: add a mutex */

    strncpy(mqtt_snd_pkt.topic, send_topic, topic_len);
    strncpy(mqtt_snd_pkt.data, send_data, data_len);
    msg_to_mqtt_thr.type = MQTT_MBED;    
    mqtt_snd_pkt.topic[topic_len] = '\0';
    mqtt_snd_pkt.data[data_len] = '\0';
    msg_to_mqtt_thr.content.ptr = &mqtt_snd_pkt;    

    if (msg_try_send(&msg_to_mqtt_thr, mqtt_thread_pid))
        DEBUG("on_pub_mbed : Successfully sent to the mqtt control thread\n");  
    else
        DEBUG("on_pub_mbed : Failed to send to to the mqtt control thread\n");
}
    
/**
 * @brief      publish to a topic
 *
 * @param      pub_topic  The pub topic
 * @param      data       The data
 *
 * @return     status
 */
static int pub(char* pub_topic, char* data)
{
    emcute_topic_t t;
    unsigned flags = EMCUTE_QOS_0;
    mqtt_topic_entry_t *new_topic_entry;
    mqtt_topic_entry_t *entry;
    DEBUG("pub: Trying to publish with topic: %s and data %s \n", pub_topic, data);

    /* step 1: get topic id */
    t.name = pub_topic;

    /* First Check if the topic id is available locally. Otherwise get it from the broker*/
    t.id  = mqtt_search_scalar(pub_topic);
    if ( !t.id )
    {
        DEBUG("pub: No registry found %d\n",strlen(pub_topic));

        if (emcute_reg(&t) != EMCUTE_OK) {
            DEBUG("error: unable to obtain topic ID");
            fflush(stdout);
            return 1;
        }

        /* Add the topic id in the local cache*/
        new_topic_entry = (mqtt_topic_entry_t *)malloc(sizeof(mqtt_topic_entry_t));
        new_topic_entry->id = t.id;
        memcpy(new_topic_entry->topic, pub_topic, strlen(pub_topic) +1);
        mqtt_topic_register(new_topic_entry);
    }

    /* step 2: publish data */
    if (emcute_pub(&t, data, strlen(data), flags) != EMCUTE_OK) {
        DEBUG("error: unable to publish data to topic %s\n",
                t.name);
        fflush(stdout);
        return 1;
    }

    DEBUG("Published %i bytes to topic '%s [%i]'\n",
            (int)strlen(data), t.name, t.id);

    return 0;
}

/**
 * @brief      authomatically subscribes to a mqtt topic
 *
 * @param      sub_topic  The sub topic
 *
 * @return     status
 */
static int topic_sub(char* sub_topic)
{
    unsigned flags = EMCUTE_QOS_0;

    if (strlen(sub_topic) > TOPIC_MAXLEN) {
        DEBUG("error: topic name exceeds maximum possible size");
        return 1;
    }
    
    /* find empty subscription slot */
    unsigned i = 0;
    for (; (i < MAX_NUM_SUBS) && (subscriptions[i].topic.id != 0); i++) {}
    if (i == MAX_NUM_SUBS) {
        DEBUG("error: no memory to store new subscriptions");
        return 1;
    }

    /* cb is a function used in the emcute_sub_t that reads the data
    @ref emcute.h line 216   */
    subscriptions[i].cb = on_pub_mbed;  // This is the call back function upon receiving some data
    strcpy(topics[i], sub_topic);
    subscriptions[i].topic.name = topics[i];
    
    if (emcute_sub(&subscriptions[i], flags) != EMCUTE_OK) {
        DEBUG("error: unable to subscribe to %s\n", sub_topic);
        return 1;
    }

    DEBUG("Now subscribed to %s\n", sub_topic);
    return 0;
}

/**
 * @brief      Automatically connects to the mqtt-sn broker
 *
 * @param      addr  The address
 * @param      port  The port
 *
 * @return     status 0 on success, 1 on error
 */
static int mqtt_sn_auto_con(char* addr, char* port)
{
    //Uses the udp struct to assign to struct gw
    sock_udp_ep_t gw = { .family = AF_INET6, .port = EMCUTE_PORT };
    //Setting the will topic/message to null 
    char *topic = NULL;
    char *message = NULL;

    size_t len = 0;

    //converts the addr from string and stores it in struct 
    if (ipv6_addr_from_str((ipv6_addr_t *)&gw.addr.ipv6, addr) == NULL) {
        printf("error parsing IPv6 address\n");
        return 1;
    }
    //storing the value of the port in the struct.port
    gw.port = atoi(port);

    /* Continually try to connect to mqtt-sn broker */
    while (1)
    {
        if (emcute_con(&gw, true, topic, message, len, 0) != EMCUTE_OK) {
            DEBUG("error: unable to connect to [%s]:%i\n", addr, (int)gw.port);
            DEBUG("Retrying...\n"); 
        }
        else {
            printf("Successfully connected to gateway at [%s]:%i\n",
                   addr, (int)gw.port);
            return 0;
        }    
    }
}

static void *_mqtt_thread(void *arg)
{
    int mqtt_go = 0;        //send mqtt_go msg to mbed if true
    int sent_hwaddr = 0;    //successfully sent hwaddr over init_info 
    int hdlc_pkt_length;

    /* Pointer to the data received from the MQTT thread */
    mqtt_pkt_t *mqtt_data_rcv;
    /* Receives the pkt from mbed */
    mqtt_pkt_t *mbed_rcv_pkt;
    /* Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in 
    the mbed received pkt */
    void *mbed_rcv_ptr;

    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;
    msg_init_queue(mqtt_thread_queue, sizeof(mqtt_thread_queue));  
    hdlc_entry_t thread2 = { NULL, MQTT_THR_PORT, thread_getpid() };
    hdlc_register(&thread2);

    mqtt_thread_pid = thread_getpid();

    /* Start here */
    /* Initialize subscription buffers */
    memset(subscriptions, 0, (MAX_NUM_SUBS * sizeof(emcute_sub_t)));

    /* start the emcute thread */
    thread_create(stack, sizeof(stack), EMCUTE_PRIO, 0,
                  emcute_thread, mqtt_thread_pid, "emcute");
    
    mqtt_sn_auto_con(MQTT_SN_SERVER, MQTT_SN_PORT) 

    topic_sub(my_hwaddr_topic);

    msg_t msg_snd, msg_rcv;

    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = 0 };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t uart_rcv_hdr;

    int exit = 0;
    while(1)
    {        
        while(1)
        { 
            if (!sent_hwaddr) {
                pub_server[0] = '0';//HWADDR
                for(int i = 1; i < sizeof(my_hwaddr_topic); i++){
                    pub_server[i] = my_hwaddr_topic[i];
                }

                /* tell server I am up and running */
                pub(INIT_INFO_TOPIC, pub_server);
            }

            if (mqtt_go == 1) {   
                mqtt_go = 0;
                uart_hdr.src_port = MQTT_THR_PORT; //PORT 170
                uart_hdr.dst_port = MBED_PORT; //PORT 200
                uart_hdr.pkt_type = MQTT_GO; 
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, &uart_hdr);
                hdlc_snd_pkt.length = UART_PKT_HDR_LEN; //header-only pkt
                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("mqtt_control_thread: the MQTT GO message was not sent to the hdlc thread\n");
                    nsg_rcv.type = HDLC_RESP_RETRY_W_TIMEO;
                    msg_rcv.content.ptr = RTRY_TIMEO_USEC;
                    msg_send_to_self(&msg_rcv);
                } else {
                    DEBUG("mqtt_control_thread: MQTT GO message to mbed has been sent\n");
                }
            }

            //pub to init_info
            if (!sent_hwaddr)
                /* retry until server responds */
                xtimer_msg_receive_timeout(&msg_rcv, 1000000);            
            else
                msg_receive(&msg_rcv);
            
            switch (msg_rcv.type)
            {
                case MQTT_MBED:
                    mqtt_data_rcv = (mqtt_pkt_t *)msg_rcv.content.ptr;

                    /* first message from server on my_hwaddr topic must be a 
                     server ACK so do not service the packet */
                    if (mqtt_go == 0 && sent_hwaddr == 1) {
                        mqtt_go = 1;
                        sent_hwaddr = 0;
                        break;
                    }

                    if(!mbed_initialization_flag)
                        break;
                    //Data to be sent to mbed
                    DEBUG("mqtt_control_thread: MQTT dump to mbed\n");
                    uart_hdr.src_port = MQTT_THR_PORT; //PORT 170
                    uart_hdr.dst_port = MBED_PORT; //PORT 200
                    uart_hdr.pkt_type = MQTT_PKT_TYPE; 
                    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

                    DEBUG("mqtt_control_thread: The data from the MQTT is %s \n", mqtt_data_rcv->data);   
                    DEBUG("mqtt_control_thread: The topic received from the MQTT is %s \n", mqtt_data_rcv->topic);
                    //the size of the hdlc packet
                    
                    hdlc_pkt_length = UART_PKT_DATA_FIELD + sizeof(mqtt_pkt_t);
                    hdlc_snd_pkt.length = hdlc_pkt_length;
                    //copying the data into the hdlc pkt
                    uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, mqtt_data_rcv, sizeof(mqtt_pkt_t));  
                    //taking the message from the mqtt thread and
                    msg_snd.type = HDLC_MSG_SND;
                    msg_snd.content.ptr = &hdlc_snd_pkt;         
                    
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("mqtt_control_thread: HDLC msg queue full\n");
                        continue;
                    } 
                    // DEBUG("mqtt_control_thread: the MQTT packet dump SUCCESS\n");
                    fflush(stdout);
                    break;

                case HDLC_RESP_SND_SUCC:
                    DEBUG("mqtt_control_thread: the MQTT packet dump SUCCESS\n");
                    DEBUG("mqtt_control_thread: sent frame_no %d!\n", frame_no);                    
                    exit = 1;
                    break;

                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    //DEBUG("mqtt_control_thread: retrying frame_no %d\n", frame_no);
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("mqtt_control_thread: HDLC msg queue full!\n");
                        msg_send_to_self(&msg_rcv);
                    }
                    break;

                case HDLC_PKT_RDY:
                    //Received from MBED 
                    DEBUG("mqtt_control_thread: a packet has been received from mbed\n");
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;   
                    uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                    mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;
                    mbed_rcv_pkt = (mqtt_pkt_t *)mbed_rcv_ptr;

                    switch (uart_rcv_hdr.pkt_type)
                    {
                        case MQTT_GO_ACK:
                            DEBUG("mqtt_control_thread: received MQTT_GO_ACK\n");
                            mbed_initialization_flag = 1;
                            break;

                        case MQTT_SUB:
                            DEBUG("mqtt_control_thread: Subscribe Request received from mbed on Topic %s\n", mbed_rcv_pkt->topic);
                            if ( topic_sub(mbed_rcv_pkt->topic) == 0 ){
                                uart_hdr.src_port = MQTT_THR_PORT; //PORT 170
                                uart_hdr.dst_port = MBED_PORT; //PORT 200
                                uart_hdr.pkt_type = MQTT_SUB_ACK; 
                                //adds the uart hdr to the hdlc data
                                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                msg_snd.type = HDLC_MSG_SND;
                                msg_snd.content.ptr = &hdlc_snd_pkt;
                                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                    DEBUG("mqtt_control_thread: (sub ack failed) HDLC msg queue full\n");
                                    // TODO: On the mbed side, if the reply is not received after certain time. Just retry
                                } 
                            }
                            else
                                DEBUG("mqtt_control_thread: Subscribe Request Failed\n");

                            break;

                        case MQTT_PUB:
                            DEBUG("mqtt_control_thread: Mqtt Publish Request Received from MBED with topic: %s and data: %s \n", mbed_rcv_pkt->topic, mbed_rcv_pkt->data);
                            if (pub(mbed_rcv_pkt->topic, mbed_rcv_pkt->data) == 0){
                                uart_hdr.src_port = MQTT_THR_PORT; //PORT 170
                                uart_hdr.dst_port = MBED_PORT; //PORT 200
                                uart_hdr.pkt_type = MQTT_PUB_ACK; 
                                //adds the uart hdr to the hdlc data
                                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                msg_snd.type = HDLC_MSG_SND;
                                msg_snd.content.ptr = &hdlc_snd_pkt;
                                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                    DEBUG("mqtt_control_thread: (pub ack failed) HDLC msg queue full\n");
                                }
                            }
                            else
                                DEBUG("mqtt_control_thread: Publish Request Failed\n");
                            break;
                        default:
                            //error
                            break;
                    }

                    //DEBUG("mqtt_control_thread: received pkt %d\n", hdlc_rcv_pkt->data[UART_PKT_DATA_FIELD]);
                    hdlc_pkt_release(hdlc_rcv_pkt);
                    break;
                default:
                    //error 
                    LED3_ON;
                    break;
            }

            if(exit) {
                exit = 0;
                break;
            }
        }
        
        frame_no++;
    }

    //should be never reached 
    return 0;
    
}

int main(void)
{
    int res; 
    uint8_t hwaddr_long[8];

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    gnrc_netif_get(ifs); 
    res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, hwaddr_long, 
                          sizeof(hwaddr_long));
    char hwaddr_long_str[res * 3];
    gnrc_netif_addr_to_str(hwaddr_long_str, 
            sizeof(hwaddr_long_str), hwaddr_long, res);

    /* Get last 8 characters of l2 addr to subscribe to device topic */
    int i = 1; 
    while (count >= 0)
    {
        if (hwaddr_long_str[strlen(hwaddr_long_str) - i] != ':') {
            my_hwaddr_topic[count] = hwaddr_long_str[strlen(hwaddr_long_str) - i];
            count--;
        }
        i++;
    } 

    my_hwaddr_topic[9] = '\0';
    DEBUG("The Hardware address is %s \n", my_hwaddr_topic);

    msg_init_queue(main_msg_queue, sizeof(main_msg_queue));
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, thread_getpid() };
    hdlc_register(&main_thr);
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(1));
    thread_create(mqtt_thread_stack, sizeof(mqtt_thread_stack), THREAD2_PRIO, 
            THREAD_CREATE_STACKTEST, _mqtt_thread, hdlc_pid, "thread2");
    
    while(1)
    {
        msg_receive(&msg_rcv);

        switch (msg_rcv.type)
        {
            default:
                DEBUG("main_thr: error, should not be receiving msgs.\n");
                LED3_ON;
                break;
        }

    }

    //should be never reached 
    return 0;
}

