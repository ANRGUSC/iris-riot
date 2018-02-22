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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

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
#include "mqtt_thread.h"
#include "mqtt.h"
#include "main-conf.h"
#include "mbox.h"


#define ENABLE_DEBUG (0)
#include "debug.h"

/* see openmote-cc2538's periph_conf.h for second UART pin config */
//setting the message queue with message structs
static msg_t mqtt_msg_queue[HDLC_MSG_QUEUE_SIZE];
//creating the stacks

#define EMCUTE_PORT         (1883U)  
#define MQTT_SN_SERVER      ("fd00:dead:beef::1")
#define MQTT_SN_PORT        ("8888")
#define TOPIC               ("init_info")
#define TOPIC_CONT          ("common")


#define EMCUTE_PRIO         (THREAD_PRIORITY_MAIN - 1 )

#define NUMOFSUBS           (16U)  //Define the maximum number of subscriptions
#define TOPIC_MAXLEN        (16U)
#define EMCUTE_ID_LEN       (8)

static char mqtt_m3pi_ID[9];
static char emcute_stack[THREAD_STACKSIZE_DEFAULT];
static char topics[NUMOFSUBS][TOPIC_MAXLEN];
static emcute_sub_t subscriptions[NUMOFSUBS];
static kernel_pid_t mqtt_thread_pid;

int  mqtt_state = MQTT_DISCON;

//Global variables 
char pub_server[32];

#define MAX_MQTT_BUFF_SIZE  5

typedef struct {
    mbox_t mbox;
    msg_t msgs[HDLC_MSG_QUEUE_SIZE];
} mqtt_mbox_t;

mqtt_mbox_t mqtt_mbox;
mqtt_pkt_dup_t mqtt_snd_pkt_buf[MAX_MQTT_BUFF_SIZE];
static int mqtt_buf_cnt = 0;

static void *emcute_thread(void *arg)
{
    mqtt_thread_pid = (kernel_pid_t)arg;
    DEBUG("Starting MQTT thread %d \n ", mqtt_thread_pid);

    emcute_run(EMCUTE_PORT, mqtt_m3pi_ID);
    return NULL;    /* should never be reached */
}

/**
 * @brief      callback function for subscibing to a topic. When some data is published in that topic, this function is called.
 *
 * @param[in]  topic     The topic
 * @param      data      The data
 * @param[in]  data_len  The data length
 */

static void on_mqtt_data_recv(const emcute_topic_t *topic, void *data, size_t data_len)
{
    int topic_len = strlen(topic->name);
    DEBUG("on_mqtt_data_recv : mqtt_thread_pid %d\n",mqtt_thread_pid);  

    static msg_t msg_to_mqtt_control_thread;

    msg_to_mqtt_control_thread.type         = MQTT_MBED;    
    msg_to_mqtt_control_thread.content.ptr  = &mqtt_snd_pkt_buf[mqtt_buf_cnt];    
    
    mqtt_snd_pkt_buf[mqtt_buf_cnt].topic = topic->name;
    strncpy(mqtt_snd_pkt_buf[mqtt_buf_cnt].data, data, data_len);
    mqtt_snd_pkt_buf[mqtt_buf_cnt].data[data_len] = '\0';

    if ( get_mqtt_state () == MQTT_MBED_INIT_DONE ) // Only when the initialization is done use the maibox
    {
        if (mbox_try_put(&mqtt_mbox, &msg_to_mqtt_control_thread)){
            DEBUG("on_mqtt_data_recv : Successfully sent to the mqtt control thread via mbox %d, topic %s, count %d\n",mqtt_thread_pid, topic->name, mqtt_buf_cnt);  
            mqtt_buf_cnt ++;
            if (mqtt_buf_cnt >= MAX_MQTT_BUFF_SIZE)
                mqtt_buf_cnt = 0;
        }
        else
            DEBUG("on_mqtt_data_recv : Failed to send to to the mqtt control thread, topic %s\n",topic->name);  
    }
    else //Before the initalization is done, use the normal msg
    {
        if (msg_try_send(&msg_to_mqtt_control_thread, mqtt_thread_pid)){
            DEBUG("on_mqtt_data_recv : Successfully sent to the mqtt control thread %d, topic %s, count %d\n",mqtt_thread_pid, topic->name, mqtt_buf_cnt);  
            mqtt_buf_cnt ++;
            if (mqtt_buf_cnt >= MAX_MQTT_BUFF_SIZE)
                mqtt_buf_cnt = 0;
        }
        else
            DEBUG("on_mqtt_data_recv : Failed to send to to the mqtt control thread, topic %s\n",topic->name);

    }

   return;
}
    
/**
 * @brief      publish to a topic
 *
 * @param      pub_topic  The pub topic
 * @param      data       The data
 *
 * @return     status
 */
static int mqtt_pub(char* pub_topic, char* data)
{
    emcute_topic_t t;
    unsigned flags = EMCUTE_QOS_0;
    mqtt_topic_entry_t *new_topic_entry;

    DEBUG("mqtt_pub: Trying to publish with topic: %s and data %s \n", pub_topic, data);

    /* step 1: get topic id */
    t.name = pub_topic;

    /* First Check if the topic id is available locally. Otherwise get is from the broker*/
    t.id  = mqtt_search_scalar(pub_topic);
    if ( !t.id )
    {
        DEBUG("mqtt_pub: No registry found %d\n",strlen(pub_topic));

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
static int mqtt_sub(char* sub_topic)
{
    unsigned flags = EMCUTE_QOS_0;

    if (strlen(sub_topic) > TOPIC_MAXLEN) {
        DEBUG("error: topic name exceeds maximum possible size");
        return 1;
    }
    
    /* find empty subscription slot */
    unsigned int i = 0;
    for (; (i < NUMOFSUBS) && (subscriptions[i].topic.id != 0); i++) {}
    if (i == NUMOFSUBS) {
        DEBUG("error: no memory to store new subscriptions");
        return 1;
    }

    /* cb is a function used in the emcute_sub_t that reads the data
    @ref emcute.h line 216   */
    subscriptions[i].cb = on_mqtt_data_recv;  // This is the call back function upon receiving some data
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
 * @return     status
 */
static int mqtt_sn_try_connect(char* addr, char* port)
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

    //calling the Emcute_con function
    if (emcute_con(&gw, true, topic, message, len, 0) != EMCUTE_OK) {
        DEBUG("error: unable to connect to [%s]:%i\n", addr, (int)gw.port);
        return 1;
    }
    else
    {
        DEBUG("Successfully connected to gateway at [%s]:%i\n",
               addr, (int)gw.port);
        return 0;
    }    
}

static void *_mqtt_thread(void *arg)
{
    mqtt_pkt_dup_t  *mbed_rcv_pkt;
    //Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in the mbed received pkt

    //getting the hdlc_pid from the arg
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;

    //intializing the message queue for the thread
    msg_init_queue(mqtt_msg_queue, HDLC_MSG_QUEUE_SIZE);  

    //Initializing the thread2 to the HDLC   
    hdlc_entry_t thread2 = { NULL, RIOT_MQTT_PORT, thread_getpid() };
    //Registering the thread to the list of threads
    hdlc_register(&thread2);

    //storing the thread2 pid to pass as an argument in the EMCUTE thread

    /*Start here */
    //initialize our subscription buffers 
    memset(subscriptions, 0, (NUMOFSUBS * sizeof(emcute_sub_t)));

    // start the emcute thread 
    thread_create(emcute_stack, sizeof(emcute_stack), EMCUTE_PRIO, 0,
                  emcute_thread, thread_getpid(), "emcute");
    
    xtimer_usleep(2000);   
    DEBUG("\n mqtt_control_thread: trying to connect\n");
    //automatically connects to the MQTT-SN server  
    while (mqtt_sn_try_connect(MQTT_SN_SERVER, MQTT_SN_PORT) != 0){
        xtimer_usleep(100);   
    }

    set_mqtt_state( MQTT_CON_PUB_HW );

    //automatically connects to the topic init_info and emcute id
    mqtt_sub(mqtt_m3pi_ID);

    /*
     * TODO? Optional extra subscriptions to app specific topics
     */
    {
        mqtt_sub(TOPIC_CONT);
    }

    msg_t msg_snd;
    msg_t msg_rcv;
    msg_t msg_to_prc;


    char frame_no = 0;
    // create packets with max size
    char send_data[HDLC_MAX_PKT_SIZE];
    //creating the hdlc send pkt
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;

    //creating two uart hdr structs
    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t uart_rcv_hdr;
    
    int exit = 0;

    pub_server[0] = '0'; //This is used the server to identify that it is the HWADDR
    for(int i = 0; i < EMCUTE_ID_LEN; i++){
        pub_server[i + 1] = mqtt_m3pi_ID[i];
    }
    
    /**
     * This postion contains initialization of the mbed for the overlay network
     */
    while(1)
    { 
        if (get_mqtt_state() == MQTT_CON_PUB_HW){               
            mqtt_pub(TOPIC, pub_server);
        }

        // DEBUG("In while loop\n");
        if (get_mqtt_state() == MQTT_CON_MQTT_GO)
        {   
            uart_hdr.src_port   = RIOT_MQTT_PORT; //PORT 170
            uart_hdr.dst_port   = MBED_MQTT_PORT; //PORT 200
            uart_hdr.pkt_type   = MQTT_GO; 
            hdlc_snd_pkt.length = UART_PKT_HDR_LEN;
            //adds the uart hdr to the hdlc data
            uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
            
            msg_snd.type        = HDLC_MSG_SND;
            msg_snd.content.ptr = &hdlc_snd_pkt;

            if(!msg_try_send(&msg_snd, hdlc_pid)) 
                DEBUG("mqtt_control_thread: the MQTT GO message was not sent to the hdlc thread\n");
            else{
                DEBUG("mqtt_control_thread: MQTT GO message has been sent\n");
                set_mqtt_state( MQTT_CON_MQTT_GO_WAIT );
            }
        }

        //pub to init_info
        if (get_mqtt_state() == MQTT_CON_PUB_HW)
            xtimer_msg_receive_timeout(&msg_rcv, 1000000);            
        else
            msg_receive(&msg_rcv);
        
        // DEBUG("received msg %d \n", msg_rcv.type);
        switch (msg_rcv.type)
        {
            case MQTT_MBED:
                if (get_mqtt_state() <= MQTT_CON_MQTT_GO_WAIT){
                    set_mqtt_state( MQTT_CON_MQTT_GO );
                    break;
                }
                break;

            case HDLC_RESP_SND_SUCC:
                // DEBUG("mqtt_control_thread: the MQTT packet dump SUCCESS\n");
                DEBUG("mqtt_control_thread: sent frame_no %d!\n", frame_no);                    
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
                // DEBUG("mqtt_control_thread: a packet has been received from mbed\n");
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;   
                uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                mbed_rcv_pkt = (mqtt_pkt_t *) (hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD);

                switch (uart_rcv_hdr.pkt_type)
                {
                    case MQTT_GO_ACK:
                        set_mqtt_state( MQTT_CON_COMPLETE );
                        DEBUG("mqtt_control_thread: received MQTT_GO_ACK\n");
                        //Publishes to init_info to start the request 
                        //sends a request to the server to get the list of connected clients
                        //sending the MBED the hwaddr of the current node
                        xtimer_usleep(10000);       
                        hdlc_snd_pkt.data   = send_data;
                        uart_hdr.src_port   = RIOT_MQTT_PORT; //PORT 170
                        uart_hdr.dst_port   = MBED_MQTT_PORT; //PORT 200
                        uart_hdr.pkt_type   = HWADDR_GET; 
                        hdlc_snd_pkt.length = UART_PKT_HDR_LEN + EMCUTE_ID_LEN + 1;
                        //adds the uart hdr to the hdlc data
                        uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);                
                        uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, &mqtt_m3pi_ID, EMCUTE_ID_LEN);                
                        msg_snd.type        = HDLC_MSG_SND;
                        msg_snd.content.ptr = &hdlc_snd_pkt;
                        //sending the hwaddr of the current node
                        if(!msg_try_send(&msg_snd, hdlc_pid))
                            DEBUG("mqtt_control_thread: the HWADDR was not sent to the hdlc thread\n");  
                        break;

                    case HWADDR_ACK:
                        DEBUG("mqtt_control_thread: received HWADDR_ACK\n");
                        set_mqtt_state( MQTT_MBED_INIT_DONE );
                        exit = 1;
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
        frame_no++;
    }
    //control transmission rate via interpacket intervals 
    // xtimer_usleep(1000000);

    // At this point all the intialization of the mbed and the openmote should be done.
    // 
    printf("mqtt_control_thread: initialization done\n");

    /**
     * The following is the main body of our mqtt code
     */
    while(1)
    {       
        /**
         * Get the next packet to process
         */
        mbox_get(&mqtt_mbox, &msg_to_prc);

        mbed_rcv_pkt = (mqtt_pkt_dup_t *)msg_to_prc.content.ptr;
        //Data to be sent to mbed
        // DEBUG("mqtt_control_thread: MQTT dump to mbed\n");
        uart_hdr.src_port   = RIOT_MQTT_PORT; //PORT 170
        uart_hdr.dst_port   = MBED_MQTT_PORT; //PORT 200
        uart_hdr.pkt_type   = MQTT_PKT_TYPE;
        hdlc_snd_pkt.length = UART_PKT_HDR_LEN + sizeof(mqtt_pkt_t);
        uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
        DEBUG("mqtt_control_thread: MQTT dump to mbed. Topic %s and data %s \n",
                                mbed_rcv_pkt->topic, mbed_rcv_pkt->data);   
        uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, mbed_rcv_pkt->topic, 16);  
        uart_pkt_cpy_data(hdlc_snd_pkt.data + 16, HDLC_MAX_PKT_SIZE, mbed_rcv_pkt->data, 32);
        
        msg_snd.type        = HDLC_MSG_SND;
        msg_snd.content.ptr = &hdlc_snd_pkt;         
        
        if(!msg_try_send(&msg_snd, hdlc_pid)) {
            DEBUG("mqtt_control_thread: HDLC msg queue full\n");
            continue;
        } 
        // DEBUG("mqtt_control_thread: the MQTT packet dump SUCCESS\n");

        while(1)
        { 
            msg_receive(&msg_rcv);
            // DEBUG("received msg %d \n", msg_rcv.type);
            switch (msg_rcv.type)
            {
                case HDLC_RESP_SND_SUCC:
                    // DEBUG("mqtt_control_thread: the MQTT packet dump SUCCESS\n");
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
                    // DEBUG("mqtt_control_thread: a packet has been received from mbed\n");
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;   
                    uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                    mbed_rcv_pkt = (mqtt_pkt_t *) (hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD);

                    switch (uart_rcv_hdr.pkt_type)
                    {
                        case MQTT_SUB:
                            DEBUG("mqtt_control_thread: Subscribe Request received from mbed on Topic %s\n", mbed_rcv_pkt->topic);
                            if ( mqtt_sub(mbed_rcv_pkt->topic) == 0 ){
                                uart_hdr.src_port   = RIOT_MQTT_PORT; //PORT 170
                                uart_hdr.dst_port   = MBED_MQTT_PORT; //PORT 200
                                uart_hdr.pkt_type   = MQTT_SUB_ACK; 
                                hdlc_snd_pkt.length = UART_PKT_HDR_LEN;
                                //adds the uart hdr to the hdlc data
                                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                msg_snd.type        = HDLC_MSG_SND;
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
                            if (mqtt_pub(mbed_rcv_pkt->topic, mbed_rcv_pkt->data) == 0){
                                uart_hdr.src_port   = RIOT_MQTT_PORT; //PORT 170
                                uart_hdr.dst_port   = MBED_MQTT_PORT; //PORT 200
                                uart_hdr.pkt_type   = MQTT_PUB_ACK; 
                                hdlc_snd_pkt.length = UART_PKT_HDR_LEN;
                                //adds the uart hdr to the hdlc data
                                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                msg_snd.type    = HDLC_MSG_SND;
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

        //control transmission rate via interpacket intervals 
        // xtimer_usleep(1000000);
    }

    //should be never reached 
    return 0;
    
}

/**
 * @brief      intializes the mqtt control thread as well as the emcute thread
 *
 * @param      stack      The stack
 * @param[in]  stacksize  The stacksize
 * @param[in]  priority   The priority
 * @param[in]  name       The name
 * @param      arg        The argument
 *
 * @return     the pid of the mqtt_control thread
 */
kernel_pid_t mqtt_thread_init(char *stack, int stacksize, char priority, const char *name,  void *arg)
{ 
    /** 
     * Getting Hardware address 
     */
    
    int ifs_res; //Variable to store the length of the HWADDR
    uint8_t hwaddr_long[8];
    
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    gnrc_netif_get(ifs); 
    /*Returns the length of the data received*/
    ifs_res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, 
                        hwaddr_long, sizeof(hwaddr_long));
    char hwaddr_long_str[ifs_res * 3];

    /*storing the string value of hwaddr*/
    gnrc_netif_addr_to_str(hwaddr_long_str, 
            sizeof(hwaddr_long_str), hwaddr_long, ifs_res);

    /**
     * Get the last 8 characters from the ipv6 address
     */
    int i = 1, count = 7;
 
    mbox_init(&mqtt_mbox.mbox, mqtt_mbox.msgs, HDLC_MSG_QUEUE_SIZE);

    while (count >- 1){
        if (hwaddr_long_str[strlen(hwaddr_long_str) - i] != ':'){
            mqtt_m3pi_ID[count] = hwaddr_long_str[strlen(hwaddr_long_str)-i];
            count--;
        }
        i++;
    }
    mqtt_m3pi_ID[8] = '\0';

    DEBUG("The Hardware address is %s \n", mqtt_m3pi_ID);

    kernel_pid_t res;

    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                                         _mqtt_thread, arg, name);
    if (res <= 0) 
        return -EINVAL;

    return res;
}

static mutex_t state_mutex;

int get_mqtt_state (void){
    mutex_lock(&state_mutex);
    // DEBUG("get state\n");
    int state = mqtt_state;
    mutex_unlock(&state_mutex);
    return state;
}

void set_mqtt_state (int state){
    mutex_lock(&state_mutex);
    mqtt_state = state;
    mutex_unlock(&state_mutex);
}

void get_node_id (char *ret)
{
    strcpy(ret, mqtt_m3pi_ID); 
}
