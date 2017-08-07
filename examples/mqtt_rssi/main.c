/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
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
 * @brief       Full-duplex hdlc test using a single thread (run on both sides).
 *
 * In this test, the main thread and thread2 thread will contend for the same
 * UART line to communicate to another MCU also running a main and thread2 
 * thread. It seems as though stability deteriorates if the hdlc thread is given
 * a higher priority than the two application threads (RIOT's MAC layer priority
 * is well below the default priority for the main thread. Note that two threads
 * are equally contending for the UART line, one thread may starve the other to 
 * the point where the other thread will continue to retry. Increasing the msg 
 * queue size of hdlc's thread may also increase stability. Since this test can
 * easily stress the system, carefully picking the transmission rates (see below)
 * and tuning the RTRY_TIMEO_USEC and RETRANSMIT_TIMEO_USEC timeouts in hdlc.h
 * may lead to different stability results. The following is one known stable
 * set of values for running this test:
 *
 * -100ms interpacket intervals in xtimer_usleep() below
 * -RTRY_TIMEO_USEC = 200000
 * -RETRANSMIT_TIMEO_USEC 50000
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
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
#include "hdlc.h"
#include "uart_pkt.h"

#include "net/gnrc.h"
#include "net/gnrc/netapi.h"
#include <stdlib.h>
#include "net/emcute.h"
#include "net/ipv6/addr.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "mqtt.h"

#define ENABLE_DEBUG (1)
#include "debug.h"
//setting the priority of the hdlc and thread2
#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)
#define RSSI_DUMP_PRIO          (THREAD_PRIORITY_MAIN - 1)
//setting the port of the main thread
//setting the port of thread2 
#define MAIN_THR_PORT   165
#define THREAD2_PORT    170
#define MBED_PORT       200

/* see openmote-cc2538's periph_conf.h for second UART pin config */
//setting the message queue with message structs
static msg_t thread2_msg_queue[32];
static msg_t main_msg_queue[16];
static msg_t rssi_dump_msg_queue[16];
//creating the stacks
static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];//16896
static char thread2_stack[THREAD_STACKSIZE_MAIN];//16384
static char rssi_dump_stack[THREAD_STACKSIZE_MAIN];//16384

#define EMCUTE_PORT         (1883U)  
#define MQTT_SN_SERVER      ("fd00:dead:beef::1")
#define MQTT_SN_PORT        ("8888")
#define TOPIC               ("init_info")

#define EMCUTE_PRIO         (THREAD_PRIORITY_MAIN - 1)

#define NUMOFSUBS           (16U)  //Define the maximum number of subscriptions
#define TOPIC_MAXLEN        (16U)

static char EMCUTE_ID[9];
static char stack[THREAD_STACKSIZE_DEFAULT];
static emcute_sub_t subscriptions[NUMOFSUBS];
static char topics[NUMOFSUBS][TOPIC_MAXLEN];
static kernel_pid_t thread2_pid;
static kernel_pid_t rssi_pid;


//Global variables 
char send_data[32];
char pub_server[32];
char send_topic[16];
mqtt_pkt_t mqtt_snd_pkt;

static void *emcute_thread(void *arg)
{
    DEBUG("Starting  MQTT thread \n ");
    thread2_pid = (kernel_pid_t)arg;
    emcute_run(EMCUTE_PORT, EMCUTE_ID);
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
    int         topic_len;
    topic_len = strlen(topic->name);

    msg_t       msg_to_mqtt_control_thread;

    memcpy((void *)&send_data, data, data_len);
    send_data[data_len]='\0';
    memcpy((char *)&send_topic, topic->name, topic_len);
    send_topic[topic_len]='\0';

    /* TO DO: add a mutex */

    strncpy(mqtt_snd_pkt.topic, send_topic, topic_len);
    strncpy(mqtt_snd_pkt.data, send_data, data_len);
    msg_to_mqtt_control_thread.type         = MQTT_MBED;    
    mqtt_snd_pkt.topic[topic_len]           = '\0';
    mqtt_snd_pkt.data[data_len]             = '\0';
    msg_to_mqtt_control_thread.content.ptr  = &mqtt_snd_pkt;    

    if (msg_try_send(&msg_to_mqtt_control_thread, thread2_pid))
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
static int auto_pub(char* pub_topic, char* data)
{
    emcute_topic_t t;
    unsigned flags = EMCUTE_QOS_0;
    mqtt_topic_entry_t *new_topic_entry;
    mqtt_topic_entry_t *entry;
    DEBUG("auto_pub: Trying to publish with topic: %s and data %s \n", pub_topic, data);

    /* step 1: get topic id */
    t.name = pub_topic;

    /* First Check if the topic id is available locally. Otherwise get is from the broker*/
    t.id  = mqtt_search_scalar(pub_topic);
    if ( !t.id )
    {
        DEBUG("auto_pub: No registry found %d\n",strlen(pub_topic));

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
static int auto_sub(char* sub_topic)
{
    unsigned flags = EMCUTE_QOS_0;

    if (strlen(sub_topic) > TOPIC_MAXLEN) {
        DEBUG("error: topic name exceeds maximum possible size");
        return 1;
    }
    
    /* find empty subscription slot */
    unsigned i = 0;
    for (; (i < NUMOFSUBS) && (subscriptions[i].topic.id != 0); i++) {}
    if (i == NUMOFSUBS) {
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
 * @return     status
 */
static int auto_con(char* addr, char* port)
{
    //Uses the udp struct to assign to struct gw
    sock_udp_ep_t gw = { .family = AF_INET6, .port = EMCUTE_PORT };
    //Setting the will topic/message to null 
    char        *topic = NULL;
    char        *message = NULL;
    size_t      len = 0;

    //converts the addr from string and stores it in struct 
    if (ipv6_addr_from_str((ipv6_addr_t *)&gw.addr.ipv6, addr) == NULL) {
        printf("error parsing IPv6 address\n");
        return 1;
    }
    //storing the value of the port in the struct.port
    gw.port = atoi(port);

    //calling the Emcute_con function
    if (emcute_con(&gw, true, topic, message, len, 0) != EMCUTE_OK) {
        printf("error: unable to connect to [%s]:%i\n", addr, (int)gw.port);
        return 1;
    }
    else
    {
        printf("Successfully connected to gateway at [%s]:%i\n",
               addr, (int)gw.port);
        return 0;
    }    
}

void reset(void){
    printf("Haven't connected to broker in 1 minute\n");
    printf("RESETTING\n");
    pm_reboot();
}

static void *_mqtt_thread(void *arg)
{
    xtimer_ticks32_t ticks;
    xtimer_t timer = {.target=0, .long_target=0, .callback=reset};
    int                 mqtt_go = 1;//not connected state
    int                 mqtt_connected = 1;    
    int                 sent_hwaddr = 1;//not sent state
    int                 hdlc_pkt_length;
    //Pointer to the data received from the MQTT thread
    mqtt_pkt_t          *mqtt_data_rcv;
    //Receives the pkt from mbed 
    mqtt_pkt_t          *mbed_rcv_pkt;
    //Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in the mbed received pkt
    void                *mbed_rcv_ptr;

    //getting the hdlc_pid from the arg
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;

    //intializing the message queue for thread2
    msg_init_queue(thread2_msg_queue, 32);  

    //Initializing the thread2 to the HDLC   
    hdlc_entry_t thread2 = { NULL, THREAD2_PORT, thread_getpid() };
    //Registering the thread to the list of threads
    hdlc_register(&thread2);

    //storing the thread2 pid to pass as an argument in the EMCUTE thread
    kernel_pid_t thread2_pid = thread_getpid();

    /*Start here */
    //initialize our subscription buffers 
    memset(subscriptions, 0, (NUMOFSUBS * sizeof(emcute_sub_t)));

    // start the emcute thread 
    thread_create(stack, sizeof(stack), EMCUTE_PRIO, 0,
                  emcute_thread, thread2_pid, "emcute");
    ticks = xtimer_ticks_from_usec(RESET_TIMEOUT);
    xtimer_set(&timer,ticks.ticks32);
    
    //automatically connects to the MQTT-SN server  
    
    while (mqtt_go == 1){
        mqtt_go = auto_con(MQTT_SN_SERVER, MQTT_SN_PORT);
    } 
    mqtt_go = 1;


    //automatically connects to the topic init_info and emcute id
    while (mqtt_go == 1)
    {
        mqtt_go = auto_sub(EMCUTE_ID);  
    }
    mqtt_go = 1;
    xtimer_remove(&timer);

    
    //creating a struct to send a message to the rssi thread
    msg_t msg_rssi;    
    //creating two message structs 
    msg_t msg_snd, msg_rcv;

    //frame_no check count
    char frame_no = 0;
    // create packets with max size
    char send_data[HDLC_MAX_PKT_SIZE];
    //creating the hdlc send pkt
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    //creating two uart hdr structs
    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t uart_rcv_hdr;

    //setting up the header for the packets

    //Used as a way to escape the inner loop
    int exit = 0;
    while(1)
    {        
        //if a message has been received 
        while(1)
        { 
            if (sent_hwaddr==1)
            {
                pub_server[0]= HW_ADDR + '0';//HWADDR
                for(int i=0; i<sizeof(EMCUTE_ID);i++){
                    pub_server[i+1]=EMCUTE_ID[i];
                }
                auto_pub(TOPIC, pub_server);
            }
            // DEBUG("In while loop\n");
            if (mqtt_go == 0)
            {   
                mqtt_go = 1;
                sent_hwaddr = 0;
                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                uart_hdr.dst_port = MBED_PORT; //PORT 200
                uart_hdr.pkt_type = MQTT_GO; 
                //adds the uart hdr to the hdlc data
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("mqtt_control_thread: the MQTT GO message was not sent to the hdlc thread\n");
                    mqtt_go = 0;
                } 
                else
                    DEBUG("mqtt_control_thread: MQTT GO message has been sent\n");
                msg_rssi.type = MQTT_RSSI;
                //msg_rssi.content.ptr = &rssi_send;
                if(msg_try_send(&msg_rssi, rssi_pid)){
                    DEBUG("mqtt_control_thread: The RSSI GO message was sent\n");
                }
                else
                    DEBUG("mqtt_control_thread: The RSSI GO message was not sent\n");
                //Publishes to init_info to start the request 
                xtimer_usleep(1000000);
                //sends a request to the server to get the list of connected clients
                auto_pub(TOPIC,"1");
                //sending the MBED the hwaddr of the current node
                hdlc_snd_pkt.data=send_data;
                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                uart_hdr.dst_port = MBED_PORT; //PORT 200
                uart_hdr.pkt_type = HWADDR_GET; 
                //adds the uart hdr to the hdlc data
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);                
                uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, &EMCUTE_ID, sizeof(EMCUTE_ID));                
                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr= &hdlc_snd_pkt;
                //sending the hwaddr of the current node
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("mqtt_control_thread: the MQTT GO message was not sent to the hdlc thread\n");
                                    
                }                 
            }


            //pub to init_info
            if (sent_hwaddr == 1)
                xtimer_msg_receive_timeout(&msg_rcv, 1000000);            
            else
                msg_receive(&msg_rcv);
            
            switch (msg_rcv.type)
            {
                case MQTT_MBED:

                    mqtt_data_rcv = (mqtt_pkt_t *)msg_rcv.content.ptr;
                    if (mqtt_go == 1 && sent_hwaddr == 1)
                    {
                        mqtt_go = 0;
                        sent_hwaddr = 0;
                        break;
                    }
                    //Data to be sent to mbed
                    DEBUG("mqtt_control_thread: MQTT dump to mbed\n");
                    uart_hdr.src_port = THREAD2_PORT; //PORT 170
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
                    if (frame_no==75)
                    {
                        xtimer_usleep(2944000);
                        printf("RIOT:REBOOTING\n");
                        pm_reboot();
                    }                
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
                        case MQTT_SUB:
                            DEBUG("mqtt_control_thread: Subscribe Request received from mbed on Topic %s\n", mbed_rcv_pkt->topic);
                            if ( auto_sub(mbed_rcv_pkt->topic) == 0 ){
                                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                                uart_hdr.dst_port = MBED_PORT; //PORT 200
                                uart_hdr.pkt_type = SUB_ACK; 
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
                            if (auto_pub(mbed_rcv_pkt->topic, mbed_rcv_pkt->data) == 0){
                                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                                uart_hdr.dst_port = MBED_PORT; //PORT 200
                                uart_hdr.pkt_type = PUB_ACK; 
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
                        /*
                        case RESET_RIOT:
                            printf("Resetting RIOT\n");
                            pm_reboot();
                            break;
                            */
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
        
    }

    //should be never reached 
    return 0;
    
}

/*rssi thread starts here*/

/**
 * @brief      Gets the rssi value from the gnrc packet
 *
 * @param      pkt   The packet snip
 *
 * @return     { returns the rssi value of the udp message }
 */

static uint8_t rssi_val(gnrc_pktsnip_t *pkt)
{
    int snips = 0;
    int size = 0;
    uint8_t raw_rssi =0;
    gnrc_netif_hdr_t *hdr;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_NETIF)
        {
            hdr = (gnrc_netif_hdr_t *) (snip->data);
            raw_rssi=hdr->rssi;            
            return raw_rssi;          
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
    ipv6_addr_t addr;
    unsigned int num =1;

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }
    /* parse port */
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return 1;
    }

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        unsigned payload_size;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return 1;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return 1;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return 1;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return 1;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
               port);               
        xtimer_usleep(1000000);       
        return 0;
    }
}

static void *_rssi_dump(void *arg) 
{
    //hdlc pid
    char rssi_data = "hello";
    kernel_pid_t hdlc_pid = (kernel_pid_t) (uintptr_t) arg;
    msg_t msg_send_to_rssi_dump;
    msg_t msg, msg_snd;
    uint8_t rssi_value;
    int rssi_go=0;   
    int i=0; 
    int c=15;   
    char send_data[HDLC_MAX_PKT_SIZE];
    char ipv6_send_addr[24]="fe80::212:4b00:";
    gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               thread_getpid());
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    msg_init_queue(rssi_dump_msg_queue, sizeof(rssi_dump_msg_queue));
    hdlc_entry_t rssi_dump_thr = { .next = NULL, .port = RSSI_THREAD_PORT, 
                                         .pid = thread_getpid() };
    hdlc_register(&rssi_dump_thr); 
    uart_pkt_hdr_t uart_hdr;
    uart_pkt_hdr_t uart_rcv_hdr; 
    char node_ID[9];  

    printf("completed\n" );
    
    while(rssi_go==0)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            case MQTT_RSSI:
                DEBUG("rssi: Go message received\n");
                rssi_go=1;
                break;
        }
    }
    while(1)
    {
        printf("waiting for message\n");
        msg_receive(&msg);
        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                DEBUG("rssi: message from hdlc\n"); 
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg.content.ptr;   
                uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                switch(uart_rcv_hdr.pkt_type){
                    case RESET_RIOT:
                        printf("resetting riot\n");
                        pm_reboot();
                        break;
                    case RSSI_SND:

                        DEBUG("rssi: received the rssi send message\n");                        
                        memcpy(node_ID,hdlc_rcv_pkt->data+UART_PKT_DATA_FIELD, sizeof(node_ID));
                        DEBUG("(if shows more than 8 characters, it is normal)The node to send to is %s\n", node_ID);
                        while (i<8){
                            if (c==19)
                            {
                                ipv6_send_addr[c]=':';
                                c++;
                            }
                            else{                                
                                ipv6_send_addr[c]=node_ID[i];
                                c++;
                                i++;
                            }
                        }
                        ipv6_send_addr[c]='\0';
                        i=0;
                        c=15;
                        DEBUG("The ipv6 is %s\n", ipv6_send_addr); 
                        //sending to the constructed ipv6 address at port 9000 
                                                                    
                        if(rssi_send(ipv6_send_addr,RSSI_DUMP_PORT,rssi_data)==0){
                            printf("udp message sent\n");
                        } 
                        
                        break;  
                }
                hdlc_pkt_release(hdlc_rcv_pkt);
                break;
            case HDLC_RESP_RETRY_W_TIMEO:
                xtimer_usleep(msg.content.value);
                    //DEBUG("mqtt_control_thread: retrying frame_no %d\n", frame_no);
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("mqtt_control_thread: HDLC msg queue full!\n");
                    msg_send_to_self(&msg);
                }
                break;                
            case HDLC_RESP_SND_SUCC:
                DEBUG("_rssi_dump : sent frame \n");
               
                break;
            case GNRC_NETAPI_MSG_TYPE_RCV:                
                rssi_value = rssi_val(msg.content.ptr);                
                printf("rssi:%d\n", rssi_value);
                //sending information to rssi thread on the MBED side                
                uart_hdr.src_port = RSSI_THREAD_PORT; //PORT 220
                uart_hdr.dst_port = RSSI_MBED_DUMP_PORT; //PORT 9111
                uart_hdr.pkt_type = RSSI_DATA_PKT;
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, &rssi_value, sizeof(rssi_value));  

                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;                
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("rssi: HDLC msg queue full\n");
                        continue;
                    } 
                
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                /* just in case */
                printf("gnrc_pkt send\n");
                gnrc_pktbuf_release((gnrc_pktsnip_t *)msg.content.ptr);
                break;

            default:
                /* error */
                DEBUG("rssi_dump: invalid packet\n");
                break;
            }
            
        }    
    

    /* should be never reached */
    DEBUG("Error: Reached Exit!");
    return NULL;
}




int main(void)
{
    /*Getting the hardware address*/
    int             i = 1;
    int             count = 7;
    int             res; //Variable to store the length of the HWADDR
    uint8_t         hwaddr_long[8];

    //Setting up the PID using the ifs interface variable
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    /*Getting Hardware address */
    gnrc_netif_get(ifs); 
    /*Returns the length of the data received*/
    res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, 
                        hwaddr_long, sizeof(hwaddr_long));
    char hwaddr_long_str[res * 3];
    /*storing the string value of hwaddr*/
    gnrc_netif_addr_to_str(hwaddr_long_str, 
            sizeof(hwaddr_long_str), hwaddr_long, res);
    
    while (count >- 1)
    {
        if (hwaddr_long_str[strlen(hwaddr_long_str)-i]!=':')
        {
            EMCUTE_ID[count]=hwaddr_long_str[strlen(hwaddr_long_str)-i];
            count--;
        }
        i++;
    }    
    EMCUTE_ID[8]='\0';
    DEBUG("The Hardware address is %s \n", EMCUTE_ID);

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming packets */
    msg_init_queue(main_msg_queue, 16);
    //intializing the port and pid 
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, thread_getpid() };
    //registering the main_thr to the list 
    hdlc_register(&main_thr);
    //setting the hdlc pid 
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(1));
    
    //Creates the thread 2 from the main thread
    thread_create(thread2_stack, sizeof(thread2_stack), THREAD2_PRIO, 
            THREAD_CREATE_STACKTEST, _mqtt_thread, hdlc_pid, "thread2");

    //creates rssi_dump thread
    rssi_pid = thread_create(rssi_dump_stack, sizeof(rssi_dump_stack), RSSI_DUMP_PRIO, NULL,
                  _rssi_dump, (void *) (uint32_t) hdlc_pid, "rssi_dump");

    //Initializes and starts the udp server on RSSI_DUMP_PORT
    static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);
    uint32_t port=RSSI_DUMP_PORT;
    server.target.pid = rssi_pid;
    server.demux_ctx = port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);


    //The main thread DOES NOT send and receive messages in this example
    //setting up the two message structs 
    msg_t msg_snd, msg_rcv;
    char frame_no = 0;
    //create packets with max size 
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;
    void *main_mbed_rcv_ptr;
    mqtt_pkt_t *main_rcv_pkt;

    // hdr for each pkt is the same for this test 
    uart_hdr.src_port = MAIN_THR_PORT;
    uart_hdr.dst_port = MBED_PORT;
    uart_hdr.pkt_type = HWADDR_GET;
    
    random_init(xtimer_now().ticks32);

    //DEBUG("Main Thread pid is %" PRIkernel_pid "\n", thread_getpid());

    int exit = 0;

    while(1)
    {        
        while(1)
        {
            msg_receive(&msg_rcv);
            DEBUG("Waiting for a message\n");
            switch (msg_rcv.type)
            {
                case HDLC_RESP_SND_SUCC:
                    DEBUG("main_thr: sent frame_no %d!\n", frame_no);
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    DEBUG("main_thr: retrying frame_no %d\n", frame_no);
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("main_thr: HDLC msg queue full!\n");
                        msg_send_to_self(&msg_rcv);
                    }

                    break;
                case HDLC_PKT_RDY:
                /*
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                    printf("The \n packet \n has \n been received\n");  
                    main_mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;
                    main_rcv_pkt = (mqtt_pkt_t *)main_mbed_rcv_ptr;
                    printf("The data received is %s \n", main_rcv_pkt->data);
                    printf("The topic received is %s \n", main_rcv_pkt->topic);
                    break;
                    */
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
        
    }
    //should be never reached 
    
    return 0;
}

