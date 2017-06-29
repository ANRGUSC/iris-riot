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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

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


#define ENABLE_DEBUG (1)
#include "debug.h"
//setting the priority of the hdlc and thread2
#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)
//setting the port of the main thread
//setting the port of thread2 
#define MAIN_THR_PORT   165
#define THREAD2_PORT    170
#define MBED_PORT       200
#define MQTT_PKT_TYPE   0xFF
#define NULL_PKT_TYPE   0xFE
#define SUB_ACK         0xFD
#define PUB_ACK         0xFC

/* see openmote-cc2538's periph_conf.h for second UART pin config */
//setting the message queue with message structs
static msg_t thread2_msg_queue[32];
static msg_t main_msg_queue[16];
//creating the stacks
static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];//16896
static char dispatcher_stack[THREAD_STACKSIZE_MAIN];//16384
static char thread2_stack[THREAD_STACKSIZE_MAIN];//16384

#define EMCUTE_PORT         (1883U)  
#define MQTT_SN_SERVER      ("fd00:dead:beef::1")
#define MQTT_SN_PORT        ("8888")
#define TOPIC               ("init_info")

#define EMCUTE_PRIO         (THREAD_PRIORITY_MAIN - 1)

#define NUMOFSUBS           (16U)  //Define the maximum number of subscriptions
#define TOPIC_MAXLEN        (16U)

static char EMCUTE_ID[8];
static char stack[THREAD_STACKSIZE_DEFAULT];
static emcute_sub_t subscriptions[NUMOFSUBS];
static char topics[NUMOFSUBS][TOPIC_MAXLEN];
static kernel_pid_t thread2_pid;

//Global variables 
char send_data[32];
char pub_server[32];
char send_topic[16];
mqtt_pkt_t mqtt_snd_pkt;
check_mqtt_go=1;
int mqtt_go=1;//not connected state
int sent_hwaddr = 1;//not sent state

static void *emcute_thread(void *arg)
{
    DEBUG("Starting  MQTT thread \n ");
    thread2_pid = (kernel_pid_t)arg;
    emcute_run(EMCUTE_PORT, EMCUTE_ID);
    return NULL;    /* should never be reached */
}

static void on_pub_mbed(const emcute_topic_t *topic, void *data, size_t data_len)
{
    int topic_len = 0;
    while(topic->name[topic_len]!='\0')
    {
        topic_len++;
    }
    msg_t snd_thread1;

    memcpy((void *)&send_data,data,data_len);
    send_data[data_len]='\0';
    memcpy((char *)&send_topic,topic->name, topic_len);
    send_topic[topic_len]='\0';

    /* TO DO: add a mutex */

    strncpy(mqtt_snd_pkt.topic, send_topic, topic_len);
    strncpy(mqtt_snd_pkt.data, send_data, data_len);
    if (strcmp(send_topic, TOPIC)==0)
    {
        snd_thread1.type = MQTT_SN;
    }
    else{
        snd_thread1.type = MQTT_MBED;
    }
    
    mqtt_snd_pkt.topic[topic_len]='\0';
    mqtt_snd_pkt.data[data_len]='\0';
    snd_thread1.content.ptr = &mqtt_snd_pkt;    
    if (msg_try_send(&snd_thread1, thread2_pid))
        {
            DEBUG("Successfully sent to thread 2\n");
        }  
}
    
/*Automatically subscribes to the test topic*/
static int auto_pub(char* pub_topic, char* data)
{
    emcute_topic_t t;
    unsigned flags = EMCUTE_QOS_0;

    printf("pub with topic: %s and data %s \n", pub_topic, data);

    /* step 1: get topic id */
    t.name = pub_topic;
    if (emcute_reg(&t) != EMCUTE_OK) {
        puts("error: unable to obtain topic ID");
        return 1;
    }

    /* step 2: publish data */
    if (emcute_pub(&t, data, strlen(data), flags) != EMCUTE_OK) {
        printf("error: unable to publish data to topic %s\n",
                t.name);
        return 1;
    }

    printf("Published %i bytes to topic '%s [%i]'\n",
            (int)strlen(data), t.name, t.id);

    return 0;
}

static int auto_sub(char* sub_topic)
{
    unsigned flags = EMCUTE_QOS_0;

    if (strlen(sub_topic) > TOPIC_MAXLEN) {
        puts("error: topic name exceeds maximum possible size");
        return 1;
    }
    
    /* find empty subscription slot */
    unsigned i = 0;
    for (; (i < NUMOFSUBS) && (subscriptions[i].topic.id != 0); i++) {}
    if (i == NUMOFSUBS) {
        puts("error: no memory to store new subscriptions");
        return 1;
    }
    /* cb is a function used in the emcute_sub_t that reads the data
    @ref emcute.h line 216   */
    subscriptions[i].cb = on_pub_mbed; 
    strcpy(topics[i], sub_topic);
    subscriptions[i].topic.name = topics[i];
    if (emcute_sub(&subscriptions[i], flags) != EMCUTE_OK) {
        printf("error: unable to subscribe to %s\n", sub_topic);
        return 1;
    }
    printf("Now subscribed to %s\n", sub_topic);
    return 0;
}

static int auto_con(char* addr, char* port)
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

static void *_thread2(void *arg)
{
    int hdlc_pkt_length;
    int switch_get;
    //Pointer to the data received from the MQTT thread
    mqtt_pkt_t *mqtt_data_rcv;
    //Receives the pkt from mbed 
    mqtt_pkt_t *mbed_rcv_pkt;
    //Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in the mbed received pkt
    void *mbed_rcv_ptr;

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
    
    //automatically connects to the MQTT-SN server  
    mqtt_go = auto_con(MQTT_SN_SERVER, MQTT_SN_PORT);
    while (mqtt_go==1){
        mqtt_go = auto_con(MQTT_SN_SERVER,MQTT_SN_PORT);
    } 
    mqtt_go=1;
    //automatically connects to the topic init_info and emcute id
    auto_sub(TOPIC);
    auto_sub(EMCUTE_ID);

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
    uart_hdr.src_port = THREAD2_PORT; //PORT 170
    uart_hdr.dst_port = MBED_PORT; //PORT 200
    uart_hdr.pkt_type = MQTT_GO; 
    //adds the uart hdr to the hdlc data
    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

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
                xtimer_usleep(1000000);
            }
            if (mqtt_go==0)
            {   
                mqtt_go=1;
                msg_snd.type = HDLC_MSG_SND;
                msg_snd.content.ptr = &hdlc_snd_pkt;
                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                    DEBUG("MESSAGE NOT SENT\n");
                } 
                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                uart_hdr.dst_port = MBED_PORT; //PORT 200
                uart_hdr.pkt_type = MQTT_PKT_TYPE; 
                //adds the uart hdr to the hdlc data
                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                xtimer_usleep(1000000);
            }
            //pub to init_info
            msg_receive(&msg_rcv);
            switch (msg_rcv.type)
            {
                case MQTT_SN:
                    //received from MQTT "server"
                    DEBUG("Got from topic: %s \n", TOPIC);
                    mqtt_data_rcv = (mqtt_pkt_t *)msg_rcv.content.ptr;
                    //enum struct for pub to server
                    switch(mqtt_data_rcv->data[0] - '0')
                    {
                        case HW_ADDR:
                            DEBUG("case HW_ADDR\n");
                            //sent_hwaddr=0;
                            //mqtt_go=0;
                            break;
                        case HW_SENT:
                            if (check_mqtt_go==1){
                                DEBUG("Successfully sent hardware address\n");
                                check_mqtt_go=0;
                                sent_hwaddr=0;
                                mqtt_go=0;
                            }
                            break;
                        default:
                            switch_get = mqtt_data_rcv->data[0] - '0';
                            DEBUG("The pubbing has stopped %d \n", switch_get);
                            break;
                    }
                   
                    
                    break;
                case MQTT_MBED:
                    //Data to be sent to mbed
                    DEBUG("MQTT to mbed\n");
                    uart_hdr.src_port = THREAD2_PORT; //PORT 170
                    uart_hdr.dst_port = MBED_PORT; //PORT 200
                    uart_hdr.pkt_type = MQTT_PKT_TYPE; 
                    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                    DEBUG("The data from the MQTT THREAD is %s \n", mqtt_data_rcv->data);   
                    DEBUG("The topic received from the MQTT thread is %s \n", mqtt_data_rcv->topic);
                    //the size of the hdlc packet
                    hdlc_pkt_length = UART_PKT_DATA_FIELD + sizeof(mqtt_pkt_t);
                    hdlc_snd_pkt.length=hdlc_pkt_length;
                    //copying the data into the hdlc pkt
                    uart_pkt_cpy_data(hdlc_snd_pkt.data, HDLC_MAX_PKT_SIZE, mqtt_data_rcv, sizeof(mqtt_pkt_t));  
                    //taking the message from the mqtt thread and
                    msg_snd.type = HDLC_MSG_SND;
                    msg_snd.content.ptr = &hdlc_snd_pkt;         
                    
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        continue;
                    } 
                    fflush(stdout);
                    break;
                case HDLC_RESP_SND_SUCC:
                    DEBUG("The MQTT information has been sent \n");
                    DEBUG("thread2: sent frame_no %d!\n", frame_no);                    
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    //DEBUG("thread2: retrying frame_no %d\n", frame_no);
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("thread2: HDLC msg queue full!\n");
                        msg_send_to_self(&msg_rcv);
                    }
                    break;
                case HDLC_PKT_RDY:
                    //Received from MBED 
                    printf("The packet has been received rom mbed\n");
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;   
                    uart_pkt_parse_hdr(&uart_rcv_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                    mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;
                    mbed_rcv_pkt = (mqtt_pkt_t *)mbed_rcv_ptr;
                    switch (uart_rcv_hdr.pkt_type)
                    {
                        case MQTT_SUB:
                            if (auto_sub(mbed_rcv_pkt->topic)==0){
                            uart_hdr.src_port = THREAD2_PORT; //PORT 170
                            uart_hdr.dst_port = MBED_PORT; //PORT 200
                            uart_hdr.pkt_type = SUB_ACK; 
                            //adds the uart hdr to the hdlc data
                            uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                            msg_snd.type = HDLC_MSG_SND;
                            msg_snd.content.ptr = &hdlc_snd_pkt;
                            if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                DEBUG("MESSAGE NOT SENT\n");
                            } 
                            DEBUG("Topic to SUB %s\n", mbed_rcv_pkt->topic);
                            }
                            break;
                        case MQTT_PUB:
                            printf("The data received is %s \n", mbed_rcv_pkt->data);
                            printf("The topic received is %s \n", mbed_rcv_pkt->topic);
                            if (auto_pub(mbed_rcv_pkt->topic, mbed_rcv_pkt->data)==0){
                                uart_hdr.src_port = THREAD2_PORT; //PORT 170
                                uart_hdr.dst_port = MBED_PORT; //PORT 200
                                uart_hdr.pkt_type = PUB_ACK; 
                                //adds the uart hdr to the hdlc data
                                uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                msg_snd.type = HDLC_MSG_SND;
                                msg_snd.content.ptr = &hdlc_snd_pkt;
                                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                    DEBUG("MESSAGE NOT SENT\n");
                                }
                            }
                            break;
                        default:
                            //error
                            break;
                    }

                    //DEBUG("thread2: received pkt %d\n", hdlc_rcv_pkt->data[UART_PKT_DATA_FIELD]);
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
        xtimer_usleep(10000);
    }

    //should be never reached 
    return 0;
    
}

int main(void)
{
    /*Getting the hardware address*/
    int i=1;
    int count=7;
    int res; //Variable to store the length of the HWADDR
    uint8_t hwaddr_long[8];
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
    while(count>-1)
    {
        if (hwaddr_long_str[strlen(hwaddr_long_str)-i]!=':')
        {
            EMCUTE_ID[count]=hwaddr_long_str[strlen(hwaddr_long_str)-i];
            count--;
        }
        i++;
    }
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
            THREAD_CREATE_STACKTEST, _thread2, hdlc_pid, "thread2");
    
    //The main thread DOES NOT send and receive messages in this example
    //setting up the two message structs 
    msg_t msg_snd, msg_rcv;
    char frame_no = 0;
    //create packets with max size 
    char send_data[HDLC_MAX_PKT_SIZE];//size 16
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;
    void *main_mbed_rcv_ptr;
    mqtt_pkt_t *main_rcv_pkt;

    // hdr for each pkt is the same for this test 
    uart_hdr.src_port = MAIN_THR_PORT;
    uart_hdr.dst_port = MAIN_THR_PORT;
    uart_hdr.pkt_type = MQTT_PKT_TYPE;
    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

    random_init(xtimer_now().ticks32);

    //DEBUG("Main Thread pid is %" PRIkernel_pid "\n", thread_getpid());

    int exit = 0;

    while(1)
    {        
        while(1)
        {
            msg_receive(&msg_rcv);
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
        xtimer_usleep(10000);
    }
    //should be never reached 
    
    return 0;
}

