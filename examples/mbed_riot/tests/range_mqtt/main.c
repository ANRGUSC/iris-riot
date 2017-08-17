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
 * @author      Yutong Gu <yutonggu@usc.edu>
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
#include "mqtt.h"
#include "range.h"
#include "dac.h"
#include "main-conf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"
//setting the priority of the hdlc and thread2
#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)
//setting the port of the main thread
//setting the port of thread2 
#define MAIN_THR_PORT   165
#define THREAD2_PORT    170
#define MBED_PORT       200

/* see openmote-cc2538's periph_conf.h for second UART pin config */
//setting the message queue with message structs
static msg_t main_msg_queue[HDLC_MAX_PKT_SIZE];
//creating the stacks
static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];//16896
static char thread2_stack[THREAD_STACKSIZE_MAIN];//16384

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

//Global variables 
char send_data[32];
char pub_server[32];
char send_topic[16];
mqtt_pkt_t mqtt_snd_pkt;

/**
 * Set channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static int _set_channel(uint16_t channel)
{
    if (channel < 11 || channel > 26) {
        return -1; /* invalid channel number */
    }
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        return gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    }

    return -1; /* fail */
}

/**
 * Get channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static uint16_t _get_channel(void)
{
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    uint16_t channel;
    size_t numof = gnrc_netif_get(ifs);
    if (numof == 1) {
        gnrc_netapi_get(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
        return channel;
    }

    return -1; /* fail */
}

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

    DEBUG("msg received\n");
    memcpy((void *)&send_data, data, data_len);
    send_data[data_len]='\0';
    memcpy((char *)&send_topic, topic->name, topic_len);
    send_topic[topic_len]='\0';

    DEBUG("data: %s\n", send_data);
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
    int error_msg = 0;
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

        error_msg = emcute_reg(&t);

        if (error_msg != EMCUTE_OK) {
            DEBUG("error: unable to obtain topic ID %d", error_msg);
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
    int error_msg;

    DEBUG("Now subscribing");

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
    
    error_msg = emcute_sub(&subscriptions[i], flags);
    if (error_msg != EMCUTE_OK) {
        DEBUG("error: unable to subscribe to %s %d\n", sub_topic, error_msg);
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
    int error_msg;
    //Uses the udp struct to assign to struct gw
    sock_udp_ep_t gw = { .family = AF_INET6, .port = EMCUTE_PORT };
    //Setting the will topic/message to null 
    char        *topic = NULL;
    char        *message = NULL;
    size_t      len = 0;

    //converts the addr from string and stores it in struct 
    if (ipv6_addr_from_str((ipv6_addr_t *)&gw.addr.ipv6, addr) == NULL) {
        DEBUG("error parsing IPv6 address\n");
        return 1;
    }
    //storing the value of the port in the struct.port
    gw.port = atoi(port);
    error_msg = emcute_con(&gw, true, topic, message, len, 0);
    //calling the Emcute_con function
    if (error_msg != EMCUTE_OK) {
        DEBUG("error: unable to connect to [%s]:%i\n", addr, (int)gw.port);
        DEBUG("error message: %d\n", error_msg);
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
    int                 mqtt_go = 1;//not connected state
    int                 mqtt_connected = 1;    
    int                 sent_hwaddr = 1;//not sent state
    int                 hdlc_pkt_length;
    uint16_t            old_channel;

    //pointer to the range_params recieved from the mbed
    range_params_t*     range_params;

    //Pointer to the data received from the MQTT thread
    mqtt_pkt_t          *mqtt_data_rcv;

    //Receives the pkt from mbed 
    mqtt_pkt_t          *mbed_rcv_pkt;
    //Pointer to the hdlc data packet(Starting from the UART_PKT_HDR_LEN) in the mbed received pkt
    void                *mbed_rcv_ptr;

    //getting the hdlc_pid from the arg
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;

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
    while (mqtt_go == 1){
        DEBUG("Connecting...\n");
        mqtt_go = auto_con(MQTT_SN_SERVER, MQTT_SN_PORT);
    } 
    mqtt_go = 1;

    //automatically connects to the topic init_info and emcute id
    while (mqtt_go == 1){
        DEBUG("Subscribing...\n");
        mqtt_go = auto_sub(EMCUTE_ID);
    } 
    mqtt_go = 1;


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
            if (sent_hwaddr == 1)
            {
                pub_server[0] = '0';//HWADDR
                for(int i = 0; i < sizeof(EMCUTE_ID);i ++){
                    pub_server[i + 1] = EMCUTE_ID[i];
                }
                auto_pub(TOPIC, pub_server);
            }
            // DEBUG("In while loop\n");
            if (mqtt_go == 0)
            {   
                mqtt_go = 1;
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
            }

            DEBUG("************************waiting for a message\n");

            //pub to init_info
            if (sent_hwaddr == 1){
                xtimer_msg_receive_timeout(&msg_rcv, 1000000);            
            }
            else{
                msg_receive(&msg_rcv);
            }

            DEBUG("************************got a message\n");
            
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
                            if (auto_pub(mbed_rcv_pkt->topic, mbed_rcv_pkt->data) == 0){
                                uart_hdr.src_port = THREAD2_PORT; //PORT 170
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
                        case SOUND_RANGE_REQ:
                            range_params = (range_params_t *)uart_pkt_get_data(hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                            if(range_params->ranging_mode!= ONE_SENSOR_MODE && 
                                range_params->ranging_mode!= TWO_SENSOR_MODE && 
                                range_params->ranging_mode!= XOR_SENSOR_MODE && 
                                range_params->ranging_mode!= OMNI_SENSOR_MODE){
                                DEBUG("Recieved an invalid ranging mode\n");
                                break;
                            } else{
                                switch(range_params->ranging_mode){
                                    case ONE_SENSOR_MODE:
                                        DEBUG("******************ONE SENSOR MODE*******************\n");
                                        break;
                                    case TWO_SENSOR_MODE:
                                        DEBUG("******************TWO SENSOR MODE*******************\n");
                                        break;
                                    case XOR_SENSOR_MODE:
                                        DEBUG("******************XOR SENSOR MODE*******************\n");
                                        break;
                                    case OMNI_SENSOR_MODE:
                                        DEBUG("******************OMNI SENSOR MODE******************\n");
                                        break;
                                }
                                old_channel = _get_channel();
                                DEBUG("Switching from channel %d to %d\n",old_channel, RSSI_LOCALIZATION_CHAN);
                                _set_channel(RSSI_LOCALIZATION_CHAN);
                                range_and_send(range_params, hdlc_pid, THREAD2_PORT, uart_rcv_hdr.src_port);
                                _set_channel(old_channel);
                                DEBUG("Switching from channel %d to %d\n",RSSI_LOCALIZATION_CHAN, old_channel);
                                DEBUG("******************RANGING COMPLETED******************\n");
                            }        
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
        // xtimer_usleep(10000);
    }

    //should be never reached 
    return 0;
    
}

int main(void)
{
    /*Getting the hardware address*/
    int             i = 1;
    int             count = 7;
    int             res; //Variable to store the length of the HWADDR
    uint8_t         hwaddr_long[8];

    DEBUG("starting DAC\n");
    if(init_dac(DEFAULT_DAC_CS, SPI_CLK_400KHZ) == SPI_OK){
        DEBUG("Setting voltage at %d%%\n", DEFAULT_SENSOR_THRESH*100/255);
        set_voltage((uint8_t) DEFAULT_SENSOR_THRESH, DAC_GAIN_1);
        stop_dac();

    } else{
        DEBUG("SPI failed\n");
    }

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
    
    while (count > -1)
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
    msg_init_queue(main_msg_queue, HDLC_MAX_PKT_SIZE);
    //intializing the port and pid 
    hdlc_entry_t main_thr = { NULL, MAIN_THR_PORT, thread_getpid() };
    //register  ing the main_thr to the list 
    hdlc_register(&main_thr);
    //setting the hdlc pid 
    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(0));

    
    //Creates the thread 2 from the main thread
    thread_create(thread2_stack, sizeof(thread2_stack), THREAD2_PRIO, 
            THREAD_CREATE_STACKTEST, _mqtt_thread, hdlc_pid, "thread2");
    
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
                    DEBUG("The \n packet \n has \n been received\n");  
                    main_mbed_rcv_ptr = hdlc_rcv_pkt->data + UART_PKT_DATA_FIELD;
                    main_rcv_pkt = (mqtt_pkt_t *)main_mbed_rcv_ptr;
                    DEBUG("The data received is %s \n", main_rcv_pkt->data);
                    DEBUG("The topic received is %s \n", main_rcv_pkt->topic);
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

