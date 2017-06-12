/*
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

#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "net/netdev2.h"
#include "timex.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/adc.h"

#include "thread.h"
#include "msg.h"

#define TX_POWER            7
#define TX_NODE_IPV6_ADDR   "fe80::212:4b00:613:622" //fe80::212:4b00:433:ed81"
#define RX_NODE_IPV6_ADDR   "fe80::212:4b00:433:ece1" //"fe80::212:4b00:433:ed4f"

#define CLIENT_PORT         8000
#define SERVER_PORT         8888

#define MAX_ADDR_LEN        (8U)
#define RANGE_RX_HW_ADDR    "ff:ff"
#define QUEUE_SIZE          8
#define CC2538_RSSI_OFFSET  73

#define RANGE_REQ_FLAG      0x12
#define RANGE_RDY_FLAG      0x34
#define RANGE_GO_FLAG       0x56
#define TX_NODE_ID 0x00

typedef struct {
    int* num_threads;
    int* stop_flag;
    int udelay;
    int adc_line;
    int adc_res;
    int adc_shift;
    int sample_size;
} scan_rx_param;

int QUIT_RX_SCAN_FLAG = 0;
int NUM_SCAN_RX_THREADS = 0;

char scan_rx_stack[THREAD_STACKSIZE_MAIN];

scan_rx_param param;
int udelay;
uint8_t _tx_node_id      = 0;
unsigned int adc_line;
int socadc_rshift        = 0;
int adc_res              = 0; /* default resolution */
int adc_shift;
int sample_size = -1;

extern int ranging_on;

static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
                                        KERNEL_PID_UNDEF};

/* this is used to unregister the thread from receiving UDP packets sent to the 
   port in the "server" struct */
static void _unregister_thread(void)
{
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.pid = KERNEL_PID_UNDEF;
}
/*----------------------------------------------------------------------------*/
int range_rx(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s <ultrasound_thresh>\n", argv[0]);
        return 1;
    }

    if (atoi(argv[1]) <= 0)
    {
        puts("error: please input value greater than 0");
        return 1;
    }

    int ultrasound_thresh = atoi(argv[1]);
    unsigned long time_diff = 0;
    char *tx_node_addr_str = TX_NODE_IPV6_ADDR;
    ipv6_addr_t tx_node_ip_addr;
    gnrc_pktsnip_t *pkt, *snip;
    int16_t tx_power = TX_POWER;
    gnrc_pktsnip_t *payload, *udp, *ip;
    char buf[2] = {0x00, 0x00};

    /* register this thread to the chosen UDP port */
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    if (ipv6_addr_from_str(&tx_node_ip_addr, tx_node_addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }

    /* send ultrasound ranging request */
    buf[0] = RANGE_REQ_FLAG;
    buf[1] = TX_NODE_ID;
    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, CLIENT_PORT, SERVER_PORT);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, &tx_node_ip_addr);
    if (ip == NULL) {
        puts("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
        puts("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    printf("REQ signal sent to %s\n",tx_node_addr_str);

    /* wait for "RDY" packet */
    while(1) {   
        puts("Waiting for RDY pkt.");
        int response= xtimer_msg_receive_timeout(&msg,1000000);
        if(response < 0){
            puts ("Timed out");
             _unregister_thread();
            return 1;
        }
        if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
            pkt = msg.content.ptr;

            /* get snip containing packet data where we put the packet number */
            snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
            if ( RANGE_RDY_FLAG == ((uint8_t *) snip->data)[0] && 
                 TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                puts("Got init msg. Turning on ranging mode.");
                break;
            } else {
                puts("Unknown packet.");
            }

            gnrc_pktbuf_release(pkt);
        } 
    } /* while */

    /* send "GO" packet */
    buf[0] = RANGE_GO_FLAG;
    buf[1] = TX_NODE_ID;
    payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
    if (payload == NULL) {
        puts("Error: unable to copy data to packet buffer");
        return 1;
    }

    udp = gnrc_udp_hdr_build(payload, CLIENT_PORT, SERVER_PORT);
    if (udp == NULL) {
        puts("Error: unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 1;
    }

    ip = gnrc_ipv6_hdr_build(udp, NULL, &tx_node_ip_addr);
    if (ip == NULL) {
        puts("Error: unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 1;
    }

    range_rx_init(TX_NODE_ID, ultrasound_thresh, AD4_PIN, ADC_RES_7BIT, 2000);

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, 
        ip)) {
        puts("Error: unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 1;
    }

    //xtimer_sleep(1);
    while(ranging_on==1){
        xtimer_usleep(10000);
    }

    if( (time_diff = range_rx_stop()) > 0 ) {
        printf("TDoA = %lu\n", time_diff);
    } else {
        puts("Ranging failed.");
    }

    _unregister_thread();

    return 0;
}
/*----------------------------------------------------------------------------*/
int range_tx(int argc, char **argv)
{
    /* for sending L2 pkt */
    kernel_pid_t dev;
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    gnrc_pktsnip_t *pkt, *hdr, *snip;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;    

    /* for sending udp pkt */
    char *rx_node_addr_str = RX_NODE_IPV6_ADDR;
    char buf[3] = {0x00, 0x00, 0x00};
    ipv6_addr_t rx_node_ip_addr;
    int16_t tx_power = TX_POWER;
    gnrc_pktsnip_t *payload, *udp, *ip;

    /* enable output on Port D pin 3 */
    if(gpio_init(GPIO_PD3, GPIO_OUT) < 0) {
        puts("Error initializing GPIO_PIN.");
        return 1;
    }

    gpio_clear(GPIO_PD3);

    /* register this thread to the chosen UDP port */
    server.next = NULL;
    server.demux_ctx = (uint32_t) SERVER_PORT; 
    server.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        /* FIXME: mising error checks for both */
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    if (ipv6_addr_from_str(&rx_node_ip_addr, rx_node_addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return 1;
    }
    
    /* ultrasound transmitter is always ready for request (infinite loop) */
    while(1) {
        puts("Waiting for REQ signal"); //error
start: 
        
        /* wait until for ranging request packet */
        while(1) {
            msg_receive(&msg);

            if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
                pkt = msg.content.ptr;

                /* get snip containing packet data where we put the packet number */
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
                if ( RANGE_REQ_FLAG == ((uint8_t *)snip->data)[0] && 
                        TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                    puts("Got REQ. Sending 'RDY' pkt now!"); //error!
                    break;
                } else {
                    puts("Not a ranging request packet.");
                }

                gnrc_pktbuf_release(pkt);
            }
        }

        /* send "READY" message */
        buf[0] = RANGE_RDY_FLAG;
        buf[1] = TX_NODE_ID;
        payload = gnrc_pktbuf_add(NULL, &buf, 2, GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return 1;
        }

        udp = gnrc_udp_hdr_build(payload, (uint16_t) CLIENT_PORT, (uint16_t) SERVER_PORT);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return 1;
        }

        ip = gnrc_ipv6_hdr_build(udp, NULL, &rx_node_ip_addr);
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

        /* wait for "GO" packet */
        int retries = 0;

        while(1){
            msg_receive(&msg);

            if (msg.type == GNRC_NETAPI_MSG_TYPE_RCV) {
                pkt = msg.content.ptr;
                snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);

                if ( RANGE_GO_FLAG == ((uint8_t *)snip->data)[0] && 
                        TX_NODE_ID == ((uint8_t *)snip->data)[1] ) {
                    puts("Got 'GO' pkt. Time to send RF/Ultrasound ping!");
                    break;
                } else if(retries > 2) {
                    /* for testing purposes */
                    puts("No valid packets. Resetting...");
                    gnrc_pktbuf_release(pkt);
                    goto start;
                } else {
                    puts("Not a ranging request packet.");
                }

                gnrc_pktbuf_release(pkt);
            }

            ++retries;
        }

        range_tx_init(GPIO_PD3);

        /** Send L2 Packet **/
        /* network interface */
        dev = ifs[0];
        hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);

        if (hw_addr_len == 0) {
            if (strcmp(argv[2], "bcast") == 0) {
                flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
            } else {
                puts("error: invalid address given");
                return 1;
            }
        }

        /* put packet together */
        buf[0] = RANGE_FLAG_BYTE0;
        buf[1] = RANGE_FLAG_BYTE1;
        buf[2] = TX_NODE_ID;
        pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            puts("error: packet buffer full");
            return 1;
        }
       
        hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
        if (hdr == NULL) {
            puts("error: packet buffer full");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        LL_PREPEND(pkt, hdr);
        nethdr = (gnrc_netif_hdr_t *)hdr->data;
        nethdr->flags = flags;
        /* ready to send */

        //make sure no packets are to be sent!!
        if (gnrc_netapi_send(dev, pkt) < 1) {
            puts("error: unable to send");
            gnrc_pktbuf_release(pkt);
            return 1;
        }
        
        range_tx_off(); //turn off just in case
        puts("RF and ultrasound pings sent");        
    } /* end while (infinite loop) */

    /* should never reach here! */
    _unregister_thread();
    return 0; 
}

int scan_tx(int argc, char **argv)
{

    puts("Initializing GPIO_PD3");
    /* enable output on Port D pin 3 */
    if(gpio_init(GPIO_PD3, GPIO_OUT) < 0) {
        puts("Error initializing GPIO_PIN.");
        return 1;
    }

    puts("Clearing GPIO_PD3");
    gpio_clear(GPIO_PD3);

    puts("Pinging...");
    /* set pin to 1 for around 50uS */
    gpio_set(GPIO_PD3);

    /* ultrasound ping should execute 20.5msec after gpio pin goes up */
    xtimer_spin(100);

    puts("Enter 'kill' to kill signal");
    char str[40];
    gets(str);

    if(strcmp(str, "kill") == 0)
    {
        // range_tx_off();
        gpio_clear(GPIO_PD3);
        puts("finished");
        return 0;
    }

    return 0;
}

/*************************************************/
//scan_rx stuff


void *scan_rx_thread(void *arg)
{
    scan_rx_param* param = (scan_rx_param*) arg;
    int low = 30; // changed low to 30 instead of 35 - Richard
    int med = 50;
    int high = 60;
    int pinged = 0;

    printf("Started scan_rx_thread\n");
    int adcsample = 0;
    int i=0;
    int j=0;
    int ping_rcvd=0;
    int sample_size = param->sample_size;
    double avg = 0;

    int num_ping_rcvd=0;
    
    (*(param->num_threads))++;

    switch(param->adc_res)
    {
        case ADC_RES_7BIT:
            break;
        case ADC_RES_9BIT:
            med *= 4;
            high *= 4;
            break;
        case ADC_RES_10BIT:
            med *= 8;
            high *= 8;
            break;
        case ADC_RES_12BIT:
            med *= 10;
            high *= 10;
            break;
        default:
            return NULL;
    }

    printf("Flag value: %d\n",*(param->stop_flag));

    while (adcsample < high && *(param->stop_flag) == 0){
        adcsample = adc_sample(param->adc_line, param->adc_res) >> param->adc_shift;
        xtimer_usleep(param->udelay);
    }

    if(*(param->stop_flag) != 0){
         (*(param->num_threads))--;
        printf("Thread stopped\n");
        return NULL;
    }

    int num_iter = (int)(99000/param->udelay);
    while(*(param->stop_flag) == 0){
        //xtimer_usleep(param->udelay);

        ping_rcvd=1;
        pinged = 0;
        for(i=0; i<(num_iter); i++){
            adcsample = adc_sample(param->adc_line, param->adc_res) >> param->adc_shift;
            
            if(*(param->stop_flag) != 0){
                (*(param->num_threads))--;
                printf("Thread stopped\n");
                return NULL;
            }

            // if(adcsample > med && !pinged){

            if(adcsample >= low && !pinged){ // changed adcsample >= med to low -Richard
                int increment = 0;
                int prev_sample = adcsample;
                xtimer_usleep(param->udelay);
                adcsample = adc_sample(param->adc_line, param->adc_res) >> param->adc_shift;
                while(prev_sample < adcsample){
                    prev_sample = adcsample;
                    xtimer_usleep(param->udelay);
                    adcsample = adc_sample(param->adc_line, param->adc_res) >> param->adc_shift;
                    increment++;
                }
                i=0;
                printf("Ping Recieved: %d\n",prev_sample);
                ping_rcvd=0;
                pinged = 1;
                avg+=prev_sample;
                num_ping_rcvd++;
                j++;
            }
            else if(adcsample < low){
                pinged = 0;
            }
            //printf("%d: %d\n",i,adcsample);
            if(sample_size>0){
                if(j>=sample_size){
                    avg/=sample_size;
                    double percent_loss = 100-((100*num_ping_rcvd)/sample_size);
                    printf("Sample size: %d; Pings recieved: %d\n", sample_size, num_ping_rcvd);
                    printf("Percent loss: %d%%\n", (int)percent_loss);
                    printf("Average strength: %d\n", (int)avg);
                    (*(param->num_threads))--;
                    printf("Thread stopped\n");
                    return NULL;
                }
            }
            xtimer_usleep(param->udelay);
        }
        if(ping_rcvd){
            printf("Ping missed\n");   
            j++;
        }
        
    }
    (*(param->num_threads))--;
    printf("Thread stopped\n");
    return NULL;
}

int scan_rx_start(int argc, char **argv)
{

    sample_size = -1;

    if(NUM_SCAN_RX_THREADS != 0){
        printf("There already exists a scan_rx thread\n");
        return 1;
    }

    if (argc < 2) {
        printf("usage: %s <udelay> -s <sample_size>\n", argv[0]);
        return 1;
    }

    if (atoi(argv[1]) <= 0)
    {
        puts("error: please input value greater than 0\n");
        return 1;
    }

    if(strcmp(argv[2],"-s") == 0){
        sample_size = atoi(argv[3]);
        if(atoi(argv[3])<0){
            puts("error: sample size must be greater than 0");
        }
    }

    QUIT_RX_SCAN_FLAG = 0;
    udelay = atoi(argv[1]);
    _tx_node_id = TX_NODE_ID;
    adc_line = AD4_PIN;
    adc_init(adc_line);
    adc_res = ADC_RES_7BIT; 

    switch(adc_res)
    {
        case ADC_RES_7BIT:
            socadc_rshift = SOCADC_7_BIT_RSHIFT;
            break;
        case ADC_RES_9BIT:
            socadc_rshift = SOCADC_9_BIT_RSHIFT;
            break;
        case ADC_RES_10BIT:
            socadc_rshift = SOCADC_10_BIT_RSHIFT;
            break;
        case ADC_RES_12BIT:
            socadc_rshift = SOCADC_12_BIT_RSHIFT;
            break;
        default:
            return 1;
    }

    param.num_threads = &NUM_SCAN_RX_THREADS;
    param.stop_flag = &QUIT_RX_SCAN_FLAG;
    param.udelay = udelay;
    param.adc_line = adc_line;
    param.adc_res = adc_res;
    param.adc_shift = socadc_rshift;
    param.sample_size = sample_size;

    puts("Trying to start thread\n");
    thread_create(scan_rx_stack, sizeof(scan_rx_stack),
                         THREAD_PRIORITY_MAIN - 1,
                         THREAD_CREATE_STACKTEST,
                         scan_rx_thread, &param,
                         "thread");

    puts("Thread should be started\n");
    return 0;
}

int scan_rx_stop(int argc, char **argv)
{
    if(NUM_SCAN_RX_THREADS == 0){
        printf("No scan threads currently running\n");
        return 1;
    }
    printf("Trying to stop thread\n");
    QUIT_RX_SCAN_FLAG = 1;
    return 0;
}
