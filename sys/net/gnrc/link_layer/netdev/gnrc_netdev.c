/*
 * Copyright (C) 2015 Freie Universität Berlin
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net
 * @file
 * @brief       Glue for netdev devices to netapi
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @}
 */

#include <errno.h>

#include "msg.h"
#include "thread.h"

#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev.h"

#include "net/gnrc/netdev.h"
#include "net/ethernet/hdr.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif

#define NETDEV_NETAPI_MSG_QUEUE_SIZE 8

static void _pass_on_packet(gnrc_pktsnip_t *pkt);
static void _sound_ranging(void);

/* sound ranging */
#include "periph/adc.h"
#include "xtimer.h"
#define MAX_SAMPLES 2000 /* for timeout of ranging */
static int sample1;
static int sample2;
static uint8_t _tx_node_id      = 0;
static gpio_rx_line_t rx_line;
static int max_samps            = 0;
static int range_sys_flag       = 0;
int ranging_on           = 0;
msg_t ranging_complete;
static int ranging_pid;

static uint32_t last            = 0;
static uint32_t last2           = 0;
static range_data_t time_diffs;
int ranging                     = 0;

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) dev->context;
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch(event) {
            case NETDEV_EVENT_RX_COMPLETE:
                {
                    gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                    /* first, check if it's a ranging packet */
                    if(ranging_on && !ranging)
                    {
                        if(RANGE_FLAG_BYTE0 == ((uint8_t *) pkt->data)[0] && 
                            RANGE_FLAG_BYTE1 == ((uint8_t *) pkt->data)[1] &&
                            _tx_node_id == ((uint8_t *) pkt->data)[2])
                        {
                            // printf("ranging");
                            ranging_complete.type=RF_RCVD;
                            ranging_complete.content.value=0;
                            msg_send(&ranging_complete,ranging_pid);
                            _sound_ranging();
                        }
                    }

                    if (pkt) {
                        _pass_on_packet(pkt);
                    }

                    break;
                }
#ifdef MODULE_NETSTATS_L2
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default:
                DEBUG("gnrc_netdev: warning: unhandled event %u.\n", event);
        }
    }
}

static void _sound_ranging(void)
{
    //unsigned old_state = irq_disable();

    int cnt = 0;
    int first = 2;
    int second = 2;
    uint32_t test;
    ranging = 1;
    last = xtimer_now_usec();
    time_diffs.tdoa = 0;
    time_diffs.orient_diff = 0;
    time_diffs.status = 0;
    unsigned int rx_line_array[] = {rx_line.one_pin, rx_line.two_pin, rx_line.xor_pin};

    
    //xtimer_spin(XTIMER_USEC_TO_TICKS(5));
    while(cnt < max_samps)
    {
        test = xtimer_now_usec();
        if(test < last){
            printf("failed on iteration: %d\n",cnt);
        }
        if(range_sys_flag == ONE_SENSOR_MODE){
            sample1 = gpio_read(rx_line_array[0]);
            DEBUG("%d ",sample1);
            if(sample1 != 0){
                last2 = xtimer_now_usec();
                DEBUG("%lu - %lu ",last, last2);
                time_diffs.tdoa = last2 - last;
                range_rx_successful_stop();
                break;
            }
        }
        else if(range_sys_flag == TWO_SENSOR_MODE){
            //printf("%d\n",cnt);
            sample1 = gpio_read(rx_line_array[0]);
            sample2 = gpio_read(rx_line_array[1]);
            DEBUG("%d ",sample1);
            DEBUG("%d ",sample2);
            /* wait for 200us before next poll for input capacitor to settle */
            if(sample1 != 0){ first = 0; second = 1; }
            else if(sample2 != 0){ first = 1; second = 0; }

            //xtimer_spin(XTIMER_USEC_TO_TICKS(1));
            if (first != 2) {
                last2 = xtimer_now_usec();
                do {
                    sample1 = gpio_read(rx_line_array[first]);
                    sample2 = gpio_read(rx_line_array[second]);
                } while((sample1 != 0) && (sample2 == 0));
                
                time_diffs.orient_diff = xtimer_now_usec() - last2;
                time_diffs.tdoa = last2 - last;

                time_diffs.status = first + 1;
                
                if(sample1 == 0){
                    time_diffs.status += 10; //returns the pin that first recieve if both recieved, otherwise add 10 to the flag
                }
                
                range_rx_successful_stop();
                break;
            } 
        }
        else if(range_sys_flag == XOR_SENSOR_MODE){
            sample1 = gpio_read(rx_line_array[2]);
            DEBUG("%d ",sample1);
            if(sample1 != 0){
                last2 = xtimer_now_usec();
                while(gpio_read(rx_line_array[2]) != 0);
                time_diffs.orient_diff = xtimer_now_usec() - last2;
                time_diffs.tdoa = last2 - last;
                range_rx_successful_stop();
                break;
            }
        }

        ++cnt;
    }

    last = xtimer_now_usec();

    if(cnt >= max_samps){
        DEBUG("cnt>max_samps\n");
    }
    //irq_restore(old_state);
}


static void _pass_on_packet(gnrc_pktsnip_t *pkt)
{
    /* throw away packet if no one is interested */
    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
        DEBUG("gnrc_netdev: unable to forward packet of type %i\n", pkt->type);
        gnrc_pktbuf_release(pkt);
        return;
    }
}

/**
 * @brief   Startup code and event loop of the gnrc_netdev layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_netdev_thread(void *args)
{
    DEBUG("gnrc_netdev: starting thread\n");

    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) args;
    netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV_NETAPI_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV_NETAPI_MSG_QUEUE_SIZE);

    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void*) gnrc_netdev;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);

    /* start the event loop */
    while (1) {
        DEBUG("gnrc_netdev: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SND received\n");
                gnrc_pktsnip_t *pkt = msg.content.ptr;
                gnrc_netdev->send(gnrc_netdev, pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            default:
                DEBUG("gnrc_netdev: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_netdev_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev_t *gnrc_netdev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev == NULL || gnrc_netdev->dev == NULL) {
        return -ENODEV;
    }

    /* create new gnrc_netdev thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                        _gnrc_netdev_thread, (void *)gnrc_netdev, name);
    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}

/* Successful ranging will immediately turn off ranging mode. */
void range_rx_init(char tx_node_id, int pid, gpio_rx_line_t lines, unsigned int max_gpio_samps, int mode)
{
    //puts("started");
    range_sys_flag = mode;
    ranging_on = 1;
    _tx_node_id = tx_node_id;
    ranging_pid = pid;
    rx_line = lines;
    gpio_init(rx_line.one_pin,GPIO_IN);
    gpio_init(rx_line.two_pin,GPIO_IN);
    gpio_init(rx_line.xor_pin,GPIO_IN);
    max_samps = max_gpio_samps;

    time_diffs.tdoa = 0;
    time_diffs.orient_diff = 0;
    time_diffs.status = 0;

    DEBUG("ranging initialized!\n");
}

void range_rx_successful_stop(void)
{
    //puts("stopped");
    range_rx_stop();
    ranging_complete.type=ULTRSND_RCVD;
    ranging_complete.content.ptr=&time_diffs;
    msg_send(&ranging_complete,ranging_pid);
}

void range_rx_stop(void)
{
    //puts("stopped");
    ranging = 0;
    ranging_on = 0;
}
