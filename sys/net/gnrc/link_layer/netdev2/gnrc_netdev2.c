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
 * @brief       Glue for netdev2 devices to netapi
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
#include "net/netdev2.h"

#include "net/gnrc/netdev2.h"
#include "net/ethernet/hdr.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif

#define NETDEV2_NETAPI_MSG_QUEUE_SIZE 8

static void _pass_on_packet(gnrc_pktsnip_t *pkt);
static void _sound_ranging(void);

/* sound ranging */
#include "periph/adc.h"
#include "xtimer.h"
#define MAX_SAMPLES 2000 /* for timeout of ranging */
static int ultrasound_thresh;
static int sample;
static uint8_t _tx_node_id      = 0;
static unsigned int adc_line;
static int socadc_rshift        = 0;
static int max_samps            = 0;
static int ranging_on           = 0;
static int adc_res              = 0; /* default resolution */
static uint32_t last            = 0;
static uint32_t time_diff       = 0;
static kernel_pid_t pid_of_request = -1;

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 */
static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) dev->context;

    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV2_MSG_TYPE_EVENT;
        msg.content.ptr = gnrc_netdev2;

        if (msg_send(&msg, gnrc_netdev2->pid) <= 0) {
            puts("gnrc_netdev2: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev2: event triggered -> %i\n", event);
        switch(event) {
            case NETDEV2_EVENT_RX_COMPLETE:
                {
                    gnrc_pktsnip_t *pkt = gnrc_netdev2->recv(gnrc_netdev2);

                    /* first, check if it's a ranging packet */
                    if(ranging_on)
                    {
                        if(RANGE_FLAG_BYTE0 == ((uint8_t *) pkt->data)[0] && 
                            RANGE_FLAG_BYTE1 == ((uint8_t *) pkt->data)[1] &&
                            _tx_node_id == ((uint8_t *) pkt->data)[2])
                        {
                            _sound_ranging();
                            ranging_on = 0;
                        }
                    }

                    if (pkt) {
                        _pass_on_packet(pkt);
                    }

                    break;
                }
#ifdef MODULE_NETSTATS_L2
            case NETDEV2_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV2_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default:
                DEBUG("gnrc_netdev2: warning: unhandled event %u.\n", event);
        }
    }
}

static void _sound_ranging(void)
{
    int cnt = 0;
    last = xtimer_now();
    msg_t msg;

    unsigned old_state = irq_disable();

    while(cnt < max_samps)
    {
        sample = adc_sample(adc_line, adc_res) 
            >> socadc_rshift;

        /* wait for 200us before next poll for input capacitor to settle */
        xtimer_spin(200);

        if (sample > ultrasound_thresh) {
            time_diff = xtimer_now() - last; 
            DEBUG("Ranging Successful - sample: %d, time_diff: %lu\n", sample, time_diff);
            range_rx_stop();
            break;
        }

        ++cnt;
    }

    irq_restore(old_state);

    if (pid_of_request != -1) {
        msg.type = RANGE_RX_COMPLETE;
        msg.content.value = (uint32_t) time_diff;
        time_diff = 0;
        msg_send(&msg, pid_of_request);
    }
}

static void _pass_on_packet(gnrc_pktsnip_t *pkt)
{
    /* throw away packet if no one is interested */
    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
        DEBUG("gnrc_netdev2: unable to forward packet of type %i\n", pkt->type);
        gnrc_pktbuf_release(pkt);
        return;
    }
}

/**
 * @brief   Startup code and event loop of the gnrc_netdev2 layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_netdev2_thread(void *args)
{
    DEBUG("gnrc_netdev2: starting thread\n");

    gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) args;
    netdev2_t *dev = gnrc_netdev2->dev;

    gnrc_netdev2->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV2_NETAPI_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV2_NETAPI_MSG_QUEUE_SIZE);

    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void*) gnrc_netdev2;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);

    /* start the event loop */
    while (1) {
        DEBUG("gnrc_netdev2: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV2_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev2: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_SND received\n");
                gnrc_pktsnip_t *pkt = msg.content.ptr;
                gnrc_netdev2->send(gnrc_netdev2, pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev2: response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = msg.content.ptr;
                DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev2: response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            default:
                DEBUG("gnrc_netdev2: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_netdev2_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev2_t *gnrc_netdev2)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev2 == NULL || gnrc_netdev2->dev == NULL) {
        return -ENODEV;
    }

    /* create new gnrc_netdev2 thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _gnrc_netdev2_thread, (void *)gnrc_netdev2, name);
    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}

void range_rx_init(char tx_node_id, int thresh, unsigned int line, 
                   unsigned int res, unsigned int max_adc_samps)
{
    ranging_on = 1;
    _tx_node_id = tx_node_id;
    ultrasound_thresh = thresh;
    adc_line = line;
    max_samps = max_adc_samps;
    adc_res = res; 
    time_diff = 0;
    pid_of_request = thread_getpid();

    switch(res)
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
            DEBUG("range_rx_init failed!");
            return;
    }

    DEBUG("ranging initialized!\n");
}

void range_rx_stop(void)
{
    ranging_on = 0;
    pid_of_request = -1;
}
