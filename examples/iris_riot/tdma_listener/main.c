/**
 * Copyright (c) 2017-2019, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
 * Richard Kim
 * Yukuan Jia
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
 * @file        tdma_listener.c
 * @brief       Localization Module of Client Node in IRIS (Intelligent Robotics IoT System) Testbed v2
 *              For debugging use, check the debug outputs
 * 
 * @author      Yutong Gu <yutonggu@usc.edu>
 * @author      Yukuan Jia <jyk19980526@163.com>
 *
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/pktdump.h"
#include "net/netdev.h"
#include "periph/gpio.h"
#include "timex.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "board.h"
#include "thread.h"
#include "msg.h"
#include "shell.h"

#include "range_param.h"

#define SOUND_MAX_ITER                30000
#define RANGE_TIMEOUT_USEC            (TDMA_SLOT_TIME_USEC * 1.2)

#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)

#define ENABLE_DEBUG (1)
#include "debug.h"

#define RX_OMNI_PIN                   GPIO_PIN(1, 1) // GPIO_PB1

static range_data_t time_diffs[1];
static gpio_rx_line_t gpio_lines = (gpio_rx_line_t){RX_OMNI_PIN};

static char range_stack[THREAD_STACKSIZE_MAIN];

static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

/**
 * Set channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static int _set_channel(uint16_t channel)
{
    if (channel < 11 || channel > 26) {
        return -1; /* invalid channel number */
    }

    gnrc_netif_t *netif = NULL;
    netif = gnrc_netif_iter(netif);

    gnrc_netapi_set(netif->pid, NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));

    return 0;
}

/**
 * Get channel of radio. Works only if one netif exists.
 * @param channel Target channel with a valid range of 11 to 26.
 */
static uint16_t _get_channel(void)
{
    uint16_t channel;

    gnrc_netif_t *netif = NULL;
    netif = gnrc_netif_iter(netif);

    gnrc_netapi_get(netif->pid, NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    return channel;
}

range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, int8_t node_id, uint32_t max_iter){ 
    // Check correct argument usage.
    uint8_t mode = range_mode;
    int exit = 0;

    if(gpio_init(RX_OMNI_PIN, GPIO_IN) < 0) {
        DEBUG("Error initializing RX_OMNI_PIN.\n");
        return NULL;
    }

    /* setup the message queue */
    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];
    msg_init_queue(msg_queue, QUEUE_SIZE);

    while(exit == 0){
        range_rx_init(node_id, thread_getpid(), gpio_lines, mode, max_iter);
        if(xtimer_msg_receive_timeout(&msg,timeout_usec)<0){
            DEBUG("Range: Timeout!\n");
            range_rx_stop();
            time_diffs[0] = (range_data_t) {0, RF_MISSED, node_id};
            return time_diffs;
        }
        switch(msg.type){
            case ULTRSND_RCVD: // Ultrasound signal rseceived
                time_diffs[0] = *(range_data_t*) msg.content.ptr;
                DEBUG("Range: Anchor Node ID: %d, TDoA = %d\n", time_diffs[0].node_id, time_diffs[0].tdoa);
                exit = 1;
                break;
            default:
                break;
        }
    }

    return time_diffs;
}

static void *_range_rx_thread(void *arg)
{   
    uint16_t old_channel;
    range_params_t range_param;
    
    msg_t msg_rcv;

    while(1){
        old_channel = _get_channel();
        _set_channel(RSSI_LOCALIZATION_CHAN);

        range_param.ranging_mode = OMNI_SENSOR_MODE;
        range_param.node_id = -1; // RX node

        range_rx((uint32_t) RANGE_TIMEOUT_USEC, range_param.ranging_mode, range_param.node_id, SOUND_MAX_ITER);

        _set_channel(old_channel);

        //timer_usleep(50000); // 50ms
    }

    /* should be never reached */
    return 0;
}

int main(void)
{
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    DEBUG("Starting reciever thread\n");
    thread_create(range_stack, sizeof(range_stack), THREAD2_PRIO, 
        THREAD_CREATE_STACKTEST, _range_rx_thread,NULL, "range_rx thread");
}