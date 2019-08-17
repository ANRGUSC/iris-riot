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
 * @ingroup     examples
 * @{
 *
 * @file        iris_client/main.c
 * @brief       Localization Module of Client Node in IRIS (Intelligent Robotics IoT System) Testbed v2
 *              This should communicate with RPi
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
 * @author      Yukuan Jia <jyk19980526@163.com>
 *
 * @}
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

#define QUEUE_SIZE          8

#define RANGE_TIMEOUT_USEC                2000000 // 2s

#define TDMA_ANCHOR_ID_REQ_U16_FLAG         0x5444  /* 'T' and 'D' for TDma */
#define TDMA_ANCHOR_ID_RESP_U16_FLAG        0x4d41  /* 'M' and 'A' for tdMA */

#define TDMA_BOOTSTRAP_CHANNEL              11
#define TDMA_LOCALIZATION_CHAN              26

#define SOUND_MAX_ITER                30000

#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)

#define ENABLE_DEBUG (0)
#include "debug.h"

#define RX_OMNI_PIN                   GPIO_PIN(1, 1) // GPIO_PB1

static range_params_t range_param;
static range_data_t time_diffs[1];

static gpio_rx_line_t gpio_lines = (gpio_rx_line_t){RX_OMNI_PIN};

static msg_t msg; 
static msg_t msg_queue[QUEUE_SIZE];

int get_anchor_command(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    {"get-anchor", "get the TDoA data from the specified anchor", get_anchor_command},
    {NULL, NULL, NULL}
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

    // Iterate to get the only network interface
    // we can get netif thread pid via 'netif->pid' later
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

    // Iterate to get the only network interface
    // we can get netif thread pid via 'netif->pid' later
    gnrc_netif_t *netif = NULL;
    netif = gnrc_netif_iter(netif);

    gnrc_netapi_get(netif->pid, NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));

    return channel;
}

range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, int8_t node_id, uint32_t max_iter, int8_t anchor_id)
{ 
    uint8_t mode = range_mode;
    int exit = 0;
    int count = 0; // Count of how many signals have been received from wrong anchors

    while(exit == 0 && count <= 10){ 
        range_rx_init(node_id, thread_getpid(), gpio_lines, mode, max_iter);
        if(xtimer_msg_receive_timeout(&msg,timeout_usec)<0){
            DEBUG("Range: Timeout!\n");
            printf("Timeout!\n"); // printf for communication with RPi
            exit = 1;
            range_rx_stop();
            time_diffs[0] = (range_data_t) {0, RF_MISSED, node_id};
            return time_diffs;
        }
        switch(msg.type){
            case ULTRSND_RCVD: // Ultrasound signal received
                time_diffs[0] = *(range_data_t*) msg.content.ptr;
                count++;
                if(time_diffs[0].node_id == anchor_id)
                {
                    exit = 1;
                    DEBUG("Range: AnchorID = %d, TDoA = %d\n", time_diffs[0].node_id, time_diffs[0].tdoa);
                    printf("%d\n", time_diffs[0].tdoa); // printf for communication with RPi
                }
                break;
            default:
                break;
        }
    }
    if (count >= 10) // If received too many signals from wrong anchors, abort!
        printf("Timeout!\n");

    return time_diffs;
}

int get_anchor_command(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: get-anchor [anchor-id]\n");
        return 1;
    }

    range_rx((uint32_t) RANGE_TIMEOUT_USEC, range_param.ranging_mode, range_param.node_id, SOUND_MAX_ITER, atoi(argv[1]));
    return 0;
}

int main(void)
{
    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

    /* set RF channel */
    _set_channel(TDMA_LOCALIZATION_CHAN);
    
    /* Init GPIO Pin */
    gpio_init(RX_OMNI_PIN, GPIO_IN);

    range_param.ranging_mode = OMNI_SENSOR_MODE;
    range_param.node_id = -1; // RX node

    /* start shell */
    DEBUG("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
}
