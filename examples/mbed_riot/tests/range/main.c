/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
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
 * @brief       Ultrasound ranging test using packets passed over hdlc
 * 
 * In this test, the openmote is set up to listen for a range request from the 
 * mbed The packets sent will contain information on the mode to range with. 
 * Available options are ONE_SENSOR_MODE, TWO_SENSOR_MODE, and XOR_SENSOR_MODE. 
 * The loop will alternate through all three options, taking a specified sample
 * number, SAMPS_PER_MODE, at a delay of LOOP_DELAY. The fastest this system 
 * can range at is 100 ms and this is due to the hardware limitations of the 
 * ultrasound sensors.
 * 
 * @author      Yutong Gu <yutonggu@usc.edu>
 *
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "random.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/netdev.h"
#include "periph/uart.h"
#include "main-conf.h"
#include "range_param.h"
#include "net/hdlc.h"
#include "net/uart_pkt.h"
#include "range.h"
#include "dac.h"
#include "shell.h"

#define ENABLE_DEBUG (1)
#define TX_MODE      (0) //TOGGLE THIS TO SET UP OPENMOTE AS RECEIVER OR TRANSMITTER

#include "debug.h"

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)

#define MAIN_THR_PORT       1234
#define RANGE_PORT          5678


/* see openmote-cc2538's periph_conf.h for second UART pin config */

static msg_t range_msg_queue[16];

static char hdlc_stack[THREAD_STACKSIZE_MAIN + 512];
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
        return gnrc_netapi_get(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    }

    return -1; /* fail */
}

static void *_range_tx_thread(void *arg){
    uint16_t old_channel;
    while(1){
        old_channel = _get_channel();
        _set_channel(RSSI_LOCALIZATION_CHAN);
        
        range_tx();

        _set_channel(old_channel);
        xtimer_usleep(TRANSMIT_DELAY);
    }
    return 0;
}

static void *_range_rx_thread(void *arg)
{   
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;
    uint16_t old_channel;
    range_params_t* range_params;
    msg_init_queue(range_msg_queue, sizeof(range_msg_queue));
    hdlc_entry_t range_entry;
    range_entry.next = NULL;
    range_entry.port = (int16_t)RANGE_PORT;
    range_entry.pid = thread_getpid();
    hdlc_register(&range_entry);
    
    msg_t msg_rcv;
    /* create packets with max size */
    
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr_rx;

    DEBUG("starting DAC\n");
    if(init_dac(DEFAULT_DAC_CS, SPI_CLK_400KHZ) == SPI_OK){
        DEBUG("Setting voltage at %d%%\n", DEFAULT_SENSOR_THRESH*100/255);
        set_voltage((uint8_t) DEFAULT_SENSOR_THRESH, DAC_GAIN_1);
        stop_dac();

    } else{
        DEBUG("SPI failed\n");
    }

    while(1)
    {

        DEBUG("Range pid is %" PRIkernel_pid "\n", thread_getpid());
        DEBUG("PORT is %lu\n",(uint32_t)RANGE_PORT);

        DEBUG("Waiting for message...\n");
        msg_receive(&msg_rcv);

        DEBUG("Message recieved\n");
        switch (msg_rcv.type)
        {
            case HDLC_PKT_RDY:
                
                
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                uart_pkt_parse_hdr(&uart_hdr_rx, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                switch (uart_hdr_rx.pkt_type){
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
                            _set_channel(RSSI_LOCALIZATION_CHAN);
                            range_and_send(range_params, hdlc_pid, RANGE_PORT, RANGE_PORT);
                            _set_channel(old_channel);
                        }
                        
                        break;
                    default:
                        DEBUG("Recieved a msg type other than SOUND_RANGE_REQ\n");
                        break;
                }
                hdlc_pkt_release(hdlc_rcv_pkt);
                break;
            default:
                /* error */
                DEBUG("Recieved something else");
                LED3_ON;
                break;

        }
        

        /* control transmission rate via interpacket intervals */
        xtimer_usleep(50000);
    }

    /* should be never reached */
    return 0;
}

int main(void)
{
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    /* auto-run */
    // char *temp[3];
    // temp[0] = "range_rx";
    // temp[1] = "50";          //num pkts
    // temp[2] = "1000000";     //interval_in_us
    // range_rx(3, temp);

    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(0));

    if(TX_MODE){
        DEBUG("Starting transmitter thread\n");
        thread_create(range_stack, sizeof(range_stack), THREAD2_PRIO, 
                  THREAD_CREATE_STACKTEST, _range_tx_thread, hdlc_pid, "range_tx thread");
    } else {
        DEBUG("Starting reciever thread\n");
        thread_create(range_stack, sizeof(range_stack), THREAD2_PRIO, 
                  THREAD_CREATE_STACKTEST, _range_rx_thread, hdlc_pid, "range_rx thread");
    }


    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
