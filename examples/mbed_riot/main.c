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
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/netdev.h"
#include "periph/uart.h"
#include "main-conf.h"
#include "range_param.h"
#include "hdlc.h"
#include "uart_pkt.h"
#include "range.h"
#include "shell.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN)

#define MAIN_THR_PORT       1234
#define RANGE_PORT          5678

#define PKT_FROM_MAIN_THR   0
#define RANGE_PKT_DONE      10 //TODO: get rid of this.

#define RANGE_TIMEO_USEC    250000
#define MAIN_QUEUE_SIZE     (8)

#undef BIT
#define BIT(n) ( 1 << (n) )
/* Bit field definitions for the UART Line Control Register: */
#define FEN   BIT( 4) /**< Enable FIFOs */
#define UART_CTL_HSE_VALUE    0

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

/**
 * Checks if params_ranging_mode is valid.
 * @param ranging_mode_char From params->ranging_mode.
 */
// bool isInvalidRangingMode(char ranging_mode_char)
// {
//     return ranging_mode_char != ONE_SENSOR_MODE && 
//             ranging_mode_char != TWO_SENSOR_MODE && 
//             ranging_mode_char != XOR_SENSOR_MODE;
// }                          

/**
 * Main ranging code.
 * @param *arg Pointer to the HDLC pid.
 */
static void *_range_thread(void *arg)
{   
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;
    uint16_t old_channel;
    range_params_t* params;
    msg_init_queue(range_msg_queue, sizeof(range_msg_queue));
    hdlc_entry_t range_entry;
    range_entry.next = NULL;
    range_entry.port = (int16_t)RANGE_PORT;
    range_entry.pid = thread_getpid();
    hdlc_register(&range_entry);
    int pkt_size = sizeof(range_data_t)*8 + sizeof(uart_pkt_hdr_t);
    int i = 0;
    int num_iter = 0;
    int remainder = 0;
    int end_of_series = 0;

    msg_t msg_snd, msg_rcv;
    /* create packets with max size */
    char send_data[pkt_size];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = pkt_size};
    hdlc_pkt_t *hdlc_rcv_pkt, *hdlc_rcv_pkt2;
    uart_pkt_hdr_t uart_hdr;
    range_data_t* time_diffs;


    int exit = 0;

    while(1)
    {
        hdlc_snd_pkt.length = pkt_size;
        uart_hdr.src_port = RANGE_PORT;
        uart_hdr.dst_port = RANGE_PORT;
        uart_hdr.pkt_type = SOUND_RANGE_DONE;
        uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

        DEBUG("Range pid is %" PRIkernel_pid "\n", thread_getpid());
        DEBUG("PORT is %lu\n",(uint32_t)RANGE_PORT);

        DEBUG("Waiting for message...\n");
        msg_receive(&msg_rcv);

        DEBUG("Message recieved\n");
        switch (msg_rcv.type)
        {
            case HDLC_PKT_RDY:
                hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                uart_pkt_parse_hdr(&uart_hdr, hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);
                switch (uart_hdr.pkt_type){
                    case SOUND_RANGE_REQ:
                        old_channel = _get_channel();
                        _set_channel(RSSI_LOCALIZATION_CHAN);

                        params = (range_params_t *)uart_pkt_get_data(hdlc_rcv_pkt->data, hdlc_rcv_pkt->length);

                        if( params->ranging_mode != ONE_SENSOR_MODE && 
                            params->ranging_mode != TWO_SENSOR_MODE && 
                            params->ranging_mode != XOR_SENSOR_MODE ){
                            DEBUG("Recieved an invalid ranging mode\n");
                            break;
                        }
                        /*if( isInvalidRangingMode(params->ranging_mode)
                        {
                            DEBUG("Recieved an invalid ranging mode\n");
                            break;
                        }
                        */
                        else
                        {
                            num_iter = params->num_samples / 8;
                            remainder = params->num_samples % 8;
                            DEBUG("num_iter = %d\n", num_iter);
                            DEBUG("remainder = %d\n", remainder);

                            if(remainder == 0){
                                num_iter++;
                            }
                            if(num_iter == 0){
                                num_iter++;
                            }

                            if(params->num_samples > 8 && remainder != 0) 
                                i = -1;
                            else                                          
                                i = 0;

                            for(i; i < num_iter+1; i++){
                                DEBUG("i= %d\n", i);
                                if(!end_of_series){
                                    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 0;

                                    if(i != num_iter-1){
                                        time_diffs = range_rx((uint32_t) RANGE_TIMEO_USEC, params->ranging_mode, 8);
                                        DEBUG("sampling 8\n");
                                    } 
                                    else{
                                        end_of_series = 1;

                                        if(remainder != 0){   
                                            time_diffs = range_rx((uint32_t) RANGE_TIMEO_USEC, params->ranging_mode, remainder);
                                            DEBUG("sampling %d\n",remainder);
                                        } 
                                        else{
                                            UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
                                            UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
                                            UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
                                            UART1->cc2538_uart_dr.ECR = 0xFF;
                                            UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
                                            UART1->cc2538_uart_lcrh.LCRH |= FEN;
                                            UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;
                                            DEBUG("SKIPPING\n");
                                            continue;
                                        }
                                    }

                                    UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
                                    UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
                                    UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
                                    UART1->cc2538_uart_dr.ECR = 0xFF;
                                    UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
                                    UART1->cc2538_uart_lcrh.LCRH |= FEN;
                                    UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;

                                    if(time_diffs == NULL){
                                        DEBUG("An error occured while ranging\n");
                                        break;
                                    }
                                    
                                    //sending the data back down the hdlc

                                    DEBUG("Sending data back down hdlc\n");
                                    uart_pkt_cpy_data(hdlc_snd_pkt.data, hdlc_snd_pkt.length, time_diffs, sizeof(range_data_t)*8);
                                } else{
                                    DEBUG("End of series reached\n");
                                    end_of_series = 0;
                                    hdlc_snd_pkt.length = sizeof(uart_pkt_hdr_t)+sizeof(uint8_t);
                                    uart_hdr.src_port = RANGE_PORT;
                                    uart_hdr.dst_port = RANGE_PORT;
                                    uart_hdr.pkt_type = RANGE_PKT_DONE;
                                    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);
                                    uart_pkt_cpy_data(hdlc_snd_pkt.data, hdlc_snd_pkt.length, 0, sizeof(uint8_t));
                                }

                                msg_snd.type = HDLC_MSG_SND;
                                msg_snd.content.ptr = &hdlc_snd_pkt;
                                if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                    /* TODO: use xtimer_msg_receive_timeout() instead */
                                    /* this is where applications can decide on a timeout */
                                    DEBUG("HDLC busy retrying...\n");
                                    msg_rcv.type = HDLC_RESP_RETRY_W_TIMEO;
                                    msg_rcv.content.value = RTRY_TIMEO_USEC;
                                    msg_send_to_self(&msg_rcv);
                                }

                                while(1)
                                {
                                    msg_receive(&msg_rcv);

                                    switch (msg_rcv.type)
                                    {
                                        case HDLC_RESP_SND_SUCC:
                                            DEBUG("Successfully sent pkt\n");
                                            exit = 1;
                                            break;
                                        case HDLC_RESP_RETRY_W_TIMEO:
                                            xtimer_usleep(msg_rcv.content.value);
                                            DEBUG("Range thread: retrying\n");
                                            if(!msg_try_send(&msg_snd, hdlc_pid)) {
                                                DEBUG("Range thread: HDLC msg queue full!\n");
                                                msg_send_to_self(&msg_rcv);
                                            }
                                            break;
                                        case HDLC_PKT_RDY:
                                            hdlc_rcv_pkt2 = (hdlc_pkt_t *) msg_rcv.content.ptr;
                                            DEBUG("Range thread: received pkt while trying to send\n");
                                            hdlc_pkt_release(hdlc_rcv_pkt2);
                                            break;
                                        default:
                                            /* error */
                                            LED3_ON;
                                            break;
                                    }

                                    if(exit) {
                                        exit = 0;
                                        break;
                                    }
                                }
                                free(time_diffs);
                            }
                        }
                        _set_channel(old_channel);
                        
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
                                      "hdlc", UART_DEV(1));

    /* comment the lines below to test a single thread */
    thread_create(range_stack, sizeof(range_stack), THREAD2_PRIO, 
                  THREAD_CREATE_STACKTEST, _range_thread, hdlc_pid, "range thread");

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
