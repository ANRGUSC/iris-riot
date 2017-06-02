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
#include "dispatcher.h"
#include "uart_pkt.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)
#define THREAD2_PRIO            (THREAD_PRIORITY_MAIN - 1)

#define MAIN_THR_PORT   1234
#define THREAD2_PORT    5678

#define PKT_FROM_MAIN_THR   0
#define PKT_FROM_THREAD2    1

/* see openmote-cc2538's periph_conf.h for second UART pin config */

static msg_t thread2_msg_queue[8];
static msg_t main_msg_queue[8];

static char hdlc_stack[THREAD_STACKSIZE_MAIN];
static char dispatcher_stack[THREAD_STACKSIZE_MAIN];
static char thread2_stack[THREAD_STACKSIZE_MAIN];

static void *_thread2(void *arg)
{
    kernel_pid_t hdlc_pid = (kernel_pid_t)arg;
    msg_init_queue(thread2_msg_queue, 8);
    dispatcher_entry_t thread2 = { NULL, THREAD2_PORT, thread_getpid() };
    dispatcher_register(&thread2);

    msg_t msg_snd, msg_rcv;
    char frame_no = 0;
    /* create packets with max size */
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;

    /* hdr for each pkt is the same for this test */
    uart_hdr.src_port = THREAD2_PORT;
    uart_hdr.dst_port = THREAD2_PORT;
    uart_hdr.pkt_type = PKT_FROM_THREAD2;
    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

    DEBUG("thread2 pid is %" PRIkernel_pid "\n", thread_getpid());

    int exit = 0;

    while(1)
    {
        hdlc_snd_pkt.data[UART_PKT_DATA_FIELD] = frame_no;

        /* fill packet with chars that will not be escaped */
        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            hdlc_snd_pkt.data[i] = (char) (random_uint32() % 0x7E);
        }

        DEBUG("main_thr: sending pkt no %d\n", frame_no);

        msg_snd.type = HDLC_MSG_SND;
        msg_snd.content.ptr = &hdlc_snd_pkt;
        msg_send(&msg_snd, hdlc_pid);

        while(1)
        {
            msg_receive(&msg_rcv);

            switch (msg_rcv.type)
            {
                case HDLC_RESP_SND_SUCC:
                    DEBUG("thread2: sent frame_no %d!\n", frame_no);
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    msg_send(&msg_snd, hdlc_pid);
                    break;
                case HDLC_PKT_RDY:
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                    DEBUG("thread2: received pkt %d\n", hdlc_rcv_pkt->data[UART_PKT_DATA_FIELD]);
                    hdlc_pkt_release(hdlc_rcv_pkt);
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

        frame_no++;
    }

    /* should be never reached */
    return 0;
}

void main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming packets */
    msg_init_queue(main_msg_queue, 8);

    kernel_pid_t hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, 
                                      "hdlc", UART_DEV(1));
    dispacher_init(dispatcher_stack, sizeof(dispatcher_stack), HDLC_PRIO, 
                   "dispatcher", (void *) (uint32_t) hdlc_pid);
    thread_create(thread2_stack, sizeof(thread2_stack), THREAD2_PRIO, 
                  THREAD_CREATE_STACKTEST, _thread2, hdlc_pid, "thread2");

    dispatcher_entry_t main_thr = { NULL, MAIN_THR_PORT, thread_getpid() };

    dispatcher_register(&main_thr);

    msg_t msg_snd, msg_rcv;
    char frame_no = 0;
    /* create packets with max size */
    char send_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = HDLC_MAX_PKT_SIZE };
    hdlc_pkt_t *hdlc_rcv_pkt;
    uart_pkt_hdr_t uart_hdr;

    /* hdr for each pkt is the same for this test */
    uart_hdr.src_port = MAIN_THR_PORT;
    uart_hdr.dst_port = MAIN_THR_PORT;
    uart_hdr.pkt_type = PKT_FROM_MAIN_THR;
    uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr);

    random_init(xtimer_now().ticks32);

    DEBUG("hdlc_txvr_test pid is %" PRIkernel_pid "\n", thread_getpid());

    int exit = 0;

    while(1)
    {
        hdlc_snd_pkt.data[UART_PKT_DATA_FIELD] = frame_no;

        /* fill packet with chars that will not be escaped */
        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            hdlc_snd_pkt.data[i] = (char) (random_uint32() % 0x7E);
        }

        DEBUG("main_thr: sending pkt no %d\n", frame_no);

        msg_snd.type = HDLC_MSG_SND;
        msg_snd.content.ptr = &hdlc_snd_pkt;
        msg_send(&msg_snd, hdlc_pid);

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
                    msg_send(&msg_snd, hdlc_pid);
                    break;
                case HDLC_PKT_RDY:
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                    DEBUG("main_thr: received pkt %d\n", hdlc_rcv_pkt->data[UART_PKT_DATA_FIELD]);
                    hdlc_pkt_release(hdlc_rcv_pkt);
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

        frame_no++;
    }

    /* should be never reached */
    return 0;
}
