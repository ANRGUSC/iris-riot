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
 * @brief       Example for using hdlc.
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

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifndef UART_STDIO_DEV
#define UART_STDIO_DEV          (UART_UNDEF)
#endif

#define HDLC_PRIO               (THREAD_PRIORITY_MAIN - 1)

static msg_t _hdlc_dispatcher_msg_queue[16];
static kernel_pid_t hdlc_pid;

static char hdlc_stack[THREAD_STACKSIZE_MAIN];

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming packets */
    msg_init_queue(_hdlc_dispatcher_msg_queue, 16);

    hdlc_pid = hdlc_init(hdlc_stack, sizeof(hdlc_stack), HDLC_PRIO, "hdlc", UART_DEV(1));

    puts("UART INFO:");
    printf("Available devices:               %i\n", UART_NUMOF);
    printf("UART used for STDIO (the shell): UART_DEV(%i)\n\n", UART_STDIO_DEV);

    msg_t msg_resp, msg_req;
    char frame_no = 0;
    char send_data[HDLC_MAX_PKT_SIZE];
    char recv_data[HDLC_MAX_PKT_SIZE];
    hdlc_pkt_t pkt = { .data = send_data, .length = 0 };
    hdlc_buf_t *buf;

    random_init(xtimer_now());

    msg_req.type = HDLC_MSG_REG_DISPATCHER;
    msg_req.content.value = (uint32_t) NULL;
    msg_send(&msg_req, hdlc_pid);

    DEBUG("dispatcher pid is %" PRIkernel_pid "\n", thread_getpid());

    msg_req.type = 0;
    int exit = 0;

    while(1)
    {
        pkt.data[0] = frame_no;

        for(int i = 1; i < HDLC_MAX_PKT_SIZE; i++) {
            pkt.data[i] = (char) (random_uint32() % 0x7E);
        }

        pkt.length = HDLC_MAX_PKT_SIZE;

        DEBUG("dispatcher: sending pkt no %d\n", frame_no);

        /* send pkt =*/
        msg_req.type = HDLC_MSG_SND;
        msg_req.content.ptr = &pkt;
        msg_send(&msg_req, hdlc_pid);

        while(1)
        {
            msg_receive(&msg_resp);

            switch (msg_resp.type)
            {
                case HDLC_RESP_SND_SUCC:
                    DEBUG("dispatcher: sent frame_no %d!\n", frame_no);
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_resp.content.value);
                    msg_send(&msg_req, hdlc_pid);
                    break;
                case HDLC_PKT_RDY:
                    buf = (hdlc_buf_t *)msg_resp.content.ptr;
                    memcpy(recv_data, buf->data, buf->length);
                    DEBUG("dispatcher: received pkt %d\n", recv_data[0]);
                    hdlc_pkt_release(buf);
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
