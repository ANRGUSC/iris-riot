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
 * THE SOFTWARE.g
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Full duplex hdlc implementation.
 *
 * This implementation leverages yahdlc, an open source library. The current 
 * implementation is stop & wait.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */


#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "net/hdlc.h"
#include "utlist.h"
#include "net/uart_pkt.h"

#include "msg.h"
#include "xtimer.h"
#include "ringbuffer.h"
#include "net/yahdlc.h"
#include "periph/uart.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef UART_BUFSIZE
    #define UART_BUFSIZE            (1024U)
#endif

static msg_t _hdlc_msg_queue[HDLC_MSG_QUEUE_SIZE];

static kernel_pid_t hdlc_dispatcher_pid, sender_pid, hdlc_thread_pid = KERNEL_PID_UNDEF;

typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx;

/* uart access control lock */
static uint32_t uart_lock = 0;

static char hdlc_recv_data[HDLC_MAX_PKT_SIZE];
static char hdlc_send_frame[2 * (HDLC_MAX_PKT_SIZE + 2 + 2 + 2)];
static char hdlc_ack_frame[2 + 2 + 2 + 2];
static char hdlc_pkt_data[HDLC_MAX_PKT_SIZE];

static hdlc_buf_t recv_buf = { .data = hdlc_recv_data };    //hdlc internal buffer
static hdlc_buf_t send_buf = { .data = hdlc_send_frame };
static hdlc_buf_t ack_buf  = { .data = hdlc_ack_frame };
static hdlc_pkt_t recv_pkt = { .data = hdlc_pkt_data };     //buffer used to pass on packet
static mutex_t recv_pkt_mutex;

static void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)arg;

    ringbuffer_add_one(&(ctx.rx_buf), data);

    if(data == YAHDLC_FLAG_SEQUENCE)
    {
        /* wakeup hdlc thread */
        msg_t msg;
        msg.type = HDLC_MSG_RECV;
        msg.content.value = (uint32_t)dev;
        msg_send(&msg, hdlc_thread_pid); 
    }
}

static hdlc_entry_t *hdlc_reg;
//adding the hdlc to a list(which list?)
void hdlc_register(hdlc_entry_t *entry)
{
    LL_PREPEND(hdlc_reg, entry);
}

void hdlc_unregister(hdlc_entry_t *entry)
{
    LL_DELETE(hdlc_reg, entry);
}

static void _hdlc_receive(unsigned int *recv_seq_no, unsigned int *send_seq_no)
{
    msg_t msg, ack_msg;
    int ret;
    int retval;
    char c;
    uart_pkt_hdr_t hdr;
    hdlc_entry_t *entry;


    while(1) {
        retval = ringbuffer_get_one(&(ctx.rx_buf));

        if (retval == -1) {
            break;
        }

        c = (char) retval;

        mutex_lock(&(recv_buf.mtx));

        ret = yahdlc_get_data(&recv_buf.control, &c, 1, recv_buf.data, &recv_buf.length);

        mutex_unlock(&(recv_buf.mtx));

        if (ret == -ENOMSG) {
            continue;
        }

        if (ret == -EIO) {
            DEBUG("hdlc: FCS ERROR OR INVALID FRAME!\n");
            recv_buf.control.frame = recv_buf.control.seq_no = 0;
            return;
        }

        if (recv_buf.length > 0 && 
            (recv_buf.control.seq_no == *recv_seq_no % 8 ||
            recv_buf.control.seq_no == (*recv_seq_no - 1) % 8)) {
            /* valid data frame received */
            DEBUG("hdlc: received data frame w/ seq_no: %d\n", recv_buf.control.seq_no);

            /* always send ack */
            ack_msg.type = HDLC_MSG_SND_ACK;
            ack_msg.content.value = recv_buf.control.seq_no;
            msg_send_to_self(&ack_msg); /* send ack */

            /* pass on packet to dispatcher */
            if (recv_buf.control.seq_no == *recv_seq_no % 8) {
                /* lock pkt until receiving thread makes a copy and unlocks */
                mutex_lock(&recv_pkt_mutex);
                memcpy(recv_pkt.data, recv_buf.data, recv_buf.length);
                recv_pkt.length = recv_buf.length;
                uart_pkt_parse_hdr(&hdr, recv_pkt.data, recv_pkt.length);
                LL_SEARCH_SCALAR(hdlc_reg, entry, port, hdr.dst_port);
                DEBUG("hdlc: received packet for port %d\n", hdr.dst_port);
                (*recv_seq_no)++;
                if(entry) {
                    msg.type = HDLC_PKT_RDY;
                    msg.content.ptr = &recv_pkt;
                    if(!msg_try_send(&msg, entry->pid)) {
                        DEBUG("hdlc: failed to send to the thread!\n");
                        hdlc_pkt_release(&recv_pkt);
                    }
                } else {
                    DEBUG("hdlc: no thread subscribed to port!\n");
                    hdlc_pkt_release(&recv_pkt);
                }
            }

            recv_buf.control.frame = recv_buf.control.seq_no =  0;
            return;
        } else if (recv_buf.length == 0 && 
                    (recv_buf.control.frame == YAHDLC_FRAME_ACK ||
                     recv_buf.control.frame == YAHDLC_FRAME_NACK)) {
            DEBUG("hdlc: received ACK/NACK w/ seq_no: %d\n", recv_buf.control.seq_no);

            if(recv_buf.control.seq_no == *send_seq_no % 8) {
                uart_lock = 0;
                (*send_seq_no)++;
                msg.type = HDLC_RESP_SND_SUCC;
                msg.content.value = (uint32_t) 0;
                DEBUG("hdlc: valid ACK, tell pid %d a pkt is ready\n", sender_pid);
                if(!msg_try_send(&msg, sender_pid)) {
                    DEBUG("hdlc: failed to send to the thread!\n");
                }
            }
                                
            recv_buf.control.frame = recv_buf.control.seq_no = 0;
            return;
        } 
    }
}

static void *_hdlc(void *arg)
{
    uart_t dev = (uart_t)arg;
    msg_init_queue(_hdlc_msg_queue, HDLC_MSG_QUEUE_SIZE);
    uint32_t last_sent = 0;
    msg_t msg, reply, msg2;
    unsigned int recv_seq_no = 0;
    unsigned int send_seq_no = 0;

    while(1) {
        if(uart_lock) {
            int timeout = (int) (last_sent + RETRANSMIT_TIMEO_USEC) - (int) xtimer_now().ticks32;
            if(timeout < 0) {
                /* send message to self to resend msg */
                msg2.type = HDLC_MSG_RESEND;
                msg_send_to_self(&msg2);
                msg_receive(&msg);
            } else {
                if(0 > xtimer_msg_receive_timeout(&msg, timeout)) {
                    continue;
                }
            }
        } else {
            msg_receive(&msg);
        }

        switch (msg.type) {
            case HDLC_MSG_RECV:
                DEBUG("hdlc: receiving msg...\n");
                _hdlc_receive(&recv_seq_no, &send_seq_no);
                break;
            case HDLC_MSG_SND:
                DEBUG("hdlc: request to send received from pid %d\n", msg.sender_pid);
                if (uart_lock) {
                    /* ask thread to try again in x usec */
                    DEBUG("hdlc: uart locked, telling thr to retry\n");
                    reply.type = HDLC_RESP_RETRY_W_TIMEO;
                    reply.content.value = (uint32_t) RTRY_TIMEO_USEC;
                    msg_send(&reply, msg.sender_pid);
                } else {
                    uart_lock = 1;
                    sender_pid = msg.sender_pid;
                    DEBUG("hdlc: sender_pid set to %d, send_seq_no %d\n", 
                          sender_pid, send_seq_no % 8);
                    send_buf.control.frame = YAHDLC_FRAME_DATA;
                    send_buf.control.seq_no = send_seq_no % 8; 
                    hdlc_pkt_t *pkt = msg.content.ptr;
                    yahdlc_frame_data(&(send_buf.control), pkt->data, 
                            pkt->length, send_buf.data, &send_buf.length);
                    uart_write(dev, (uint8_t *)send_buf.data, send_buf.length);
                    last_sent = xtimer_now().ticks32;
                }
                break;
            case HDLC_MSG_SND_ACK:
                /* send ACK */
                ack_buf.control.frame = YAHDLC_FRAME_ACK;
                ack_buf.control.seq_no = msg.content.value;
                yahdlc_frame_data(&(ack_buf.control), NULL, 0, ack_buf.data, &(ack_buf.length));
                DEBUG("hdlc: sending ack w/ seq no %d\n", ack_buf.control.seq_no);
                uart_write(dev, (uint8_t *)ack_buf.data, ack_buf.length);
                break;
            case HDLC_MSG_RESEND:
                DEBUG("hdlc: Resending frame w/ seq no %d (on send_seq_no %d)\n", send_buf.control.seq_no, send_seq_no);
                uart_write(dev, (uint8_t *)send_buf.data, send_buf.length);
                last_sent = xtimer_now().ticks32;
                break;
            case HDLC_MSG_REG_DISPATCHER:
                DEBUG("hdlc: Registering dispatcher thread.\n");
                hdlc_dispatcher_pid = msg.sender_pid;
                DEBUG("hdlc: hdlc_dispatcher_pid set to %d\n", hdlc_dispatcher_pid);
                break;
            default:
                DEBUG("INVALID HDLC MSG\n");
                LED3_ON;
                break;
        }
    }

    /* this should never be reached */
    return NULL;
}

int hdlc_pkt_release(hdlc_pkt_t *pkt) 
{
    if(recv_pkt_mutex.queue.next != NULL) {
        DEBUG("unlocking recv_pkt\n");
        mutex_unlock(&recv_pkt_mutex);
        return 0;
    }

    DEBUG("Packet not locked. Might be empty!\n");
    return -1;
}

kernel_pid_t hdlc_init(char *stack, int stacksize, char priority, const char *name, uart_t dev)
{
    kernel_pid_t res;

    /* check if uart device number is valid */
    if(dev > UART_NUMOF - 1) {
        return -ENODEV;
    }

    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);
    uart_init(dev, 115200, rx_cb, (void *) dev);
    // DEBUG("mutex is (%d)\n", recv_pkt_mutex.queue.next);

    res = thread_create(stack, stacksize,
                        priority, THREAD_CREATE_STACKTEST, _hdlc, (void *) dev, name);

    if (res <= 0) {
        return -EINVAL;
    }

    hdlc_thread_pid = res;

    return res;
}

