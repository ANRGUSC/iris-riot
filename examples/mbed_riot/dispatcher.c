/**
 * Copyright (c) 2016, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran
 * Pradipta Ghosh
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
 * @file        dispatcher.c
 * @brief       dispatcher thread. handles all incoming packets (hdlc, radio, etc.).
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "hdlc.h"
#include "yahdlc.h"
#include "fcs16.h"
#include "dispatcher.h"
#include "uart_pkt.h"
#include "main-conf.h"
#include "utlist.h"
#include "net/gnrc.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/* TODO: merge dispatcher into hdlc? */

static msg_t _dispatcher_msg_queue[8];

static dispatcher_entry_t *dispatcher_reg;

void dispatcher_register(dispatcher_entry_t *entry)
{
    LL_PREPEND(dispatcher_reg, entry);
}

void dispatcher_unregister(dispatcher_entry_t *entry)
{
    LL_DELETE(dispatcher_reg, entry);
}

/**
 * @brief Dispatch or handle incoming hdlc messages. Get and set commands should
 *        be handled here. 
 *
 * @param[in] arg       PID of hdlc thread
 *
 */
static void *_dispatcher(void *arg)
{
    kernel_pid_t hdlc_thread_pid = (kernel_pid_t) (intptr_t) arg;

    msg_t msg;
    uart_pkt_hdr_t hdr;
    dispatcher_entry_t *entry;
    hdlc_pkt_t *recv_pkt;
    gnrc_pktsnip_t *recv_gnrc_pkt;
    msg_init_queue(_dispatcher_msg_queue, 8);

    msg.type = HDLC_MSG_REG_DISPATCHER;
    msg.content.value = (uint32_t) 0;
    msg_send(&msg, hdlc_thread_pid);

    /* subscribe this thread to pickup all UDP packets */
    /* uncomment when the dispatcher is ready to forward packets */
    // gnrc_netreg_entry_t dispatcher_server = { 
    //     NULL, 
    //     (uint32_t) GNRC_NETREG_DEMUX_CTX_ALL, 
    //     thread_getpid() 
    // };
    // gnrc_netreg_register(GNRC_NETTYPE_UDP, &dispatcher_server);

    while(1)
    {
        msg_receive(&msg);
        switch (msg.type)
        {
            case HDLC_PKT_RDY:
                recv_pkt = (hdlc_pkt_t *)msg.content.ptr;

                uart_pkt_parse_hdr(&hdr, recv_pkt->data, recv_pkt->length);
                LL_SEARCH_SCALAR(dispatcher_reg, entry, port, hdr.dst_port);
                /* fwd msg to thread */
                DEBUG("dispatcher: received a packet for port %d\n", hdr.dst_port);
                if(entry) {
                    msg_send(&msg, entry->pid);
                } else {
                    hdlc_pkt_release(recv_pkt);
                }
                break;
            case GNRC_NETAPI_MSG_TYPE_RCV:
                /* TODO: forward radio pkt to mbed or relevant thread */
                recv_gnrc_pkt = msg.content.ptr;
                gnrc_pktbuf_release(recv_gnrc_pkt);
            default:
                /* error */
                DEBUG("Dispatcher: invalid packet\n");
                break;
        }    
    }

    DEBUG("Error: Reached Exit!");
    /* should be never reached */
    return NULL;
}

kernel_pid_t dispacher_init(char *stack, int stacksize, char priority, 
    const char *name, kernel_pid_t hdlc_thread_pid)
{
    kernel_pid_t res;

    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST, 
            _dispatcher, (void *) (int) hdlc_thread_pid, name);

    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}
