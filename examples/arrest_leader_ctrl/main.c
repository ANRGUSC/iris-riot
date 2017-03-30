/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
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
 * @brief       Application to control ARREST leader (m3pi).
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "ringbuffer.h"
#include "periph/uart.h"
#include "net/gnrc.h"
#include "net/gnrc/netapi.h"
#include "../mbed_riot/main-conf.h"


#define SHELL_BUFSIZE       (128U)
#define UART_BUFSIZE        (128U)

#define PRINTER_PRIO        (THREAD_PRIORITY_MAIN - 1)

/* for the tx, keeping debug mode enabled is probably a good idea */
#define ENABLE_DEBUG        (1)
#include "debug.h"


#define MAX_TX_POWER            7       //valid range: -27 to 7
/* set default chan in makefile */


typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx;

static kernel_pid_t main_pid;
static msg_t msg_queue[8];

static void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)arg;

    ringbuffer_add_one(&(ctx.rx_buf), data);
    msg_t msg;
    msg.content.value = (uint32_t)dev;
    msg_send(&msg, main_pid);
}

int main(void)
{
    int res;
    uint8_t hwaddr_long[8];
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];

    /* initialize ringbuffer */
    ringbuffer_init(&(ctx.rx_buf), ctx.rx_mem, UART_BUFSIZE);

    /* get hwaddr and print for reference */
    gnrc_netif_get(ifs); 
    res = gnrc_netapi_get(ifs[0], NETOPT_ADDRESS_LONG, 0, hwaddr_long, sizeof(hwaddr_long));
    char hwaddr_long_str[res * 3];
    DEBUG(" hwaddr_long: ");
    DEBUG("%s\n", gnrc_netif_addr_to_str(hwaddr_long_str, sizeof(hwaddr_long_str),
                                            hwaddr_long, res));

    uint16_t channel = ARREST_DATA_CHANNEL;
    uint16_t tx_power = MAX_TX_POWER;
    gnrc_netapi_set(ifs[0], NETOPT_CHANNEL, 0, &channel, sizeof(uint16_t));
    gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    
    /* setup IPC queue for main thread*/
    msg_t msg;
    msg_init_queue(msg_queue, 8);

    uint8_t hwaddr[2];
    size_t hwaddr_len;
    gnrc_pktsnip_t *pkt, *hdr;

    hwaddr_len = gnrc_netif_addr_from_str(hwaddr, sizeof(hwaddr), ARREST_LEADER_SHORT_HWADDR);

    main_pid = thread_getpid();

    uart_init(UART_DEV(0), 115200, rx_cb, (void *) UART_DEV(0));

    char buf[2] = {REMOTE_CTRL_FLAG, 0};

    while (1) {
        msg_receive(&msg);

        while (1) 
        {
            int ret = (int)ringbuffer_get_one(&(ctx.rx_buf));
            if (ret == -1) {
                break;
            } else {
                buf[1] = (char) ret;
                printf("%c\n", buf[1]);
                fflush(stdout);

                /* put packet together */
                pkt = gnrc_pktbuf_add(NULL, &buf, sizeof(buf), GNRC_NETTYPE_UNDEF);
                if (pkt == NULL) {
                    puts("error: packet buffer full");
                    return 1;
                }

                hdr = gnrc_netif_hdr_build(NULL, 0, hwaddr, hwaddr_len);
                if (hdr == NULL) {
                    puts("error: packet buffer full");
                    gnrc_pktbuf_release(pkt);
                    return 1;
                }
                LL_PREPEND(pkt, hdr);

                //make sure no packets are to be sent!!
                if (gnrc_netapi_send(ifs[0], pkt) < 1) {
                    puts("error: unable to send");
                    gnrc_pktbuf_release(pkt);
                }
            }
        }
    }

    /* this should never be reached */
    return 0;
}