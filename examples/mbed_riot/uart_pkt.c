/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
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
 * @file        uart_pkt.c
 * @brief       Helper library for creating packets to be over UART via hdlc.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "uart_pkt.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

void *uart_pkt_insert_hdr(void *buf, size_t buf_len, const uart_pkt_hdr_t *hdr)
{
    if (buf_len < UART_PKT_HDR_LEN) {
        DEBUG("Buffer size too small\n");
        return NULL;
    }

    memcpy(buf, hdr, sizeof(uart_pkt_hdr_t));
    return (buf + UART_PKT_DATA_FIELD);
}

/**
 * Copy data from an array into a uart packet buffer.
 * @param  buf      destination buffer
 * @param  buf_len  destination buffer size
 * @param  data     buffer containing data
 * @param  data_len size of buffer containing data
 * @return          total size of packet on success or 0 on failure.
 */
size_t uart_pkt_cpy_data(void *buf, size_t buf_len, const void *data, 
    size_t data_len)
{
    if (data_len + 5 > buf_len) {
        DEBUG("Not enough space in destination buffer\n");
        return 0;
    }

    memcpy(buf + UART_PKT_HDR_LEN, data, data_len);
    return (UART_PKT_HDR_LEN + data_len);
}

int uart_pkt_parse_hdr(uart_pkt_hdr_t *dst_hdr, const void *src, size_t src_len)
{
    if(src_len < 5) {
        DEBUG("Invalid source buffer size.\n");
        return -1;
    }

    memcpy(dst_hdr, src, UART_PKT_HDR_LEN);
    return 0;
}

void *uart_pkt_get_data(void *src, size_t src_len)
{
    if (src_len < UART_PKT_HDR_LEN) {
        DEBUG("Invalid source buffer size.\n");
        return NULL;
    }
    return (src + UART_PKT_HDR_LEN);
}