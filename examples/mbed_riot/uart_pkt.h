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
 * @file        uart_pkt.h
 * @brief       Helper library for creating packets to be over UART via hdlc.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * 
 */

#ifndef UART_PKT_H_
#define UART_PKT_H_

#define UART_PKT_HDR_LEN            6
#define UART_PKT_TYPE_FIELD         4
#define UART_PKT_DATA_FIELD         5

typedef struct __attribute__((packed)) {
    uint16_t    src_port;      
    uint16_t    dst_port;      
    uint8_t     pkt_type;
} uart_pkt_hdr_t;

/**
 * @brief Message types from mbed-os to riot-os
 */
typedef enum {
    RADIO_SET_CHAN          = 0,
    RADIO_SET_POWER         = 1,
    SOUND_RANGE_REQ         = 2,
    SOUND_RANGE_X10_REQ     = 3,
    RSSI_DUMP_START         = 4,
    RSSI_DUMP_STOP          = 5
} mbed_to_riot_t;

/**
 * @brief Message types from riot-os to mbed-os
 */
typedef enum  {
    RADIO_SET_CHAN_SUCCESS  = 0,
    RADIO_SET_CHAN_FAIL     = 1,
    RADIO_SET_POWER_SUCCESS = 2,
    RADIO_SET_POWER_FAIL    = 3,
    SOUND_RANGE_DONE        = 4,
    RSSI_SCAN_STARTED       = 5,
    RSSI_SCAN_STOPPED       = 6,
    RSSI_DATA_PKT           = 7,
    RADIO_FWD_UDP_PKT       = 8
} riot_to_mbed_t;

void *uart_pkt_insert_hdr(void *buf, size_t buf_len, const uart_pkt_hdr_t *hdr);
size_t uart_pkt_cpy_data(void *buf, size_t buf_len, const void *data, size_t data_len);
int uart_pkt_parse_hdr(uart_pkt_hdr_t *dst_hdr, const void *src,  size_t src_len);
void *uart_pkt_get_data(void *src, size_t src_len);

#endif /* UART_PKT_H_ */
