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
 * @author      Pradipta Ghosh <pradiptg@usc.edu>
 * @}
 */

#ifndef HDLC_H_
#define HDLC_H_
#ifdef __cplusplus
extern "C" {
#endif

/* this file does not provide anything on it's own */

#include "net/netdev.h"
#include "net/yahdlc.h"
#include "mutex.h"
#include "thread.h"
#include "board.h"
#include "periph/uart.h"

#ifndef HDLC_RTRY_TIMEO_USEC
    #define HDLC_RTRY_TIMEO_USEC         200000
#endif 

#ifndef HDLC_RETRANS_TIMEO_USEC
    #define HDLC_RETRANS_TIMEO_USEC   50000
#endif 

#ifndef HDLC_MAX_PKT_SIZE
    #define HDLC_MAX_PKT_SIZE       64
#endif

#ifndef HDLC_MSG_QUEUE_SIZE
    #define HDLC_MSG_QUEUE_SIZE       32 
#endif
    
typedef struct {
    yahdlc_control_t control;
    char *data;
    unsigned int length;
    mutex_t mtx;
} hdlc_buf_t;

/* struct for other threads to pass to hdlc thread via IPC */
typedef struct {
    char *data;
    unsigned int length;
} hdlc_pkt_t;

typedef struct hdlc_entry {
    struct hdlc_entry *next;
    uint16_t port;
    kernel_pid_t pid;
} hdlc_entry_t;

/* HDLC thread messages */
//Added MQTT_SN
enum {
    HDLC_MSG_REG_DISPATCHER = 0,
    HDLC_MSG_RECV           = 1,
    HDLC_MSG_SND            = 2,
    HDLC_MSG_RESEND         = 3,
    HDLC_MSG_SND_ACK        = 4,
    HDLC_RESP_RETRY_W_TIMEO = 5,
    HDLC_RESP_SND_SUCC      = 6,
    HDLC_PKT_RDY            = 7,
    MQTT_MBED               = 8,
    MQTT_RSSI               = 9,
    MQTT_SN                 = 10
};

void hdlc_register(hdlc_entry_t *entry);
void hdlc_unregister(hdlc_entry_t *entry);
int hdlc_pkt_release(hdlc_pkt_t *buf);
int hdlc_send_pkt(hdlc_pkt_t *pkt);
kernel_pid_t hdlc_init(char *stack, int stacksize, char priority, const char *name, uart_t dev);

#ifdef __cplusplus
}
#endif
#endif /* HDLC_H */
