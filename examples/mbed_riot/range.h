/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
 * Richard Kim
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
 * @file        range.h
 * @brief       Ultrasound ranging library for localization
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
 *
 * @}
 */

#ifndef RANGE_H
#define RANGE_H

#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/pktdump.h"
#include "net/netdev.h"
#include "timex.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "board.h"
#include "thread.h"
#include "msg.h"
#include "range_param.h"
#include "net/hdlc.h"
#include "net/uart_pkt.h"

#undef BIT
#define BIT(n) ( 1 << (n) )
/* Bit field definitions for the UART Line Control Register: */
#define FEN   BIT( 4) /**< Enable FIFOs */
#define UART_CTL_HSE_VALUE    0

#define RANGE_MAX_ITER				  30000
#define ULTRSND_TIMEOUT               99000 //usec
#define RANGE_TIMEO_USEC              250000
#define TRANSMIT_DELAY                100000 //this is 100ms which is the minimum delay between pings

#define DATA_PER_PKT        ((HDLC_MAX_PKT_SIZE - UART_PKT_HDR_LEN - 1) / RANGE_DATA_LEN)

typedef struct __attribute__((packed)) {
    uint8_t         last_pkt;      
    range_data_t    data[DATA_PER_PKT];                  
} range_hdr_t;

//Description of ranging modes:
//ONE_SENSOR_MODE: Uses the RX_ONE_PIN (defined as GPIO_PD3 or DIO0) to listen for pings
//TWO_SENSOR_MODE: Uses the RX_ONE_PIN and RX_TWO_PIN (defined as GPIO_PD2 or DIO1) to 
//				   listen for pings and calculate the difference between their respective
//				   Time Difference of Arrival (TDoA) to get their Orientation Differential
//				   (OD)
//XOR_SENSOR_MODE: Uses the XOR_PIN (defined as GPIO1 or DDIO2) to calculate TDoA and OD.
//				   The XOR_SENSOR_MODE works by taking the outputs of the two sensors and 
//				   putting them through an XOR gate to get a pulse where the rising edge
//				   indicates the TDoA and the pulse width indicates the OD. This is 
//				   potentially advantageous for orientation because if the two events are
//				   close enough together, the XOR_PIN will miss the pulse and read a 
//				   secondary pulse (which is garbage data) that will register a higher
//				   TDoA and higher OD, which can be used as an indicator that the sensors
//				   are facing.
//				   
//The circuit diagram can be found at https://docs.google.com/a/usc.edu/document/d/1dAOTpsOR8ieO7aiEL8bq6xvZ6yDooY9ftndhAIMkdJg/edit?usp=sharing

/**
 * @brief      This function gets the ranging data, packages them into packets, and sends them down the hdlc to the mbed
 *
 * @param      params    The ranging parameters
 * @param[in]  hdlc_pid  The hdlc pid
 */
void range_and_send(range_params_t *params, kernel_pid_t hdlc_pid, uint16_t src_port, uint16_t mbed_port);

/**
 *
 * @brief      { This function call will cause the openmote to listen for the next RF
 * 				 and ultrasound ping from an anchor node and will calculate the TDoA
 * 				 and the OD between its two sensors. }
 *
 * @param[in]  timeout_usec  The max number of microseconds to wait before timeout
 * @param[in]  range_mode    The range mode. Available options are ONE_SENSOR_MODE, 
 * 							 TWO_SENSOR_MODE, or XOR_SENSOR_MODE.
 * @param[in]  num_samples   The number of samples to take
 * @param[in]  node_id       The node identifier (if it's -1 return the first node recieved)
 *
 * @return     { A pointer to the array of range_data_t with size num_samples. 
 * 				 Range_data_t consists of the TDoA and the delay between two sensors 
 * 				 (if applicable) and an error value to indicate if a pin missed an
 * 				 ultrasonic ping }
 */
range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, int8_t node_id, uint32_t max_iter);


/**
 * @brief      { This function call will cause the openmote to go into anchor mode where
 *				 it will transmit RF and Ultrasound pings in a loop }
 *
 * @param[in]  delay_usec  The delay in microseconds between pings (usually set at 100 ms)
 *
 * @return     { Doesn't return anything if successful because it will be infinitely looping }
 */
int range_tx(void);

/**
 * @brief      { This function call will cause the openmote to go into anchor mode where
 *				 it will transmit RF and Ultrasound pings according to a TDMA schedule,
 *				 dependent on tdma_master.c}
 *
 * @param[in]  delay_usec  The delay in microseconds between pings (usually set at 100 ms)
 *
 * @return     { Doesn't return anything if successful because it will be infinitely looping }
 */
int range_tx_tdma(void);

#endif
