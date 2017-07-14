
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

#include "thread.h"
#include "msg.h"
#include "range_param.h"

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
 *
 * @brief      { This function call will cause the openmote to listen for the next RF
 * 				 and ultrasound ping from an anchor node and will calculate the TDoA
 * 				 and the OD between its two sensors. }
 *
 * @param[in]  timeout_usec  The max number of microseconds to wait before timeout
 * @param[in]  range_mode    The range mode. Available options are ONE_SENSOR_MODE, 
 * 							 TWO_SENSOR_MODE, or XOR_SENSOR_MODE.
 * @param[in]  num_samples   The number of samples to take
 *
 * @return     { A pointer to the array of range_data_t with size num_samples. 
 * 				 Range_data_t consists of the TDoA and the delay between two sensors 
 * 				 (if applicable) and an error value to indicate if a pin missed an
 * 				 ultrasonic ping }
 */
range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, uint16_t num_samples);



//TODO: Change to use TX_PIN instead of GPIO_PD2 and convert this function to only send out one ping
/**
 * @brief      { This function call will cause the openmote to go into anchor mode where
 *				 it will transmit RF and Ultrasound pings in a loop }
 *
 * @param[in]  delay_usec  The delay in microseconds between pings (usually set at 100 ms)
 *
 * @return     { Doesn't return anything if successful because it will be infinitely looping }
 */
int range_tx(uint32_t delay_usec);

#endif