/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
 *
 * Permission is here
 * by granted, free of charge, to any person obtaining a copy 
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
 * @file        dac.h
 * @brief       Ultrasound ranging library for localization
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
 *
 * @}
 */


#ifndef DAC_H
#define DAC_H

#include "periph/spi.h"

#define DEFAULT_DAC_CS          GPIO_PIN(0,3)
#define DEFAULT_SENSOR_THRESH   40 // 60 for 5 V, 40 for regular 3.3 V

typedef enum {
	DAC_GAIN_2 = 0,         /**< sets the DAC to a gain of 2 */
    DAC_GAIN_1 = 1,     /**< sets the DAC to a gain of 1 */
    
} DAC_gain_t;

/**
 * @brief      Initializes the appropriate pins for SPI communication with the DAC
 *
 * @param[in]  cs    The chip select pin
 * @param[in]  clk   The clock speed
 *
 * @return     returns 1 if successful, 0 otherwise
 */
int init_dac(gpio_t cs, spi_clk_t clk);

/**
 * @brief      Converts val to DAC-readable data and sends it over SPI to set voltage
 *
 * @param[in]  val   The value between 0-255
 * @param[in]  gain  The gain
 *
 * @return     1 on success, 0 on fail
 */
int set_voltage(uint8_t val, DAC_gain_t gain);


/**
 * @brief      Shuts down the DAC line
 */
void stop_dac(void);

/* DAC_H */
#endif
