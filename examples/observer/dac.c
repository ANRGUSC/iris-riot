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
 * @file        dac.c
 * @brief       DAC library for interfacing with the MCP4921 chip
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
 *
 * @}
 */


#include "dac.h"

#define DAC_GAIN             5
#define DAC_ACTIVE           4
#define DAC_DATA_MASK            0xf0
#define DAC_DATA_OFFSET          4
#define DAC_DATA_SIZE            2

static gpio_t chip_select_pin;


int init_dac(gpio_t cs, spi_clk_t clk){
    chip_select_pin = cs;
    spi_init(SPI_DEV(0));
    spi_init_cs(SPI_DEV(0), chip_select_pin);
    return spi_acquire(SPI_DEV(0),chip_select_pin, SPI_MODE_0, clk);
}


int set_voltage(uint8_t val, DAC_gain_t gain){
    char buff[DAC_DATA_SIZE];

     if( val < 0 || val > 255){
        //printf("Value must be between 0 and 255");
        return 0;
    }


    buff[0] = 0;
    buff[1] = 0;

    if(gain == DAC_GAIN_1){
        buff[0] |= (1 << DAC_GAIN);
    }

    buff[0] |= (1 << DAC_ACTIVE);

    buff[0] |= ((val & DAC_DATA_MASK) >> DAC_DATA_OFFSET);
    buff[1] |= ((val & ~DAC_DATA_MASK) << DAC_DATA_OFFSET);

    spi_transfer_byte(SPI_DEV(0), chip_select_pin, true, buff[0]);
    spi_transfer_byte(SPI_DEV(0), chip_select_pin, false, buff[1]);

    return 1;
}

void stop_dac(void){
    spi_release(SPI_DEV(0));
    return;
}