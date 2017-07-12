
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

/**
 * @brief      { function_description }
 *
 * @param[in]  utimeout     The utimeout
 * @param[in]  sys_flag     The system flag
 * @param[in]  num_samples  The number samples
 *
 * @return     { description_of_the_return_value }
 */
range_data_t* range_rx(uint32_t utimeout, uint8_t sys_flag, uint16_t num_samples);


//BY DEFAULT USES GPIO_PD2 
//TODO:will need to fix
/**
 * @brief      { function_description }
 *
 * @param[in]  udelay  The udelay
 *
 * @return     { description_of_the_return_value }
 */
int range_tx(uint32_t udelay);

#endif