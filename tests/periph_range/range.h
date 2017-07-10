#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "net/netdev.h"
#include "timex.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/adc.h"
#include "math.h"
#include "range_param.h"

#include "thread.h"
#include "msg.h"
#include "range_param.h"

static unsigned int gpio_lines[]={GPIO_PIN(3, 3), GPIO_PIN(3, 2), GPIO_PIN(3, 1)};


int range_rx(int argc, char **argv);

//BY DEFAULT USES GPIO_PD2 **will need to fix
int range_tx(int argc, char **argv);