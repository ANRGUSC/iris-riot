#include <stdio.h>
#include "xtimer.h"
#include "periph/adc.h"

#define RES             ADC_RES_7BIT
#define DELAY           (100 * 1000U)

int main(void)
{
    uint32_t last = xtimer_now();
    int16_t sample = 0;

    while(1)
    {
        sample = adc_sample(ADC_LINE(2), RES);

        if (sample < 0) 
        {
            printf("ADC error\n");
        } 
        else 
        {
            printf("ADC output %d\n", sample);
        }

        xtimer_usleep_until(&last, DELAY);
    }

    return 0;
}