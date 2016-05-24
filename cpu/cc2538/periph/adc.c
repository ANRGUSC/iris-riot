#include <stdio.h>
#include "board.h"
#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph_conf.h"
#include "cpu_conf.h"
#include "periph/adc.h"
#include "cc2538.h"

enum
{
    EOC = 1,
    STSEL = 3,
};

int adc_init(adc_t line)
{
    /* make sure the given ADC line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    cc2538_soc_adc_t *adcaccess;
    adcaccess = SOC_ADC;
    adcaccess->cc2538_adc_adccon1.ADCCON1 |= adcaccess->cc2538_adc_adccon1.ADCCON1bits.STSEL = STSEL;

    return 0;
}

int adc_sample(adc_t line, adc_res_t res)
{
    /* make sure the given ADC line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    cc2538_soc_adc_t *adcaccess = SOC_ADC;
    uint8_t refvoltage = SOC_ADC_ADCCON_REF_AVDD5; /* TODO: is hardcoded ref voltage OK? */
    int16_t result;

  /* Start a single extra conversion with the given parameters. */
    adcaccess->ADCCON3 = ((adcaccess->ADCCON3) & ~(SOC_ADC_ADCCON3_EREF | SOC_ADC_ADCCON3_EDIV | SOC_ADC_ADCCON3_ECH)) |
                         refvoltage | res | line;

    /* Poll until end of conversion */
    while(!((adcaccess->cc2538_adc_adccon1.ADCCON1) & (adcaccess->cc2538_adc_adccon1.ADCCON1bits.EOC = EOC)));

    /* Read conversion result, reading SOC_ADC_ADCH last to clear
    * SOC_ADC_ADCCON1.EOC */
    result  = (adcaccess->ADCL) & 0xfc;
    result |= (adcaccess->ADCH) << 8;

    /* Return conversion result */
    return result;
}