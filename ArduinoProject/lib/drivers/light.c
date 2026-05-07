#include "adc.h"
#include "light.h"
#include <stdint.h>

static uint16_t adc_measure_smooth(ADC_Channel_t channel);

ADC_Error_t light_init()
{
    return adc_create(ADC_PK7, ADC_REF_5V);
}

uint16_t light_measure_raw()
{
    uint16_t raw_value = adc_measure_smooth(ADC_PK7);
    
    for(uint8_t i = 0; i < 5; i++)
    {
        if(raw_value > 1023)
        {
            raw_value = adc_measure_smooth(ADC_PK7);
        }
        else
        {
            return raw_value;
        }
    }
    
    return UINT16_MAX; // Error
}

static uint16_t adc_measure_smooth(ADC_Channel_t channel)
{
    uint16_t sum = 0;
    uint16_t count = 0;
    uint16_t value;
    for(uint8_t i = 0; i < 10; i++)
    {
        value = adc_measure(channel);
        if(value > 1023)
        {
            continue;
        }
        else
        {
            sum += value;
            count++;
        }
    }

    if(count == 0)
    {
        return UINT16_MAX; // Error
    }

    return sum/count;
}