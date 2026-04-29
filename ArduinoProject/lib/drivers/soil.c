#include "adc.h"
#include "soil.h"
#include <stdint.h>

static uint16_t adc_measure_smooth(ADC_Channel_t channel);

ADC_Error_t soil_init(ADC_Channel_t channel)
{
    return adc_create(channel, ADC_REF_5V);
}

uint16_t soil_measure_raw(ADC_Channel_t channel)
{

    uint16_t raw_value = adc_measure_smooth(channel);

    for(uint8_t i = 0; i < 5; i++)
    {
        if(raw_value > 1023)
        {
            raw_value = adc_measure_smooth(channel);
        }
        else
        {
            return raw_value;
        }
    }
    
    return UINT16_MAX; // Error
}

uint8_t soil_measure_percentage(ADC_Channel_t channel)
{
    uint16_t raw = soil_measure_raw(channel);
    
    if(raw == UINT16_MAX)
    {
        return UINT8_MAX; // Error
    }

    if(raw > SOIL_DRY_VALUE)
    {
        raw = SOIL_DRY_VALUE;
    } 
    else if (raw < SOIL_WET_VALUE)
    {
        raw = SOIL_WET_VALUE;
    }

    return (SOIL_DRY_VALUE - raw) * 100 / (SOIL_DRY_VALUE-SOIL_WET_VALUE);
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