#pragma once

#define SOIL_DRY_VALUE 500
#define SOIL_WET_VALUE 150

#include "adc.h"
#include <stdint.h>


ADC_Error_t soil_init(ADC_Channel_t channel);

uint16_t soil_measure_raw(ADC_Channel_t channel);

uint8_t soil_measure_percentage(ADC_Channel_t channel);