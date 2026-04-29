#include "pump.h"
#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>

void pump_init(void)
{
    DDRC |= (1 << PUMP_PIN);
}

void pump_on(void)
{
    PORTC |= (1 << PUMP_PIN); 
}

void pump_off(void)
{
    PORTC &= ~(1 << PUMP_PIN);
}

bool pump_is_running(void)
{
    return PINC & (1 << PUMP_PIN);
}

void pump_run_for(uint32_t miliseconds)
{
    pump_on();
    for(uint32_t i = 0; i < miliseconds; i++)
    {
        _delay_ms(1);
    }
    pump_off();
}