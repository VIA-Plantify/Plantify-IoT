#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "proximity.h"

#define DDR_Trig  DDRL
#define P_Trig    PL7
#define PORT_trig PORTL

#define PIN_Echo  PINL
#define P_Echo    PL6

volatile static bool _timeout = false;
static uint16_t _last_distance = 0;

uint16_t proximity_get_distance()
{
    return _last_distance;
}

void proximity_init()
{
    TCCR5A = 0;
    TCCR5B = (1 << WGM52);
    OCR5A = 12500;
    TIMSK5 |= (1 << OCIE5A);

    DDR_Trig |= (1 << P_Trig);
}

uint16_t proximity_measure()
{
    uint32_t cnt;
    bool done;

    PORT_trig |= (1 << P_Trig);
    _delay_us(10);
    PORT_trig &= ~(1 << P_Trig);

    cnt = 0;
    done = false;
    _timeout = false;

    TCCR5B |= (1 << CS51) | (1 << CS50);

    while (!_timeout && !(PIN_Echo & (1 << P_Echo)))
    {
        ;
    }

    TCNT5 = 0;

    while (!_timeout && !done)
    {
        if (0 == (PIN_Echo & (1 << P_Echo)))
        {
            cnt = (uint32_t)TCNT5;
            done = true;
        }
    }

    if (_timeout)
    {
        _last_distance = UINT16_MAX;
    }
    else
    {
        _last_distance = (uint16_t)(cnt * 343ul / 500ul);
    }

    TCCR5B &= ~((1 << CS51) | (1 << CS50));

    return _last_distance;
}

uint16_t proximity_calculate_water_level_percent(uint16_t distance_mm)
{
    const uint16_t min_distance_mm = 5;   // 100% full
    const uint16_t max_distance_mm = 45;  // 0% empty

    if (distance_mm == UINT16_MAX)
    {
        return 0;
    }

    if (distance_mm <= min_distance_mm)
    {
        return 100;
    }

    if (distance_mm >= max_distance_mm)
    {
        return 0;
    }

    return ((max_distance_mm - distance_mm) * 100) /
           (max_distance_mm - min_distance_mm);
}

ISR(TIMER5_COMPA_vect)
{
    _timeout = true;
}