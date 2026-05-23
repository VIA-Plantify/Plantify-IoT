#include "dht11.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>

// Data pin configuration
#define DATA_BIT PL1
#define DATA_PIN PINL
#define DATA_DDR DDRL
#define DATA_PORT PORTL

#define MAX_TIMINGS 85

DHT11_ERROR_MESSAGE_t dht11_get(uint8_t *humidity_integer, uint8_t *humidity_decimal,
                                uint8_t *temperature_integer, uint8_t *temperature_decimal)
{
    uint8_t _sreg;
    uint8_t laststate = 1;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    uint8_t data[5];
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    /* pull pin down for 18 milliseconds */
    DATA_DDR |= (1 << DATA_BIT);
    DATA_PORT &= ~(1 << DATA_BIT);
    _delay_ms(18);

    // Time sensitive code, disable interrupts to avoid timing issues.
    _sreg = SREG;
    cli();

    DATA_PORT |= (1 << DATA_BIT);
    _delay_us(40);

    /* prepare to read the pin */
    DATA_DDR &= ~(1 << DATA_BIT);
    DATA_PORT |= (1 << DATA_BIT); // pull-up

    /* detect change and read data */
    for (i = 0; i < MAX_TIMINGS; i++)
    {
        counter = 0;
        while ((DATA_PIN & (1 << DATA_BIT)) == laststate)
        {
            counter++;
            _delay_us(1);
            if (counter == 255)
                break;
        }
        laststate = (DATA_PIN & (1 << DATA_BIT));

        if (counter == 255)
            break;

        /* ignore first 3 transitions */
        if ((i >= 4) && (i % 2 == 0))
        {
            /* shove each bit into the storage bytes */
            data[j / 8] <<= 1;
            if (counter > 26)
                data[j / 8] |= 1;
            j++;
        }
    }

    SREG = _sreg; // Restore interrupt state

    /*
     * check we read 40 bits (8bit x 5) + verify checksum in the last byte
     */
    if ((j >= 40) &&
        (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
    {
        if (humidity_integer != NULL)
            *humidity_integer = data[0];
        if (humidity_decimal != NULL)
            *humidity_decimal = data[1];
        if (temperature_integer != NULL)
            *temperature_integer = data[2];
        if (temperature_decimal != NULL)
            *temperature_decimal = data[3];
        return DHT11_OK;
    }
    else
    {
        if (humidity_integer != NULL)
            *humidity_integer = 0;
        if (humidity_decimal != NULL)
            *humidity_decimal = 0;
        if (temperature_integer != NULL)
            *temperature_integer = 0;
        if (temperature_decimal != NULL)
            *temperature_decimal = 0;
        return DHT11_FAIL;
    }
}
/* Retry wrapper — DHT11 occasionally fails, 3 attempts with 1s gap */
DHT11_ERROR_MESSAGE_t dht11_get_reliable(uint8_t *humidity_integer, uint8_t *humidity_decimal,
                                         uint8_t *temperature_integer, uint8_t *temperature_decimal)
{
    DHT11_ERROR_MESSAGE_t result;
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        result = dht11_get(humidity_integer, humidity_decimal,
                           temperature_integer, temperature_decimal);
        if (result == DHT11_OK)
            return DHT11_OK;
        _delay_ms(1000);
    }
    return result;
}