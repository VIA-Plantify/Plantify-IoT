#include "dht11.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stddef.h>
#include <util/delay.h>

// Data pin configuration
#define DATA_BIT    PL1
#define DATA_PIN    PINL
#define DATA_DDR    DDRL
#define DATA_PORT   PORTL

// Named constants replacing magic numbers
#define MAX_TIMINGS             85
#define DHT11_START_SIGNAL_MS   18
#define DHT11_RESPONSE_WAIT_US  40
#define DHT11_BIT_THRESHOLD_US  26
#define DHT11_TIMEOUT_COUNT     255
#define DHT11_EXPECTED_BITS     40
#define DHT11_SKIP_TRANSITIONS  4

static void set_pin_output(void)
{
    DATA_DDR  |=  (1 << DATA_BIT);
}

static void set_pin_input(void)
{
    DATA_DDR  &= ~(1 << DATA_BIT);
    DATA_PORT |=  (1 << DATA_BIT); // Enable pull-up
}

static void pin_low(void)  { DATA_PORT &= ~(1 << DATA_BIT); }
static void pin_high(void) { DATA_PORT |=  (1 << DATA_BIT); }
static uint8_t pin_read(void) { return (DATA_PIN & (1 << DATA_BIT)) ? 1 : 0; }

static void write_outputs(uint8_t* hi, uint8_t* hd, uint8_t* ti, uint8_t* td,
                           uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3)
{
    if (hi) *hi = v0;
    if (hd) *hd = v1;
    if (ti) *ti = v2;
    if (td) *td = v3;
}

DHT11_ERROR_MESSAGE_t dht11_get(uint8_t* humidity_integer, uint8_t* humidity_decimal,
                                 uint8_t* temperature_integer, uint8_t* temperature_decimal)
{
    uint8_t _sreg;
    uint8_t laststate;
    uint8_t counter = 0;
    uint8_t j       = 0;
    uint8_t data[5] = {0, 0, 0, 0, 0};

    // Step 1: Send start signal — pull pin LOW for 18ms
    set_pin_output();
    pin_low();
    _delay_ms(DHT11_START_SIGNAL_MS);

    // Disable interrupts — timing from here is microsecond-sensitive
    _sreg = SREG;
    cli();

    // Step 2: Release line and wait for DHT11 response
    pin_high();
    _delay_us(DHT11_RESPONSE_WAIT_US);

    // Step 3: Switch to input and read actual pin state
    set_pin_input();
    laststate = pin_read();

    // Step 4: Read up to MAX_TIMINGS transitions
    for (uint8_t i = 0; i < MAX_TIMINGS; i++)
    {
        counter = 0;

        // Count how long the pin stays in its current state
        while (pin_read() == laststate)
        {
            counter++;
            _delay_us(1);
            if (counter == DHT11_TIMEOUT_COUNT)
                break;
        }

        laststate = pin_read();

        if (counter == DHT11_TIMEOUT_COUNT)
            break;

        // Skip the first 4 transitions (DHT11 acknowledgment pulses)
        // Only process even-indexed transitions after that (the data pulses)
        if ((i >= DHT11_SKIP_TRANSITIONS) && (i % 2 == 0))
        {
            data[j / 8] <<= 1;
            if (counter > DHT11_BIT_THRESHOLD_US)
                data[j / 8] |= 1;
            j++;
        }
    }

    // Restore interrupt state
    SREG = _sreg;

    // Step 5: Validate — need 40 bits and a passing checksum
    if ((j >= DHT11_EXPECTED_BITS) &&
        (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)))
    {
        write_outputs(humidity_integer, humidity_decimal,
                      temperature_integer, temperature_decimal,
                      data[0], data[1], data[2], data[3]);
        return DHT11_OK;
    }

    // Distinguish timeout from checksum failure
    if (counter == DHT11_TIMEOUT_COUNT)
    {
        write_outputs(humidity_integer, humidity_decimal,
                      temperature_integer, temperature_decimal,
                      0, 0, 0, 0);
        return DHT11_TIMEOUT;
    }

    write_outputs(humidity_integer, humidity_decimal,
                  temperature_integer, temperature_decimal,
                  0, 0, 0, 0);
    return DHT11_FAIL;
}

// Retry wrapper — DHT11 occasionally fails, 3 attempts with 1s gap
DHT11_ERROR_MESSAGE_t dht11_get_reliable(uint8_t* humidity_integer, uint8_t* humidity_decimal,
                                          uint8_t* temperature_integer, uint8_t* temperature_decimal)
{
    DHT11_ERROR_MESSAGE_t result;
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        result = dht11_get(humidity_integer, humidity_decimal,
                           temperature_integer, temperature_decimal);
        if (result == DHT11_OK)
            return DHT11_OK;
        _delay_ms(1000); // DHT11 needs at least 1s between readings
    }
    return result;
}