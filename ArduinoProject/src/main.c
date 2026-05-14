#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "mqtt.h"
#include "uart_stdio.h"
#include "led.h"
#include "button.h"
#include "display.h"
#include "proximity.h"
#include "light.h"
#include "soil.h"
#include "pir.h"
#include "wifi.h"
#include "servo.h"
#include "adc.h"
#include "dht11.h"
#include "tone.h"
#include "interactive.h"
#include "eeprom_storage.h"
#include "captive_portal.h"
#include "data_server.h"

static uint8_t humidity_integer, humidity_decimal;
static uint8_t temperature_integer, temperature_decimal;

extern char _device_mac[18];

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    MCUSR = 0;
    wdt_disable();

    led_init();
    led_on(1);

    button_init();
    display_init();
    proximity_init();
    light_init();
    soil_init(ADC_PK0);
    pir_init(pir_callback);
    wifi_init();
    servo_init(PWM_NORMAL);

    if (UART_OK != uart_stdio_init(115200))
    {
        led_on(4);
        while (1)
        {
        }
    }

    led_on(2);
    sei();
    printf_P(PSTR("SEP4 IoT Hardware\n"));
    tone_play_startup();

    /* Hold button 1 at boot to wipe saved WiFi credentials */
    if (button_get(1))
    {
        char empty[64] = {0};
        save_credentials(empty, empty);
        printf_P(PSTR("Credentials wiped! Rebooting...\n"));
        _delay_ms(1000);
        software_reset();
    }

    if (button_get(2))
        interactive_demo();

    /* Hold button 3 at boot to start TCP data export server */
    if (button_get(3))
        data_server_run(); /* never returns */

    /* ----------------------------------------------------------------
       No button pressed - run sensor loop over USB serial.
       plantify_logger.py will pick this up automatically.
    ---------------------------------------------------------------- */
    printf_P(PSTR("Running in serial sensor mode\n"));

    while (1)
    {
        uint16_t light_raw, light_value, soil_value, distance_mm;
        uint8_t motion;

        dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal);

        light_raw = light_measure_raw();
        light_value = 1023 - light_raw; /* invert: sensor measures darkness */

        soil_value = soil_measure_percentage(ADC_PK0);
        distance_mm = proximity_measure();
        motion = (pir_get_state() != PIR_NO_MOTION) ? 1 : 0;

        printf_P(PSTR("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u M:%u\n"),
                 temperature_integer, temperature_decimal,
                 humidity_integer, humidity_decimal,
                 light_value, soil_value, distance_mm, motion);

        display_int((temperature_integer * 10) + temperature_decimal);

        _delay_ms(1000);
    }
}