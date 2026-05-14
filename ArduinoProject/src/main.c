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

static uint8_t humidity_integer, humidity_decimal;
static uint8_t temperature_integer, temperature_decimal;

extern char _device_mac[18];

/* ------------------------------------------------------------------ */
/*  FIX: software_reset was called but never defined anywhere.         */
/*  Correct AVR soft-reset: enable watchdog with shortest timeout,    */
/*  then spin — watchdog fires in 15ms and resets the MCU.            */
/* ------------------------------------------------------------------ */

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

    _delay_ms(2000);
    wifi_command_AT();
    wifi_command_disable_echo();
    wifi_command("AT+CWAUTOCONN=0", 2);
    wifi_command("AT+CWQAP", 2);
    _delay_ms(500);

    char ssid[32];
    char password[64];

    load_credentials(ssid, password);

    if (ssid[0] == '\0' || (uint8_t)ssid[0] == 0xFF)
    {
        printf_P(PSTR("No credentials - starting portal\n"));
        start_captive_portal();
    }

    printf_P(PSTR("Connecting to: %s\n"), ssid);

    if (!mqtt_raw_connect_with_credentials(ssid, password))
    {
        printf_P(PSTR("MQTT connect failed - retrying\n"));
        _delay_ms(3000);
        software_reset();
    }

        /* ----------------------------------------------------------------
           Main sensor loop
        ---------------------------------------------------------------- */
        while (1)
    {
        uint16_t light_value, soil_value, distance_mm;
        uint8_t motion;
        char payload[128];

        dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal);
        light_value = light_measure_raw();
        soil_value = soil_measure_percentage(ADC_PK0);
        distance_mm = proximity_measure();
        motion = (pir_get_state() != PIR_NO_MOTION) ? 1 : 0;

        printf_P(PSTR("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u M:%u\n"),
                 temperature_integer, temperature_decimal,
                 humidity_integer, humidity_decimal,
                 light_value, soil_value, distance_mm, motion);

        display_int((temperature_integer * 10) + temperature_decimal);
        mqtt_handle_incoming();

        if (mqtt_is_connected())
        {
            snprintf(payload, sizeof(payload),
                     "{\"mac\":\"%s\",\"temp\":%u.%u,\"hum\":%u.%u,"
                     "\"light\":%u,\"soil\":%u,\"dist\":%u,\"motion\":%u}",
                     _device_mac,
                     temperature_integer, temperature_decimal,
                     humidity_integer, humidity_decimal,
                     light_value, soil_value, distance_mm, motion);

            if (!mqtt_raw_publish(payload))
            {
                printf_P(PSTR("Publish failed. Reconnecting...\n"));
                wifi_command_close_TCP_connection();
                _delay_ms(1000);
                mqtt_raw_connect_with_credentials(ssid, password);
            }
            mqtt_tick(3);
        }
        else
        {
            printf_P(PSTR("MQTT offline. Retrying...\n"));
            _delay_ms(1000);
            mqtt_raw_connect_with_credentials(ssid, password);
        }

        _delay_ms(3000);
    }
}