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
/*  Sensor read + publish helper                                        */
/* ------------------------------------------------------------------ */
static void read_and_publish(void)
{
    uint16_t light_raw, light_value, soil_value, distance_mm, water_level;
    char payload[160];

    dht11_get(&humidity_integer, &humidity_decimal,
              &temperature_integer, &temperature_decimal);

    light_raw = light_measure_raw();
    light_value = 1023 - light_raw;
    soil_value = soil_measure_percentage(ADC_PK0);
    distance_mm = proximity_measure();
    water_level = proximity_calculate_water_level_percent(distance_mm);

    printf_P(PSTR("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u W:%u%%\n"),
             temperature_integer, temperature_decimal,
             humidity_integer, humidity_decimal,
             light_value, soil_value, distance_mm, water_level);

    display_int((temperature_integer * 10) + temperature_decimal);

    snprintf(payload, sizeof(payload),
             "{\"mac\":\"%s\",\"temp\":%u.%u,\"hum\":%u.%u,"
             "\"light\":%u,\"soil\":%u,\"dist\":%u,\"waterLevel\":%u}",
             mqtt_get_device_mac(),
             temperature_integer, temperature_decimal,
             humidity_integer, humidity_decimal,
             light_value, soil_value, distance_mm, water_level);

    if (!mqtt_raw_publish(payload))
    {
        printf_P(PSTR("Publish failed. Reconnecting...\n"));
        wifi_command_close_TCP_connection();
        _delay_ms(1000);
        mqtt_raw_connect();
        if (mqtt_is_connected())
            mqtt_raw_publish(payload);
    }
}

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

    /* Button 1 — wipe credentials + start captive portal */
    if (button_get(1))
    {
        _delay_ms(50);
        if (button_get(1))
        {
            char empty[32] = {0};
            save_credentials(empty, empty);
            printf_P(PSTR("Credentials wiped! Starting portal...\n"));
            _delay_ms(500);
            start_captive_portal(); /* never returns */
        }
    }

    /* Button 2 — interactive driver demo */
    if (button_get(2))
    {
        _delay_ms(50);
        if (button_get(2))
            interactive_demo(); /* never returns */
    }

    /* Button 3 — local TCP data export */
    if (button_get(3))
    {
        _delay_ms(50);
        if (button_get(3))
            data_server_run(); /* never returns */
    }

    /* Check EEPROM for saved WiFi credentials */
    char saved_ssid[32] = {0};
    char saved_pass[64] = {0};
    load_credentials(saved_ssid, saved_pass);

    if (saved_ssid[0] == '\0' || (uint8_t)saved_ssid[0] == 0xFF)
    {
        printf_P(PSTR("No saved WiFi credentials. Starting portal...\n"));
        printf_P(PSTR("Connect to 'PlantPot-Setup' and open http://192.168.4.1\n"));
        start_captive_portal(); /* never returns */
    }

    tone_play_startup();

    mqtt_raw_connect();

    /* Publish immediately on first connect */
    if (mqtt_is_connected())
    {
        printf_P(PSTR("Initial publish...\n"));
        read_and_publish();
    }

    while (1)
    {
        mqtt_handle_incoming();

        if (mqtt_is_connected())
        {
            /* 5 minute wait with keepalive ticks every 10s */
            for (uint16_t s = 0; s < 3600; s += 10)
            {
                _delay_ms(10000);
                mqtt_tick(10);
                mqtt_handle_incoming();

                /* Print live readings every minute */
                if (s % 60 == 0)
                {
                    uint16_t light_raw = light_measure_raw();
                    uint16_t light_value = 1023 - light_raw;
                    uint16_t soil_value = soil_measure_percentage(ADC_PK0);
                    uint16_t distance_mm = proximity_measure();
                    uint16_t water_level = proximity_calculate_water_level_percent(distance_mm);
                    dht11_get(&humidity_integer, &humidity_decimal,
                              &temperature_integer, &temperature_decimal);
                    printf_P(PSTR("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u W:%u%%\n"),
                             temperature_integer, temperature_decimal,
                             humidity_integer, humidity_decimal,
                             light_value, soil_value, distance_mm, water_level);
                    display_int((temperature_integer * 10) + temperature_decimal);
                }
            }

            read_and_publish();
        }
        else
        {
            printf_P(PSTR("MQTT offline. Retrying...\n"));
            _delay_ms(1000);
            mqtt_raw_connect();
            if (mqtt_is_connected())
            {
                printf_P(PSTR("Reconnected. Publishing...\n"));
                read_and_publish();
            }
        }
    }
}