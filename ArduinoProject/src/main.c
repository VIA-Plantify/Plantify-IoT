#include <avr/io.h>
#include <avr/interrupt.h>
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
#include "interactive.h"
#include "eeprom_storage.h"
#include "captive_portal.h"
#include <avr/wdt.h>

static uint8_t humidity_integer, humidity_decimal;
static uint8_t temperature_integer, temperature_decimal;

extern char _device_mac[18];

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
        while (1) {}
    }

    led_on(2);
    sei();
    printf("SEP4 IoT Hardware\n");

    if (button_get(2))
        interactive_demo();

    _delay_ms(2000);
    wifi_command_AT();
    wifi_command_disable_echo();

    char ssid[32];
    char password[64];

    load_credentials(ssid, password);
    printf("SSID: %s\n", ssid);

    load_credentials(ssid, password);
    printf("SSID: %s\n", ssid);

    if (ssid[0] == '\0' || (uint8_t)ssid[0] == 0xFF)
    {
        printf("No credentials - starting portal\n");
        start_captive_portal();
    }

    while(1);
}
/*int main(void)
{
    led_init();
    led_on(1);  // LED 1 on = code is running

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
        led_on(4);  // LED 4 on = uart init failed
        while (1) {}
    }

    led_on(2);  // LED 2 on = uart init succeeded
    sei();
    printf("SEP4 IoT Hardware\n");

    if (button_get(2))
        interactive_demo();

    _delay_ms(2000);
    wifi_command_AT();
    wifi_command_disable_echo();

    // ─── Credentials Check ────────────────────────────────────────────────────
    char ssid[32];
    char password[64];

    load_credentials(ssid, password);

    if (ssid[0] == '\0' || (uint8_t)ssid[0] == 0xFF)
    {
        printf("No credentials - starting portal\n");
        start_captive_portal();
    }

    printf("Connecting to: %s\n", ssid);
    if (wifi_command_join_AP(ssid, password) != WIFI_OK)
    {
        printf("Connection failed - wiping credentials\n");
        char empty[64] = {0};
        save_credentials(empty, empty);
        software_reset();
    }

    printf("WiFi connected!\n");
    // ──────────────────────────────────────────────────────────────────────────

    mqtt_raw_connect();

    while (1)
    {
        uint16_t light_value, soil_value, distance_mm;
        uint8_t  motion;
        char     payload[128];

        dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal);
        light_value  = light_measure_raw();
        soil_value   = soil_measure_percentage(ADC_PK0);
        distance_mm  = proximity_measure();
        motion       = (pir_get_state() != PIR_NO_MOTION) ? 1 : 0;

        printf("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u M:%u\n",
               temperature_integer, temperature_decimal,
               humidity_integer,    humidity_decimal,
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
                     humidity_integer,    humidity_decimal,
                     light_value, soil_value, distance_mm, motion);

            if (!mqtt_raw_publish(payload))
            {
                printf("Publish failed. Reconnecting...\n");
                wifi_command_close_TCP_connection();
                _delay_ms(1000);
                mqtt_raw_connect();
            }
            mqtt_tick(3);
        }
        else
        {
            printf("MQTT offline. Retrying...\n");
            _delay_ms(1000);
            mqtt_raw_connect();
        }

        _delay_ms(3000);
    }
}*/