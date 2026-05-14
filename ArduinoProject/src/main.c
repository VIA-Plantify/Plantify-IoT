#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
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
#include "data_server.h"

static uint8_t humidity_integer, humidity_decimal;
static uint8_t temperature_integer, temperature_decimal;

int main(void)
{
    led_init();
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

    sei();

    printf("SEP4 IoT Hardware\n");
    tone_play_startup();

    if (button_get(2))
        interactive_demo();

    if (button_get(3))
        data_server_run();

    mqtt_raw_connect();

    while (1)
    {
        uint16_t light_raw;
        uint16_t light_value;
        uint16_t soil_value;
        uint16_t distance_mm;
        uint16_t water_level;

        char payload[160];

        dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal);

        light_raw = light_measure_raw();
        light_value = 1023 - light_raw;

        soil_value = soil_measure_percentage(ADC_PK0);

        distance_mm = proximity_measure();

        water_level =
            proximity_calculate_water_level_percent(distance_mm);

        printf("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u W:%u%%\n",
               temperature_integer, temperature_decimal,
               humidity_integer, humidity_decimal,
               light_value, soil_value, distance_mm, water_level);

        display_int((temperature_integer * 10) + temperature_decimal);

        mqtt_handle_incoming();

        if (mqtt_is_connected())
        {
            snprintf(payload, sizeof(payload),
                     "{\"mac\":\"%s\",\"temp\":%u.%u,\"hum\":%u.%u,"
                     "\"light\":%u,\"soil\":%u,\"dist\":%u,"
                     "\"waterLevel\":%u}",
                     mqtt_get_device_mac(),
                     temperature_integer, temperature_decimal,
                     humidity_integer, humidity_decimal,
                     light_value, soil_value, distance_mm,
                     water_level);

            uint8_t published = 0;

            for (uint8_t attempt = 0; attempt < 3; attempt++)
            {
                if (mqtt_raw_publish(payload))
                {
                    published = 1;
                    break;
                }

                printf("Publish failed (attempt %d). Reconnecting...\n",
                       attempt + 1);

                wifi_command_close_TCP_connection();
                _delay_ms(1000);
                mqtt_raw_connect();
            }

            if (!published)
                printf("Publish failed after 3 attempts. Skipping.\n");

            for (uint16_t s = 0; s < 300; s += 10)
            {
                _delay_ms(10000);
                mqtt_tick(10);
                mqtt_handle_incoming();
            }
        }
        else
        {
            printf("MQTT offline. Retrying...\n");
            _delay_ms(1000);
            mqtt_raw_connect();
        }
    }
}