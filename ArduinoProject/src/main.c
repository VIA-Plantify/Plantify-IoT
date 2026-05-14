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
#include "interactive.h"

static uint8_t humidity_integer,    humidity_decimal;
static uint8_t temperature_integer, temperature_decimal;

void pir_callback(void) { /* handled via pir_get_state() in loop */ }

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
        while (1) {}
    }

    sei();
    printf("SEP4 IoT Hardware\n");

    if (button_get(2))
        interactive_demo();

    mqtt_raw_connect();

    while (1)
    {
        uint16_t light_raw, light_value, soil_value, distance_mm;
        char     payload[128];

        dht11_get(&humidity_integer,    &humidity_decimal,
                  &temperature_integer, &temperature_decimal);

        /*  Light sensor (KY-018) is a photoresistor wired as a voltage
            divider — raw ADC value rises as light decreases (darker =
            higher number).  Invert so 0 = dark, 1023 = bright.       */
        light_raw   = light_measure_raw();
        light_value = 1023 - light_raw;

        soil_value  = soil_measure_percentage(ADC_PK0);
        distance_mm = proximity_measure();

        printf("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u\n",
               temperature_integer, temperature_decimal,
               humidity_integer,    humidity_decimal,
               light_value, soil_value, distance_mm);

        display_int((temperature_integer * 10) + temperature_decimal);
        mqtt_handle_incoming();

        if (mqtt_is_connected())
        {
            snprintf(payload, sizeof(payload),
                     "{\"mac\":\"%s\",\"temp\":%u.%u,\"hum\":%u.%u,"
                     "\"light\":%u,\"soil\":%u,\"dist\":%u}",
                     mqtt_get_device_mac(),
                     temperature_integer, temperature_decimal,
                     humidity_integer,    humidity_decimal,
                     light_value, soil_value, distance_mm);

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
}