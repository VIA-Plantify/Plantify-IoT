/****************************************************************************
 * interactive.c
 * Merged interactive demo — combines original driver set with fixed/improved
 * version.
 *
 * Retained from original:
 *   - Pump control (cases 16, 17)
 *   - Soil moisture percentage (case 13)
 *   - Accelerometer stub (case 15 → renumbered to 18)
 *   - soil_init / pump_init in setup
 *
 * Retained from fixed version:
 *   - <stdbool.h> include
 *   - Safe input handling (no gets, uses _read_line_from_uart)
 *   - volatile flags for callbacks
 *   - Correct wifi_line_callback signature (void)
 *   - MQTT demo (case 14)
 *   - ESP8266 firmware check (case 15)
 *   - mqtt_* helper functions
 *
 * Author:  Merged build
 * Date:    2026
 * Project: SPE4_API
 ****************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "uart_stdio.h"
#include "led.h"
#include "pir.h"
#include "display.h"
#include "timer.h"
#include "wifi.h"
#include "button.h"
#include "buzzer.h"
#include "dht11.h"
#include "proximity.h"
#include "servo.h"
#include "adc.h"
#include "light.h"
#include "soil.h"
#include "tone.h"
#include "pump.h"
// #include "adxl345.h"

#define MAX_STRING_LENGTH 100
#define MAX_MENU_OPTIONS  19

#define MQTT_DEFAULT_BROKER_PORT 1883
#define MQTT_DEFAULT_CLIENT_ID   "mega2560_client"
#define MQTT_DEFAULT_TOPIC       "iot/mega/sensors"

#define WIFI_DEFAULT_SSID        "Ricky"
#define WIFI_DEFAULT_PASSWORD    "r89uuios"

static int x = 0;
static char _q_buff[5] = {0};
static volatile bool _tcp_string_received = false;
static volatile bool _pir_active = false;

static char _tcp_rx_buffer[MAX_STRING_LENGTH] = {0};
static char _line_buffer1[MAX_STRING_LENGTH] = {0};
static char _line_buffer2[MAX_STRING_LENGTH] = {0};

static char _broker_ip[MAX_STRING_LENGTH] = {0};
static char _mqtt_topic[MAX_STRING_LENGTH] = MQTT_DEFAULT_TOPIC;
static char _mqtt_client_id[40] = MQTT_DEFAULT_CLIENT_ID;

static const char welcome_text[] = "Welcome from SEP4 IoT hardware!\n";

/* -------------------------------------------------------------------------
 * UART helpers
 * ------------------------------------------------------------------------- */
static void _read_line_from_uart(char *buffer, uint8_t max_length)
{
    while (1)
    {
        uint8_t len = gets_nonblocking(buffer, max_length);
        if (len > 0)
        {
            buffer[strcspn(buffer, "\r\n")] = '\0';
            return;
        }
    }
}

static bool _quit(void)
{
    return (gets_nonblocking(_q_buff, sizeof(_q_buff)) > 0 && _q_buff[0] == 'q');
}

static void _prompt_line(const char *prompt, char *buffer, uint8_t size)
{
    printf("%s", prompt);
    _read_line_from_uart(buffer, size);
}

/* -------------------------------------------------------------------------
 * MQTT / WiFi helpers
 * ------------------------------------------------------------------------- */
static WIFI_ERROR_MESSAGE_t mqtt_send_command(const char *cmd, uint16_t timeout_s)
{
    size_t len = strlen(cmd);
    if (len > 150)
    {
        printf("MQTT command too long (%u chars). Shorten topic or client ID.\n",
               (unsigned)len);
        return WIFI_FAIL;
    }
    return wifi_command(cmd, timeout_s);
}

static uint8_t mqtt_prepare_wifi(void)
{
    if (wifi_command_AT() != WIFI_OK)
    {
        printf("ESP8266 is not responding.\n");
        return 0;
    }

    wifi_command_disable_echo();

    if (wifi_command_set_mode_to_1() != WIFI_OK)
    {
        printf("Failed to set WiFi station mode.\n");
        return 0;
    }

    return 1;
}

static uint8_t mqtt_configure_and_connect(const char *broker_ip,
                                          uint16_t broker_port,
                                          const char *client_id)
{
    char cmd[160];

    snprintf(cmd, sizeof(cmd),
             "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"",
             client_id);
    printf("Sending: %s\n", cmd);
    if (mqtt_send_command(cmd, 5) != WIFI_OK)
    {
        printf("MQTT client config failed.\n");
        return 0;
    }

    snprintf(cmd, sizeof(cmd),
             "AT+MQTTCONN=0,\"%s\",%u,0",
             broker_ip, broker_port);
    printf("Sending: %s\n", cmd);
    if (mqtt_send_command(cmd, 15) != WIFI_OK)
    {
        printf("MQTT broker connection failed.\n");
        printf("Check: broker IP correct? mosquitto running? allow_anonymous true?\n");
        return 0;
    }

    return 1;
}

static uint8_t mqtt_publish_text(const char *topic, const char *payload)
{
    char cmd[160];

    if (strchr(payload, '"') != NULL)
    {
        printf("Payload contains double quotes. Use plain text or CSV.\n");
        return 0;
    }

    snprintf(cmd, sizeof(cmd),
             "AT+MQTTPUB=0,\"%s\",\"%s\",0,0",
             topic, payload);

    printf("Sending: %s\n", cmd);
    if (mqtt_send_command(cmd, 5) != WIFI_OK)
    {
        printf("MQTT publish failed.\n");
        return 0;
    }

    return 1;
}

static void mqtt_publish_sensor_snapshot(void)
{
    uint8_t humidity_integer = 0, humidity_decimal = 0;
    uint8_t temperature_integer = 0, temperature_decimal = 0;
    uint16_t light_value = 0;
    uint16_t soil_value = 0;
    uint16_t distance_mm = 0;
    uint8_t motion = 0;
    char payload[96];

    if (dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal) != DHT11_OK)
    {
        printf("DHT11 read failed.\n");
        return;
    }

    light_value  = light_measure_raw();
    soil_value   = soil_measure_raw(ADC_PK0);
    distance_mm  = proximity_measure();
    motion       = (pir_get_state() != PIR_NO_MOTION) ? 1U : 0U;

    snprintf(payload, sizeof(payload),
             "temp=%u.%u,hum=%u.%u,light=%u,soil=%u,dist=%u,motion=%u",
             temperature_integer, temperature_decimal,
             humidity_integer, humidity_decimal,
             light_value, soil_value, distance_mm, motion);

    printf("Publishing: %s\n", payload);
    mqtt_publish_text(_mqtt_topic, payload);
}

/* -------------------------------------------------------------------------
 * Menu
 * ------------------------------------------------------------------------- */
uint8_t menu(void)
{
    int choice = 0;
    char line[16];
    char *endptr;

    printf("\t----------------- M E N U ------------------\n");
    printf("\t 1. Button and LED\n");
    printf("\t 2. PIR Sensor (HC-SR501)\n");
    printf("\t 3. Display\n");
    printf("\t 4. WiFi (ESP8266 TCP test)\n");
    printf("\t 5. stdio\n");
    printf("\t 6. Timer\n");
    printf("\t 7. Buzzer\n");
    printf("\t 8. Temperature and humidity sensor (DHT11)\n");
    printf("\t 9. Proximity sensor (HC-SR04)\n");
    printf("\t10. Servo motor (SG90)\n");
    printf("\t11. Light sensor (KY-018)\n");
    printf("\t12. Soil Moisture Sensor (raw)\n");
    printf("\t13. Soil Moisture Sensor (percentage)\n");
    printf("\t14. Play Star Wars theme on the speaker\n");
    printf("\t15. Show Accelerometer values (ADXL345)\n");
    printf("\t16. Pump control\n");
    printf("\t17. Pump run for X seconds\n");
    printf("\t18. Publish a message to MQTT broker\n");
    printf("\t19. Check ESP8266 firmware version\n");

    do
    {
        printf("Choose a driver to test (1-%d): ", MAX_MENU_OPTIONS);
        _read_line_from_uart(line, sizeof(line));

        choice = (int)strtol(line, &endptr, 10);

        if (line[0] == '\0' || endptr == line || *endptr != '\0' ||
            choice < 1 || choice > MAX_MENU_OPTIONS)
        {
            printf("Invalid input. Please enter a number between 1 and %d.\n",
                   MAX_MENU_OPTIONS);
            choice = 0;
        }
    } while (choice < 1 || choice > MAX_MENU_OPTIONS);

    return (uint8_t)choice;
}

/* -------------------------------------------------------------------------
 * Callbacks
 * ------------------------------------------------------------------------- */
void pir_callback(void)
{
    if (_pir_active)
    {
        if (pir_get_state() == PIR_NO_MOTION)
            led_off(1);
        else
            led_on(1);
    }
}

void led2_callback(uint8_t id)
{
    (void)id;
    led_toggle(2);
}

void start_stop_timer(uint8_t id)
{
    if (timer_get_state(id))
        timer_pause(id);
    else
        timer_resume(id);
}

void wifi_line_callback(void)
{
    _tcp_string_received = true;
}

/* -------------------------------------------------------------------------
 * Main demo loop
 * ------------------------------------------------------------------------- */
int interactive_demo(void)
{
    static int led2_timer_id = 0;

    /* Initialize drivers that need setup */
    pump_init();
    soil_init(ADC_PK0);

    while (1)
    {
        switch (menu())
        {
        case 1:
            printf("Button and LED driver. Type 'q' to exit.\n");
            printf("LED 4 will blink. Push a button to light one of the other LEDs.\n");
            led_blink(4, 500);
            do
            {
                button_get(1) ? led_on(1) : led_off(1);
                button_get(2) ? led_on(2) : led_off(2);
                button_get(3) ? led_on(3) : led_off(3);
                _delay_ms(200);
            } while (!_quit());
            led_off(1);
            led_off(2);
            led_off(3);
            led_off(4);
            break;

        case 2:
            _pir_active = true;
            printf("PIR sensor driver. Type 'q' to exit.\n");
            printf("LED 1 should turn on when motion is detected.\n");
            do
            {
                _delay_ms(200);
            } while (!_quit());
            _pir_active = false;
            led_off(1);
            break;

        case 3:
            printf("Display driver. Type 'q' to exit.\n");
            printf("Type a number between -999 and 9999:\n");
            while (scanf("%d", &x) == 1)
            {
                display_setDecimals(1);
                display_int(x);
                printf("You wrote: %d\n", x);
            }
            scanf("%*s");
            break;

        case 4:
        {
            WIFI_ERROR_MESSAGE_t message;

            printf("WiFi TCP demo.\n");
            printf("NOTE: ESP8266 only supports 2.4GHz networks!\n");
            _prompt_line("Enter WiFi SSID (max 27 chars): ", _line_buffer1, sizeof(_line_buffer1));
            _prompt_line("Enter WiFi password (max 27 chars): ", _line_buffer2, sizeof(_line_buffer2));

            if (!mqtt_prepare_wifi())
                break;

            printf("Connecting to WiFi (up to %d attempts)...\n", WIFI_MAX_RETRIES);
            if (wifi_command_join_AP(_line_buffer1, _line_buffer2) != WIFI_OK)
            {
                printf("Failed to join WiFi network after %d attempts.\n", WIFI_MAX_RETRIES);
                break;
            }
            printf("Successfully joined WiFi network.\n");

            _prompt_line("Enter IP address of TCP server: ", _line_buffer1, sizeof(_line_buffer1));

            _tcp_rx_buffer[0] = '\0';
            message = wifi_command_create_TCP_connection(_line_buffer1,
                                                         23,
                                                         wifi_line_callback,
                                                         _tcp_rx_buffer);
            if (message != WIFI_OK)
            {
                printf("Failed to create TCP connection. Error=%d\n", message);
                break;
            }

            printf("Successfully created TCP connection. Type 'q' to stop.\n");
            wifi_command_TCP_transmit((uint8_t *)welcome_text, strlen(welcome_text));

            while (1)
            {
                if (_tcp_string_received)
                {
                    printf("TCP received: %s\n", _tcp_rx_buffer);
                    _tcp_rx_buffer[0] = '\0';
                    _tcp_string_received = false;
                }

                if (gets_nonblocking(_line_buffer2, sizeof(_line_buffer2)) > 0)
                {
                    _line_buffer2[strcspn(_line_buffer2, "\r\n")] = '\0';
                    if (_line_buffer2[0] == 'r' && _line_buffer2[1] == '\0')
                    {
                        printf("Attempting to reconnect...\n");
                        if (wifi_reconnect() == WIFI_OK)
                            printf("Reconnected successfully.\n");
                        else
                            printf("Reconnect failed.\n");
                    }
                    else if (_line_buffer2[0] == 'q' && _line_buffer2[1] == '\0')
                    {
                        wifi_command_close_TCP_connection();
                        break;
                    }
                    else
                    {
                        printf("You wrote: %s\n", _line_buffer2);
                        wifi_command_TCP_transmit((uint8_t *)_line_buffer2,
                                                  strlen(_line_buffer2));
                    }
                }
                _delay_ms(200);
            }
            break;
        }

        case 5:
        {
            int ch;
            printf("stdio driver. Type a text to echo to the terminal.\n");
            do
            {
                ch = getchar();
                if (ch != EOF)
                    putchar(ch);
            } while (ch != '\n' && ch != EOF);
            break;
        }

        case 6:
            printf("Timer driver demo. Type 'q' to exit.\n");
            printf("LED 2 will toggle every 100ms. Press button 2 to pause/resume.\n");
            led2_timer_id = timer_create_sw(led2_callback, 100);
            if (led2_timer_id < 0)
            {
                printf("Timer create failed.\n");
                _delay_ms(2000);
                break;
            }
            do
            {
                if (button_get(2))
                {
                    start_stop_timer((uint8_t)led2_timer_id);
                    _delay_ms(300);
                }
            } while (!_quit());
            led_off(2);
            timer_delete((uint8_t)led2_timer_id);
            break;

        case 7:
            printf("Buzzer driver demo. Press button 2 to hear a beep. Type 'q' to exit.\n");
            do
            {
                if (button_get(2))
                {
                    buzzer_beep();
                    _delay_ms(200);
                }
            } while (!_quit());
            break;

        case 8:
            printf("DHT11 driver demo. Type 'q' to exit.\n");
            printf("Temperature and humidity will be printed every 2 seconds.\n");
            do
            {
                uint8_t humidity_integer, humidity_decimal;
                uint8_t temperature_integer, temperature_decimal;
                DHT11_ERROR_MESSAGE_t error = dht11_get(
                    &humidity_integer, &humidity_decimal,
                    &temperature_integer, &temperature_decimal);
                if (error == DHT11_OK)
                {
                    printf("Temperature: %u.%u C, Humidity: %u.%u %%\n",
                           temperature_integer, temperature_decimal,
                           humidity_integer, humidity_decimal);
                }
                else
                {
                    printf("Failed to read DHT11 sensor data.\n");
                }
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 9:
            printf("Proximity sensor driver demo. Type 'q' to exit.\n");
            printf("Distance in mm will be printed every 2 seconds.\n");
            do
            {
                uint16_t distance = proximity_measure();
                if (distance == UINT16_MAX)
                    printf("Proximity timeout. No object detected within range.\n");
                else
                    printf("Distance: %u mm\n", distance);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 10:
        {
            int angle;
            printf("Servo motor driver demo. Enter angle (-90 to 90).\n");
            servo_start();
            while (scanf("%d", &angle) == 1)
            {
                if (angle < -90 || angle > 90)
                    printf("Invalid angle. Please enter a value between -90 and 90.\n");
                else
                {
                    servo_setAngle(PWM_A, (int8_t)angle);
                    servo_setAngle(PWM_B, (int8_t)angle);
                    printf("Servo set to %d degrees.\n", angle);
                }
            }
            servo_stop();
            scanf("%*s");
            break;
        }

        case 11:
            printf("Light sensor driver demo. Type 'q' to exit.\n");
            printf("Light level will be printed every 2 seconds.\n");
            do
            {
                uint16_t light_level = light_measure_raw();
                if (light_level == UINT16_MAX)
                    printf("Failed to read light sensor.\n");
                else
                    printf("Light level: %u (0-1023)\n", light_level);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 12:
            printf("Soil moisture sensor (raw). Type 'q' to exit.\n");
            printf("Reading from PK0 every 2 seconds.\n");
            do
            {
                uint16_t soil_moisture = soil_measure_raw(ADC_PK0);
                if (soil_moisture == UINT16_MAX)
                    printf("Failed to read soil moisture sensor.\n");
                else
                    printf("Soil moisture (raw): %u (0-1023)\n", soil_moisture);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 13:
            printf("Soil moisture sensor (percentage). Type 'q' to exit.\n");
            printf("Reading from PK0 every 2 seconds.\n");
            do
            {
                uint8_t soil_pct = soil_measure_percentage(ADC_PK0);
                if (soil_pct == UINT8_MAX)
                    printf("Failed to read soil moisture sensor.\n");
                else
                    printf("Soil moisture: %u%%\n", soil_pct);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 14:
            printf("Playing Star Wars theme on the speaker. Reset to interrupt.\n");
            tone_play_starwars();
            break;

        case 15:
        {
            // int16_t ax, ay, az;
            // printf("ADXL345 Accelerometer demo. Type 'q' to exit.\n");
            // do
            // {
            //     adxl345_read_xyz(&ax, &ay, &az);
            //     printf("Acceleration - X: %d, Y: %d, Z: %d\n", ax, ay, az);
            //     _delay_ms(2000);
            // } while (!_quit());
            printf("Accelerometer not available.\n");
            break;
        }

        case 16:
        {
            uint8_t pump_choice;
            char pump_line[8];
            char *pump_end;

            printf("Pump control.\n");
            printf("1. Turn pump on\n");
            printf("2. Turn pump off\n");
            printf("3. Check if pump is running\n");
            _read_line_from_uart(pump_line, sizeof(pump_line));
            pump_choice = (uint8_t)strtol(pump_line, &pump_end, 10);

            switch (pump_choice)
            {
            case 1:
                pump_on();
                printf("Pump is now on.\n");
                break;
            case 2:
                pump_off();
                printf("Pump is now off.\n");
                break;
            case 3:
                printf("Pump is %s.\n", pump_is_running() ? "running" : "not running");
                break;
            default:
                printf("Invalid option.\n");
                break;
            }
            break;
        }

        case 17:
        {
            char sec_line[16];
            char *sec_end;
            uint32_t seconds;

            printf("Pump run for X seconds.\n");
            printf("Enter duration in seconds: ");
            _read_line_from_uart(sec_line, sizeof(sec_line));
            seconds = (uint32_t)strtoul(sec_line, &sec_end, 10);
            printf("Running pump for %lu seconds...\n", seconds);
            pump_run_for(seconds * 1000UL);
            printf("Pump done.\n");
            break;
        }

        case 18:
        {
            char payload[80];

            printf("MQTT demo\n");
            printf("WiFi SSID: %s\n", WIFI_DEFAULT_SSID);
            printf("Broker port: %u\n", MQTT_DEFAULT_BROKER_PORT);
            printf("Topic: %s\n", MQTT_DEFAULT_TOPIC);
            printf("Auth: anonymous (ensure allow_anonymous true in mosquitto.conf)\n\n");

            _prompt_line("Enter broker IP (your laptop's LAN IP, e.g. 192.168.x.x): ",
                         _broker_ip, sizeof(_broker_ip));
            if (_broker_ip[0] == '\0')
            {
                printf("No IP entered. Aborting.\n");
                break;
            }
            printf("Broker: %s:%u\n", _broker_ip, MQTT_DEFAULT_BROKER_PORT);
            printf("To subscribe, run:\n");
            printf("  mosquitto_sub -h %s -t '#'\n\n", _broker_ip);

            if (!mqtt_prepare_wifi())
                break;

            if (wifi_command_join_AP(WIFI_DEFAULT_SSID, WIFI_DEFAULT_PASSWORD) != WIFI_OK)
            {
                printf("WiFi join failed.\n");
                break;
            }
            printf("WiFi connected.\n");

            if (!mqtt_configure_and_connect(_broker_ip,
                                            MQTT_DEFAULT_BROKER_PORT,
                                            MQTT_DEFAULT_CLIENT_ID))
                break;

            printf("MQTT connected!\n");
            printf("1 = send custom text, 2 = send sensor snapshot\n");
            _prompt_line("Choose: ", _line_buffer1, sizeof(_line_buffer1));

            if (_line_buffer1[0] == '2')
            {
                mqtt_publish_sensor_snapshot();
            }
            else
            {
                _prompt_line("Message (no double quotes): ", payload, sizeof(payload));
                if (payload[0] == '\0')
                    strcpy(payload, "Hello from Arduino Mega 2560");
                mqtt_publish_text(MQTT_DEFAULT_TOPIC, payload);
            }
            break;
        }

        case 19:
        {
            printf("Checking ESP8266 firmware version...\n");
            wifi_command_AT();
            wifi_command_disable_echo();
            wifi_command("AT+GMR", 5);
            break;
        }

        default:
            printf("Error: Invalid selection.\n");
            break;
        }
    }

    return 0;
}