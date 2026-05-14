/****************************************************************************
 * interactive.c
 ****************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
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

#define MAX_STRING_LENGTH 100
#define MAX_MENU_OPTIONS 15

#define MQTT_DEFAULT_BROKER_PORT 1883
#define MQTT_DEFAULT_CLIENT_ID "mega2560_client"
#define MQTT_DEFAULT_TOPIC "iot/mega/sensors"

#define WIFI_DEFAULT_SSID "Ricky"
#define WIFI_DEFAULT_PASSWORD "r89uuios"

static int x = 0;
static char _q_buff[5] = {0};
static volatile bool _tcp_string_received = false;
static volatile bool _pir_active = false;

static char _tcp_rx_buffer[MAX_STRING_LENGTH] = {0};
static char _line_buffer1[MAX_STRING_LENGTH] = {0};
static char _line_buffer2[MAX_STRING_LENGTH] = {0};

static char _broker_ip[MAX_STRING_LENGTH] = {0};
static char _mqtt_topic[MAX_STRING_LENGTH] = MQTT_DEFAULT_TOPIC;

static const char welcome_text[] = "Welcome from SEP4 IoT hardware!\n";

/* -------------------------------------------------------------------------
 * UART helpers
 * ------------------------------------------------------------------------- */

/*
 * Poll with a small delay so the UART RX interrupt has time to fill the
 * ring-buffer between checks.  Without the delay the AVR spins so fast
 * that gets_nonblocking() always returns 0.
 */
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
        _delay_ms(10); /* <-- KEY FIX: yield to UART ISR */
    }
}

static bool _quit(void)
{
    _delay_ms(10); /* same reason as above */
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
        printf("MQTT command too long (%u chars).\n", (unsigned)len);
        return WIFI_FAIL;
    }
    return wifi_command(cmd, timeout_s);
}

static uint8_t mqtt_prepare_wifi(void)
{
    if (wifi_command_AT() != WIFI_OK)
    {
        printf("ESP8266 not responding.\n");
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
             "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"", client_id);
    printf("Sending: %s\n", cmd);
    if (mqtt_send_command(cmd, 5) != WIFI_OK)
    {
        printf("MQTT client config failed.\n");
        return 0;
    }

    snprintf(cmd, sizeof(cmd),
             "AT+MQTTCONN=0,\"%s\",%u,0", broker_ip, broker_port);
    printf("Sending: %s\n", cmd);
    if (mqtt_send_command(cmd, 15) != WIFI_OK)
    {
        printf("MQTT broker connection failed.\n");
        printf("Check: IP correct? mosquitto running? allow_anonymous true?\n");
        return 0;
    }
    return 1;
}

static uint8_t mqtt_publish_text(const char *topic, const char *payload)
{
    char cmd[160];

    if (strchr(payload, '"') != NULL)
    {
        printf("Payload contains double quotes — use plain text.\n");
        return 0;
    }
    snprintf(cmd, sizeof(cmd),
             "AT+MQTTPUB=0,\"%s\",\"%s\",0,0", topic, payload);
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
    uint8_t hi = 0, hd = 0, ti = 0, td = 0;
    char payload[96];

    if (dht11_get(&hi, &hd, &ti, &td) != DHT11_OK)
    {
        printf("DHT11 read failed.\n");
        return;
    }

    snprintf(payload, sizeof(payload),
             "temp=%u.%u,hum=%u.%u,light=%u,soil=%u,dist=%u,motion=%u",
             ti, td, hi, hd,
             light_measure_raw(),
             soil_measure_raw(ADC_PK0),
             proximity_measure(),
             (pir_get_state() != PIR_NO_MOTION) ? 1U : 0U);

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

    printf("\n\t----------------- M E N U ------------------\n");
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
    printf("\t12. Soil Moisture Sensor (capacitive)\n");
    printf("\t13. Play Star Wars theme on the speaker\n");
    printf("\t14. Publish a message to MQTT broker\n");
    printf("\t15. Check ESP8266 firmware version\n");

    do
    {
        printf("Choose a driver to test (1-%d): ", MAX_MENU_OPTIONS);
        _read_line_from_uart(line, sizeof(line));

        choice = (int)strtol(line, &endptr, 10);

        if (line[0] == '\0' || endptr == line || *endptr != '\0' ||
            choice < 1 || choice > MAX_MENU_OPTIONS)
        {
            printf("Invalid input. Enter a number 1-%d.\n", MAX_MENU_OPTIONS);
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

    printf("\n--- Interactive demo started. ---\n");

    while (1)
    {
        switch (menu())
        {
        case 1:
            printf("Button and LED. Type 'q' to exit.\n");
            printf("LED 4 blinks. Push buttons 1-3 to light LEDs 1-3.\n");
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
            printf("PIR sensor. Type 'q' to exit.\n");
            printf("LED 1 lights when motion detected.\n");
            do
            {
                _delay_ms(200);
            } while (!_quit());
            _pir_active = false;
            led_off(1);
            break;

        case 3:
            printf("Display driver. Enter a number (-999 to 9999). Non-number exits.\n");
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
            printf("WiFi TCP demo (port 23). Type 'q' to exit.\n");
            _prompt_line("Enter WiFi SSID: ", _line_buffer1, sizeof(_line_buffer1));
            _prompt_line("Enter WiFi password: ", _line_buffer2, sizeof(_line_buffer2));

            if (!mqtt_prepare_wifi())
                break;
            if (wifi_command_join_AP(_line_buffer1, _line_buffer2) != WIFI_OK)
            {
                printf("Failed to join WiFi.\n");
                break;
            }
            printf("WiFi joined.\n");
            _prompt_line("Enter TCP server IP: ", _line_buffer1, sizeof(_line_buffer1));

            _tcp_rx_buffer[0] = '\0';
            message = wifi_command_create_TCP_connection(
                _line_buffer1, 23,
                wifi_line_callback, _tcp_rx_buffer);
            if (message != WIFI_OK)
            {
                printf("TCP connection failed. Error=%d\n", message);
                break;
            }
            printf("TCP connected. Type 'q' to stop.\n");
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
                    if (_line_buffer2[0] == 'q' && _line_buffer2[1] == '\0')
                    {
                        wifi_command_close_TCP_connection();
                        break;
                    }
                    printf("You wrote: %s\n", _line_buffer2);
                    wifi_command_TCP_transmit((uint8_t *)_line_buffer2,
                                              strlen(_line_buffer2));
                }
                _delay_ms(200);
            }
            break;
        }

        case 5:
        {
            int ch;
            printf("stdio echo. Type a line.\n");
            do
            {
                ch = getchar();
                if (ch != EOF)
                    putchar(ch);
            } while (ch != '\n' && ch != EOF);
            break;
        }

        case 6:
            printf("Timer demo. 'q' exits. Button 2 pauses/resumes.\n");
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
            printf("Buzzer demo. Button 2 beeps. 'q' exits.\n");
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
            printf("DHT11 demo. 'q' exits.\n");
            do
            {
                uint8_t hi, hd, ti, td;
                if (dht11_get(&hi, &hd, &ti, &td) == DHT11_OK)
                    printf("Temp: %u.%u C  Humidity: %u.%u %%\n", ti, td, hi, hd);
                else
                    printf("DHT11 read failed.\n");
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 9:
            printf("Proximity demo. 'q' exits.\n");
            do
            {
                uint16_t d = proximity_measure();
                if (d == UINT16_MAX)
                    printf("No object in range.\n");
                else
                    printf("Distance: %u mm\n", d);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 10:
        {
            int angle;
            printf("Servo demo. Enter angle -90 to 90. Non-number exits.\n");
            servo_start();
            while (scanf("%d", &angle) == 1)
            {
                if (angle < -90 || angle > 90)
                    printf("Out of range.\n");
                else
                {
                    servo_setAngle(PWM_A, (int8_t)angle);
                    servo_setAngle(PWM_B, (int8_t)angle);
                    printf("Servo -> %d deg\n", angle);
                }
            }
            servo_stop();
            scanf("%*s");
            break;
        }

        case 11:
            printf("Light sensor demo. 'q' exits.\n");
            do
            {
                uint16_t lv = light_measure_raw();
                if (lv == UINT16_MAX)
                    printf("Light sensor read failed.\n");
                else
                    printf("Light: %u / 1023\n", lv);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 12:
            printf("Soil moisture demo. 'q' exits.\n");
            do
            {
                uint16_t sv = soil_measure_raw(ADC_PK0);
                if (sv == UINT16_MAX)
                    printf("Soil sensor read failed.\n");
                else
                    printf("Soil: %u / 1023\n", sv);
                _delay_ms(2000);
            } while (!_quit());
            break;

        case 13:
            printf("Playing Star Wars theme. Reset to interrupt.\n");
            tone_play_startup();
            _delay_ms(2000);
            tone_play_air_raid();
            _delay_ms(2000);
            tone_play_smoke_detector();
            break;

        case 14:
        {
            char payload[80];
            printf("MQTT demo | SSID: %s | Port: %u | Topic: %s\n",
                   WIFI_DEFAULT_SSID, MQTT_DEFAULT_BROKER_PORT, MQTT_DEFAULT_TOPIC);
            printf("Requires: allow_anonymous true in mosquitto.conf\n\n");

            _prompt_line("Broker IP (e.g. 192.168.x.x): ", _broker_ip, sizeof(_broker_ip));
            if (_broker_ip[0] == '\0')
            {
                printf("No IP. Aborting.\n");
                break;
            }

            printf("To subscribe: mosquitto_sub -h %s -t '#'\n\n", _broker_ip);

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

            printf("MQTT connected!\n1 = custom text  2 = sensor snapshot\n");
            _prompt_line("Choose: ", _line_buffer1, sizeof(_line_buffer1));

            if (_line_buffer1[0] == '2')
                mqtt_publish_sensor_snapshot();
            else
            {
                _prompt_line("Message (no double quotes): ", payload, sizeof(payload));
                if (payload[0] == '\0')
                    strcpy(payload, "Hello from Arduino Mega 2560");
                mqtt_publish_text(MQTT_DEFAULT_TOPIC, payload);
            }
            break;
        }

        case 15:
            printf("ESP8266 firmware version:\n");
            wifi_command_AT();
            wifi_command_disable_echo();
            wifi_command("AT+GMR", 5);
            break;

        default:
            printf("Invalid selection.\n");
            break;
        }
    }

    return 0;
}