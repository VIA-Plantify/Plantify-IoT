#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "interactive.h"
#include "button.h"
#include "uart_stdio.h"
#include "led.h"
#include "pir.h"
#include "display.h"
#include "wifi.h"
#include "buzzer.h"
#include "dht11.h"
#include "proximity.h"
#include "servo.h"
#include "adc.h"
#include "light.h"
#include "soil.h"
#include "timer.h"

#define WIFI_SSID        "Ricky"
#define WIFI_PASSWORD    "r89uuios"
#define MQTT_BROKER_IP   "10.27.47.89"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "mega2560_client"
#define MQTT_PUB_TOPIC   "arduino"
#define MQTT_SUB_TOPIC   "iot/mega/commands"

uint8_t humidity_integer, humidity_decimal, temperature_integer, temperature_decimal;
static char _device_mac[18] = "00:00:00:00:00:00";  /* filled after WiFi connect */

/*---------------------------------------------------------------------------
 * Raw MQTT packet helpers
 * MQTT 3.1.1 over plain TCP
 *--------------------------------------------------------------------------*/

/*
  Build a MQTT CONNECT packet.
  Returns packet length.
 
  Packet layout:
    Byte 0:    0x10  (CONNECT fixed header)
    Byte 1:    remaining length
    Bytes 2-3: 0x00 0x04  (protocol name length)
    Bytes 4-7: "MQTT"
    Byte 8:    0x04  (protocol level = 3.1.1)
    Byte 9:    0x02  (connect flags: clean session)
    Bytes 10-11: 0x00 0x3C (keep-alive = 60s)
    Bytes 12-13: client ID length (2 bytes, big-endian)
    Bytes 14+:  client ID string
 */
static uint8_t mqtt_build_connect(uint8_t *buf, uint8_t buf_size,
                                  const char *client_id)
{
    uint8_t id_len = (uint8_t)strlen(client_id);
    uint8_t remaining = 10 + 2 + id_len;  /* variable header + client ID */

    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x10;            /* CONNECT */
    buf[i++] = remaining;
    buf[i++] = 0x00;            /* protocol name length MSB */
    buf[i++] = 0x04;            /* protocol name length LSB */
    buf[i++] = 'M';
    buf[i++] = 'Q';
    buf[i++] = 'T';
    buf[i++] = 'T';
    buf[i++] = 0x04;            /* protocol level 3.1.1 */
    buf[i++] = 0x02;            /* connect flags: clean session */
    buf[i++] = 0x00;            /* keep-alive MSB */
    buf[i++] = 0x3C;            /* keep-alive LSB = 60s */
    buf[i++] = 0x00;            /* client ID length MSB */
    buf[i++] = id_len;          /* client ID length LSB */
    memcpy(&buf[i], client_id, id_len);
    i += id_len;

    return i;
}

/*
 * Build a MQTT SUBSCRIBE packet for a single topic, QoS 0.
 * Returns packet length.
 */
static uint8_t mqtt_build_subscribe(uint8_t *buf, uint8_t buf_size,
                                    const char *topic, uint16_t packet_id)
{
    uint8_t topic_len = (uint8_t)strlen(topic);
    uint8_t remaining = 2 + 2 + topic_len + 1;  /* packet ID + topic len + topic + QoS */

    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x82;                        /* SUBSCRIBE */
    buf[i++] = remaining;
    buf[i++] = (uint8_t)(packet_id >> 8);   /* packet ID MSB */
    buf[i++] = (uint8_t)(packet_id & 0xFF); /* packet ID LSB */
    buf[i++] = 0x00;                        /* topic length MSB */
    buf[i++] = topic_len;                   /* topic length LSB */
    memcpy(&buf[i], topic, topic_len);
    i += topic_len;
    buf[i++] = 0x00;                        /* QoS 0 */

    return i;
}

/*
 * Build a MQTT PUBLISH packet, QoS 0, no retain.
 * Returns packet length.
 */
static uint8_t mqtt_build_publish(uint8_t *buf, uint8_t buf_size,
                                  const char *topic, const char *payload)
{
    uint8_t topic_len   = (uint8_t)strlen(topic);
    uint8_t payload_len = (uint8_t)strlen(payload);
    uint8_t remaining   = 2 + topic_len + payload_len;

    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x30;            /* PUBLISH, QoS 0, no retain */
    buf[i++] = remaining;
    buf[i++] = 0x00;            /* topic length MSB */
    buf[i++] = topic_len;       /* topic length LSB */
    memcpy(&buf[i], topic, topic_len);
    i += topic_len;
    memcpy(&buf[i], payload, payload_len);
    i += payload_len;

    return i;
}

/*
  Build a MQTT PING-request packet (2 bytes).
 */
static uint8_t mqtt_build_pingreq(uint8_t *buf)
{
    buf[0] = 0xC0;
    buf[1] = 0x00;
    return 2;
}

/*---------------------------------------------------------------------------
 * WiFi / MQTT connection state
 *--------------------------------------------------------------------------*/
static uint8_t _mqtt_connected = 0;
static uint16_t _seconds_since_ping = 0;

#define PING_INTERVAL_S 45   /* send PING-request every 45s, keep-alive is 60s */

/*
  Receive buffer for incoming TCP data (subscribed messages).
  The wifi driver writes here via the callback.
 */
static char _rx_buf[126] = {0};
static volatile uint8_t _rx_ready = 0;

static void tcp_rx_callback(void)
{
    _rx_ready = 1;
}

/*---------------------------------------------------------------------------
 * Connect WiFi -> open TCP -> send MQTT CONNECT -> SUBSCRIBE
 *--------------------------------------------------------------------------*/
static uint8_t mqtt_raw_connect(void)
{
    uint8_t pkt[64];
    uint8_t pkt_len;

    _mqtt_connected = 0;

    printf("Starting WiFi...\n");
    if (wifi_command_AT() != WIFI_OK)
    {
        printf("ESP not responding.\n");
        return 0;
    }
    wifi_command_disable_echo();

    /* Read and store the station MAC address for use in MQTT payloads */
    if (wifi_command_get_mac(_device_mac) == WIFI_OK)
        printf("Device MAC: %s\n", _device_mac);
    else
        printf("MAC read failed, using default.\n");

    wifi_command("AT+CIFSR", 5); /* get and print local IP and MAC address */

    if (wifi_command_set_mode_to_1() != WIFI_OK)
    {
        printf("Station mode failed.\n");
        return 0;
    }
    _delay_ms(500);

    printf("Joining WiFi...\n");
    if (wifi_command_join_AP(WIFI_SSID, WIFI_PASSWORD) != WIFI_OK)
    {
        printf("WiFi join failed.\n");
        return 0;
    }
    _delay_ms(2000);

    printf("Opening TCP to %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    _rx_buf[0] = '\0';
    if (wifi_command_create_TCP_connection(MQTT_BROKER_IP,
                                           MQTT_BROKER_PORT,
                                           tcp_rx_callback,
                                           _rx_buf) != WIFI_OK)
    {
        printf("TCP connection failed.\n");
        return 0;
    }
    _delay_ms(500);

    /* Send MQTT CONNECT */
    pkt_len = mqtt_build_connect(pkt, sizeof(pkt), MQTT_CLIENT_ID);
    if (pkt_len == 0)
    {
        printf("CONNECT build failed.\n");
        return 0;
    }
    printf("Sending MQTT CONNECT...\n");
    wifi_command_TCP_transmit(pkt, pkt_len);
    _delay_ms(500);

    /* Send MQTT SUBSCRIBE */
    pkt_len = mqtt_build_subscribe(pkt, sizeof(pkt), MQTT_SUB_TOPIC, 1);
    if (pkt_len == 0)
    {
        printf("SUBSCRIBE build failed.\n");
        return 0;
    }
    printf("Sending MQTT SUBSCRIBE to %s...\n", MQTT_SUB_TOPIC);
    wifi_command_TCP_transmit(pkt, pkt_len);
    _delay_ms(300);

    printf("MQTT ready.\n");
    _mqtt_connected = 1;
    _seconds_since_ping = 0;
    return 1;
}

/*---------------------------------------------------------------------------
 * Publish payload
 *--------------------------------------------------------------------------*/
static uint8_t mqtt_raw_publish(const char *payload)
{
    uint8_t pkt[198];
    uint8_t pkt_len = mqtt_build_publish(pkt, sizeof(pkt), MQTT_PUB_TOPIC, payload);
    if (pkt_len == 0)
    {
        printf("PUBLISH build failed (payload too long?).\n");
        return 0;
    }
    if (wifi_command_TCP_transmit(pkt, pkt_len) != WIFI_OK)
    {
        printf("TCP transmit failed.\n");
        _mqtt_connected = 0;
        return 0;
    }
    return 1;
}

/*---------------------------------------------------------------------------
 * Handle any incoming subscribed message
 *--------------------------------------------------------------------------*/
static void mqtt_handle_incoming(void)
{
    if (!_rx_ready)
        return;

    _rx_ready = 0;

    /*
      _rx_buf contains raw MQTT bytes from the broker.
      PUBLISH packet starts with 0x30. We extract the payload after
      the variable header (fixed header + remaining length + topic length).
      For simplicity we just print the raw buffer as a hex + ascii dump
      and also try to extract a printable payload.
     */
    uint8_t *data = (uint8_t *)_rx_buf;

    if (data[0] == 0x30)
    {
        /* PUBLISH packet received */
        uint8_t remaining = data[1];
        uint8_t topic_len = (data[2] << 8) | data[3];

        if (topic_len < remaining)
        {
            uint8_t payload_offset = 2 + 2 + topic_len;
            uint8_t payload_len    = remaining - 2 - topic_len;

            char payload_str[64] = {0};
            if (payload_len >= sizeof(payload_str))
                payload_len = sizeof(payload_str) - 1;

            memcpy(payload_str, &_rx_buf[payload_offset], payload_len);
            payload_str[payload_len] = '\0';

            printf("CMD received: %s\n", payload_str);

            /* Simple command handling */
            if (strcmp(payload_str, "led_on") == 0)
                led_on(1);
            else if (strcmp(payload_str, "led_off") == 0)
                led_off(1);
            else if (strcmp(payload_str, "beep") == 0)
                buzzer_beep();
        }
    }
    else if (data[0] == 0x20)
    {
        printf("Connection received (broker accepted connection).\n");
    }
    else if (data[0] == 0x90)
    {
        printf("Subscriber connction received (subscribed OK).\n");
    }
    else if (data[0] == 0xD0)
    {
        /* PINGRESP */
        printf("PINGRESP received.\n");
    }

    _rx_buf[0] = '\0';
}

/*---------------------------------------------------------------------------
 * main
 *--------------------------------------------------------------------------*/
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
        while (1) { }
    }

    sei();
    printf("SEP4 IoT Hardware - Raw MQTT Mode\n");

    /* Hold button 2 at boot to enter interactive demo instead */
    if (button_get(2))
    {
        interactive_demo();  /* never returns */
    }

    mqtt_raw_connect();

    while (1)
    {
        uint16_t light_value;
        uint16_t soil_value;
        uint16_t distance_mm;
        uint8_t  motion;
        char     payload[126];

        /* Read sensors */
        dht11_get(&humidity_integer, &humidity_decimal,
                  &temperature_integer, &temperature_decimal);
        light_value  = light_measure_raw();
        soil_value   = soil_measure_raw(ADC_PK0);
        distance_mm  = proximity_measure();
        motion       = (pir_get_state() != PIR_NO_MOTION) ? 1 : 0;

        /* Print to serial */
        printf("T:%u.%uC H:%u.%u%% L:%u S:%u D:%u M:%u\n",
               temperature_integer, temperature_decimal,
               humidity_integer, humidity_decimal,
               light_value, soil_value, distance_mm, motion);

        /* Show temperature on display */
        display_int((temperature_integer * 10) + temperature_decimal);

        /* Handle any incoming subscribed message */
        mqtt_handle_incoming();

        /* Publish or reconnect */
        if (_mqtt_connected)
        {
            snprintf(payload, sizeof(payload),
            "{\"mac\":\"%s\",\"temp\":%u.%u,\"hum\":%u.%u,\"light\":%u,\"soil\":%u,\"dist\":%u,\"motion\":%u}",
            _device_mac,
            temperature_integer, temperature_decimal,
            humidity_integer, humidity_decimal,
            light_value, soil_value, distance_mm, motion);

            if (!mqtt_raw_publish(payload))
            {
                printf("Publish failed. Reconnecting...\n");
                wifi_command_close_TCP_connection();
                _delay_ms(1000);
                mqtt_raw_connect();
            }

            /* Keep-alive ping */
            _seconds_since_ping += 3;  /* loop runs every ~3s */
            if (_seconds_since_ping >= PING_INTERVAL_S)
            {
                uint8_t ping[2];
                mqtt_build_pingreq(ping);
                wifi_command_TCP_transmit(ping, 2);
                _seconds_since_ping = 0;
            }
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