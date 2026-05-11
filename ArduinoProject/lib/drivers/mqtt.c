#include "mqtt.h"
#include "wifi.h"
#include "led.h"
#include "buzzer.h"
#include "pump.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ─── Internal State ───────────────────────────────────────────────────────────
static uint8_t  _mqtt_connected      = 0;
static uint16_t _seconds_since_ping  = 0;
static char     _rx_buf[128]         = {0};
static volatile uint8_t _rx_ready   = 0;
char     _device_mac[18]      = "84:f3:eb:95:b4:b3";

// ─── RX Callback ─────────────────────────────────────────────────────────────
static void tcp_rx_callback(void)
{
    _rx_ready = 1;
}

// ─── Packet Builders ─────────────────────────────────────────────────────────
static uint8_t mqtt_build_connect(uint8_t *buf, uint8_t buf_size,
                                   const char *client_id,
                                   const char *username,
                                   const char *password)
{
    uint8_t id_len   = (uint8_t)strlen(client_id);
    uint8_t user_len = (uint8_t)strlen(username);
    uint8_t pass_len = (uint8_t)strlen(password);
    uint8_t remaining = 10 + 2 + id_len + 2 + user_len + 2 + pass_len;

    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x10;
    buf[i++] = remaining;
    buf[i++] = 0x00;
    buf[i++] = 0x04;
    buf[i++] = 'M';
    buf[i++] = 'Q';
    buf[i++] = 'T';
    buf[i++] = 'T';
    buf[i++] = 0x04; // protocol level 3.1.1
    buf[i++] = 0xC2; // connect flags: username + password + clean session
    buf[i++] = 0x00; // keep-alive MSB
    buf[i++] = 0x3C; // keep-alive 60s
    buf[i++] = 0x00;
    buf[i++] = id_len;
    memcpy(&buf[i], client_id, id_len); i += id_len;
    buf[i++] = 0x00;
    buf[i++] = user_len;
    memcpy(&buf[i], username, user_len); i += user_len;
    buf[i++] = 0x00;
    buf[i++] = pass_len;
    memcpy(&buf[i], password, pass_len); i += pass_len;

    return i;
}

static uint8_t mqtt_build_subscribe(uint8_t *buf, uint8_t buf_size,
                                     const char *topic, uint16_t packet_id)
{
    uint8_t topic_len = (uint8_t)strlen(topic);
    uint8_t remaining = 2 + 2 + topic_len + 1;
    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x82;
    buf[i++] = remaining;
    buf[i++] = (uint8_t)(packet_id >> 8);
    buf[i++] = (uint8_t)(packet_id & 0xFF);
    buf[i++] = 0x00;
    buf[i++] = topic_len;
    memcpy(&buf[i], topic, topic_len); i += topic_len;
    buf[i++] = 0x00;
    return i;
}

static uint8_t mqtt_build_publish(uint8_t *buf, uint8_t buf_size,
                                   const char *topic, const char *payload)
{
    uint8_t topic_len   = (uint8_t)strlen(topic);
    uint8_t payload_len = (uint8_t)strlen(payload);
    uint8_t remaining   = 2 + topic_len + payload_len;
    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x30;
    buf[i++] = remaining;
    buf[i++] = 0x00;
    buf[i++] = topic_len;
    memcpy(&buf[i], topic, topic_len); i += topic_len;
    memcpy(&buf[i], payload, payload_len); i += payload_len;
    return i;
}

static uint8_t mqtt_build_pingreq(uint8_t *buf)
{
    buf[0] = 0xC0;
    buf[1] = 0x00;
    return 2;
}

// ─── Public API ───────────────────────────────────────────────────────────────
uint8_t mqtt_raw_connect(void)
{
    uint8_t pkt[128];
    uint8_t pkt_len;

    _mqtt_connected = 0;

    printf("Starting WiFi...\n");
    if (wifi_command_AT() != WIFI_OK)
    {
        printf("ESP not responding.\n");
        return 0;
    }
    wifi_command_disable_echo();

    if (wifi_command_get_mac(_device_mac) == WIFI_OK)
        printf("Device MAC: %s\n", _device_mac);
    else
        printf("MAC read failed, using default.\n");

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

    printf("Opening SSL to HiveMQ...\n");
    _rx_buf[0] = '\0';
    if (wifi_command_create_SSL_connection(MQTT_BROKER_HOST,
                                            MQTT_BROKER_PORT,
                                            tcp_rx_callback,
                                            _rx_buf) != WIFI_OK)
    {
        printf("SSL connection failed.\n");
        return 0;
    }
    _delay_ms(500);

    pkt_len = mqtt_build_connect(pkt, sizeof(pkt), MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    if (pkt_len == 0)
    {
        printf("CONNECT build failed.\n");
        return 0;
    }
    printf("Sending MQTT CONNECT...\n");
    wifi_command_TCP_transmit(pkt, pkt_len);
    _delay_ms(500);

    pkt_len = mqtt_build_subscribe(pkt, sizeof(pkt), MQTT_SUB_TOPIC, 1);
    if (pkt_len == 0)
    {
        printf("SUBSCRIBE build failed.\n");
        return 0;
    }
    printf("Subscribing to %s...\n", MQTT_SUB_TOPIC);
    wifi_command_TCP_transmit(pkt, pkt_len);
    _delay_ms(300);

    printf("MQTT ready.\n");
    _mqtt_connected = 1;
    _seconds_since_ping = 0;
    return 1;
}

uint8_t mqtt_raw_publish(const char *payload)
{
    uint8_t pkt[198];
    uint8_t pkt_len = mqtt_build_publish(pkt, sizeof(pkt), MQTT_PUB_TOPIC, payload);
    if (pkt_len == 0)
    {
        printf("PUBLISH build failed.\n");
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

void mqtt_handle_incoming(void)
{
    if (!_rx_ready)
        return;
    _rx_ready = 0;

    uint8_t *data = (uint8_t *)_rx_buf;

    if (data[0] == 0x30)
    {
        uint8_t remaining   = data[1];
        uint8_t topic_len   = (data[2] << 8) | data[3];
        if (topic_len < remaining)
        {
            uint8_t payload_offset = 2 + 2 + topic_len;
            uint8_t payload_len    = remaining - 2 - topic_len;
            char payload_str[64]   = {0};
            if (payload_len >= sizeof(payload_str))
                payload_len = sizeof(payload_str) - 1;
            memcpy(payload_str, &_rx_buf[payload_offset], payload_len);
            payload_str[payload_len] = '\0';
            printf("CMD received: %s\n", payload_str);

            if (strcmp(payload_str, "led_on") == 0)
                led_on(1);
            else if (strcmp(payload_str, "led_off") == 0)
                led_off(1);
            else if (strcmp(payload_str, "beep") == 0)
                buzzer_beep();
            else if (strncmp(payload_str, "pump_on_", 8) == 0)
            {
                uint8_t duration = (uint8_t)atoi(&payload_str[8]);
                pump_run_for(duration * 1000);
            }
        }
    }
    else if (data[0] == 0x20)
        printf("CONNACK received.\n");
    else if (data[0] == 0x90)
        printf("SUBACK received.\n");
    else if (data[0] == 0xD0)
        printf("PINGRESP received.\n");

    _rx_buf[0] = '\0';
}

void mqtt_send_ping(void)
{
    uint8_t ping[2];
    mqtt_build_pingreq(ping);
    wifi_command_TCP_transmit(ping, 2);
}

uint8_t mqtt_is_connected(void)
{
    return _mqtt_connected;
}

void mqtt_tick(uint8_t elapsed_seconds)
{
    _seconds_since_ping += elapsed_seconds;
    if (_seconds_since_ping >= PING_INTERVAL_S)
    {
        mqtt_send_ping();
        _seconds_since_ping = 0;
    }
}