#include "mqtt.h"
#include "eeprom_storage.h"
#include "wifi.h"
#include "led.h"
#include "tone.h"
#include "pump.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint8_t _mqtt_connected = 0;
static uint16_t _seconds_since_ping = 0;

static char _rx_buf[MQTT_RX_BUF_SIZE] = {0};
static volatile uint8_t _rx_ready = 0;

static char _device_mac[18] = {0};
static char _pub_topic[64] = {0};
static char _sub_topic[64] = {0};

char *mqtt_get_device_mac(void) { return _device_mac; }

static void tcp_rx_callback(void)
{
    _rx_ready = 1;
}

/* ------------------------------------------------------------------ */
/*  MQTT packet builders                                               */
/* ------------------------------------------------------------------ */
static uint8_t mqtt_build_connect(uint8_t *buf, uint8_t buf_size,
                                  const char *client_id,
                                  const char *username,
                                  const char *password)
{
    uint8_t id_len = (uint8_t)strlen(client_id);
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
    buf[i++] = 0x04;
    buf[i++] = 0xC2;
    buf[i++] = 0x00;
    buf[i++] = 0x3C;
    buf[i++] = 0x00;
    buf[i++] = id_len;
    memcpy(&buf[i], client_id, id_len);
    i += id_len;
    buf[i++] = 0x00;
    buf[i++] = user_len;
    memcpy(&buf[i], username, user_len);
    i += user_len;
    buf[i++] = 0x00;
    buf[i++] = pass_len;
    memcpy(&buf[i], password, pass_len);
    i += pass_len;
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
    memcpy(&buf[i], topic, topic_len);
    i += topic_len;
    buf[i++] = 0x00;
    return i;
}

static uint8_t mqtt_build_publish(uint8_t *buf, uint8_t buf_size,
                                  const char *topic, const char *payload)
{
    uint8_t topic_len = (uint8_t)strlen(topic);
    uint16_t payload_len = (uint16_t)strlen(payload);
    uint16_t remaining = 2 + topic_len + payload_len;

    /* MQTT variable-length remaining field:
       0-127 = 1 byte, 128+ = 2 bytes with continuation bit */
    uint8_t rem_bytes = (remaining < 128) ? 1 : 2;
    if ((uint16_t)(1 + rem_bytes + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x30;

    if (remaining < 128)
    {
        buf[i++] = (uint8_t)remaining;
    }
    else
    {
        buf[i++] = (uint8_t)((remaining & 0x7F) | 0x80);
        buf[i++] = (uint8_t)(remaining >> 7);
    }

    buf[i++] = 0x00;
    buf[i++] = topic_len;
    memcpy(&buf[i], topic, topic_len);
    i += topic_len;
    memcpy(&buf[i], payload, payload_len);
    i += payload_len;
    return i;
}

static uint8_t mqtt_build_pingreq(uint8_t *buf)
{
    buf[0] = 0xC0;
    buf[1] = 0x00;
    return 2;
}

/* ------------------------------------------------------------------ */
/*  Connect                                                             */
/* ------------------------------------------------------------------ */
uint8_t mqtt_raw_connect(void)
{
    uint8_t pkt[128];
    uint8_t pkt_len;
    _mqtt_connected = 0;

    _delay_ms(2000);

    printf("Starting WiFi...\n");
    uint8_t at_ok = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (wifi_command_AT() == WIFI_OK)
        {
            at_ok = 1;
            break;
        }
        printf("AT retry %d\n", i + 1);
        _delay_ms(1000);
    }
    if (!at_ok)
    {
        printf("ESP not responding.\n");
        return 0;
    }
    printf("ESP OK\n");

    wifi_command_disable_echo();
    wifi_command_set_mode_to_1();
    _delay_ms(500);

    /* Load WiFi credentials from EEPROM, fall back to hardcoded */
    char _ssid[32] = {0};
    char _pass[64] = {0};
    load_credentials(_ssid, _pass);
    if (_ssid[0] == '\0' || (uint8_t)_ssid[0] == 0xFF)
    {
        strncpy(_ssid, WIFI_SSID, sizeof(_ssid) - 1);
        strncpy(_pass, WIFI_PASSWORD, sizeof(_pass) - 1);
        printf("Using hardcoded WiFi credentials.\n");
    }
    else
    {
        printf("Using saved WiFi credentials.\n");
    }

    printf("Joining WiFi...\n");
    if (wifi_command_join_AP(_ssid, _pass) != WIFI_OK)
    {
        printf("WiFi join failed.\n");
        return 0;
    }
    _delay_ms(5000);

    /*  Play tone to signal WiFi connected (from ConnectWifi branch)  */
    tone_play_wifi_connected();

    /*  MAC fetch is mandatory — topics are built from it.
        Retry once if the first attempt fails.                         */
    _device_mac[0] = '\0';
    if (wifi_command_get_mac(_device_mac) != WIFI_OK || _device_mac[0] == '\0')
    {
        _delay_ms(1000);
        wifi_command_get_mac(_device_mac);
    }
    if (_device_mac[0] == '\0')
    {
        printf("MAC unavailable.\n");
        return 0;
    }
    printf("MAC: %s\n", _device_mac);

    snprintf(_pub_topic, sizeof(_pub_topic), "arduino/%s/sensors", _device_mac);
    snprintf(_sub_topic, sizeof(_sub_topic), "arduino/%s/commands", _device_mac);
    printf("Pub: %s\n", _pub_topic);
    printf("Sub: %s\n", _sub_topic);

    char client_id[32];
    char clean_mac[13] = {0};
    uint8_t j = 0;
    for (uint8_t i = 0; _device_mac[i] && j < 12; i++)
        if (_device_mac[i] != ':')
            clean_mac[j++] = _device_mac[i];
    snprintf(client_id, sizeof(client_id), "plantpot_%s", clean_mac);
    printf("Client ID: %s\n", client_id);

    /* Ensure single connection mode — captive portal uses CIPMUX=1 */
    wifi_command("AT+CIPSERVER=0", 2); /* stop any running server first */
    _delay_ms(200);
    WIFI_ERROR_MESSAGE_t mux_result = wifi_command("AT+CIPMUX=0", 2);
    printf("CIPMUX=0: %d\n", mux_result);
    _delay_ms(200);

    printf("TCP connecting to %s:%u...\n", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    _rx_buf[0] = '\0';
    _rx_ready = 0;

    uint8_t tcp_ok = 0;
    for (uint8_t attempt = 0; attempt < 5; attempt++)
    {
        printf("TCP attempt %d\n", attempt + 1);
        if (wifi_command_create_TCP_connection(MQTT_BROKER_HOST, MQTT_BROKER_PORT,
                                               tcp_rx_callback, _rx_buf) == WIFI_OK)
        {
            tcp_ok = 1;
            printf("TCP OK\n");
            break;
        }
        /* Exponential backoff: 2s, 4s, 8s, 16s, 32s */
        uint16_t wait_ms = 2000;
        for (uint8_t b = 0; b < attempt; b++)
            wait_ms *= 2;
        printf("Retrying in %u ms...\n", wait_ms);
        for (uint16_t w = 0; w < wait_ms / 100; w++)
            _delay_ms(100);
    }
    if (!tcp_ok)
    {
        printf("TCP failed after 5 attempts.\n");
        return 0;
    }

    pkt_len = mqtt_build_connect(pkt, sizeof(pkt), client_id,
                                 MQTT_USERNAME, MQTT_PASSWORD);
    if (!pkt_len)
    {
        printf("CONNECT build failed.\n");
        return 0;
    }

    _rx_ready = 0;
    _rx_buf[0] = '\0';
    printf("Sending MQTT CONNECT...\n");
    wifi_command_TCP_transmit(pkt, pkt_len);

    for (uint16_t i = 0; i < 500; i++)
    {
        _delay_ms(10);
        if (_rx_ready)
            break;
    }

    if (!_rx_ready)
    {
        printf("CONNACK timeout.\n");
        return 0;
    }
    if (_rx_buf[0] != 0x20)
    {
        printf("Not a CONNACK.\n");
        return 0;
    }
    if (_rx_buf[3] != 0x00)
    {
        printf("CONNACK rejected: 0x%02X\n",
               (uint8_t)_rx_buf[3]);
        return 0;
    }
    printf("CONNACK OK\n");

    _rx_ready = 0;
    _rx_buf[0] = '\0';
    pkt_len = mqtt_build_subscribe(pkt, sizeof(pkt), _sub_topic, 1);
    if (!pkt_len)
    {
        printf("SUBSCRIBE build failed.\n");
        return 0;
    }

    printf("Subscribing: %s\n", _sub_topic);
    wifi_command_TCP_transmit(pkt, pkt_len);

    for (uint16_t i = 0; i < 200; i++)
    {
        _delay_ms(10);
        if (_rx_ready)
            break;
    }

    if (!_rx_ready || (uint8_t)_rx_buf[0] != 0x90)
    {
        printf("SUBACK failed.\n");
        return 0;
    }
    printf("SUBACK OK\n");

    printf("MQTT ready.\n");
    _mqtt_connected = 1;
    /* Clear any leftover broker responses */
    _rx_ready = 0;
    _rx_buf[0] = '\0';
    _seconds_since_ping = 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Publish                                                             */
/* ------------------------------------------------------------------ */
uint8_t mqtt_raw_publish(const char *payload)
{
    /* Flush any stale rx data before transmitting */
    _rx_ready = 0;
    _rx_buf[0] = '\0';

    uint8_t pkt[198];
    uint8_t pkt_len = mqtt_build_publish(pkt, sizeof(pkt), _pub_topic, payload);
    if (!pkt_len)
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

/* ------------------------------------------------------------------ */
/*  Handle incoming commands                                            */
/* ------------------------------------------------------------------ */
void mqtt_handle_incoming(void)
{
    if (!_rx_ready)
        return;
    _rx_ready = 0;

    uint8_t *data = (uint8_t *)_rx_buf;

    if (data[0] == 0x30)
    {
        uint8_t remaining = data[1];
        uint8_t topic_len = (data[2] << 8) | data[3];
        if (topic_len < remaining)
        {
            uint8_t payload_offset = 4 + topic_len;
            uint8_t payload_len = remaining - 2 - topic_len;
            char payload_str[64] = {0};
            if (payload_len >= sizeof(payload_str))
                payload_len = sizeof(payload_str) - 1;
            memcpy(payload_str, &_rx_buf[payload_offset], payload_len);
            payload_str[payload_len] = '\0';
            printf("CMD: %s\n", payload_str);

            if (strcmp(payload_str, "led_on") == 0)
                led_on(1);
            else if (strcmp(payload_str, "led_off") == 0)
                led_off(1);
            else if (strcmp(payload_str, "beep") == 0)
                buzzer_beep();
            else if (strncmp(payload_str, "pump_on_", 8) == 0)
                pump_run_for((uint8_t)atoi(&payload_str[8]) * 1000);
            else
                printf("Unknown CMD: %s\n", payload_str);
        }
    }
    else if (data[0] == 0x20)
        printf("CONNACK\n");
    else if (data[0] == 0x90)
        printf("SUBACK\n");
    else if (data[0] == 0xD0)
        printf("PINGRESP\n");

    _rx_buf[0] = '\0';
}

/* ------------------------------------------------------------------ */
/*  Ping / keepalive                                                    */
/* ------------------------------------------------------------------ */
void mqtt_send_ping(void)
{
    uint8_t ping[2];
    mqtt_build_pingreq(ping);
    wifi_command_TCP_transmit(ping, 2);
}

uint8_t mqtt_is_connected(void) { return _mqtt_connected; }

void mqtt_tick(uint16_t elapsed_seconds)
{
    _seconds_since_ping += elapsed_seconds;
    if (_seconds_since_ping >= PING_INTERVAL_S)
    {
        mqtt_send_ping();
        _seconds_since_ping = 0;
    }
}