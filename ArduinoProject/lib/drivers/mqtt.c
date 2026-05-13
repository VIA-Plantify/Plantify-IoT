#include "mqtt.h"
#include "wifi.h"
#include "led.h"
#include "buzzer.h"
#include "pump.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static uint8_t _mqtt_connected = 0;
static uint16_t _seconds_since_ping = 0;
static char _rx_buf[128] = {0};
static volatile uint8_t _rx_ready = 0;
static char _pub_topic[64] = {0};
static char _sub_topic[64] = {0};
char _device_mac[18] = {0};

/* ------------------------------------------------------------------ */
/*  UART receive callback — fires when a complete +IPD frame arrives   */
/* ------------------------------------------------------------------ */

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
    buf[i++] = 0xC6;
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
    uint8_t payload_len = (uint8_t)strlen(payload);
    uint8_t remaining = 2 + topic_len + payload_len;
    if ((uint8_t)(2 + remaining) > buf_size)
        return 0;

    uint8_t i = 0;
    buf[i++] = 0x30;
    buf[i++] = remaining;
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

uint8_t mqtt_raw_connect_with_credentials(char *ssid, char *password)
{
    uint8_t pkt[128];
    uint8_t pkt_len;

    _mqtt_connected = 0;

    printf_P(PSTR("AT check\n"));
    if (wifi_command_AT() != WIFI_OK)
    {
        printf_P(PSTR("ESP fail\n"));
        return 0;
    }
    printf_P(PSTR("AT OK\n"));

    wifi_command_disable_echo();
    wifi_command_set_mode_to_1();
    _delay_ms(500);

    printf_P(PSTR("Joining: %s\n"), ssid);
    WIFI_ERROR_MESSAGE_t join = wifi_command_join_AP(ssid, password);
    printf_P(PSTR("Join result: %d\n"), join);
    if (join != WIFI_OK)
    {
        printf_P(PSTR("Join fail\n"));
        return 0;
    }
    _delay_ms(5000);

    if (wifi_command_get_mac(_device_mac) == WIFI_OK)
        printf_P(PSTR("MAC: %s\n"), _device_mac);
    else
        printf_P(PSTR("MAC fail\n"));

    snprintf(_pub_topic, sizeof(_pub_topic), "arduino/%s/sensors", _device_mac);
    snprintf(_sub_topic, sizeof(_sub_topic), "arduino/%s/commands", _device_mac);
    printf_P(PSTR("Pub: %s\n"), _pub_topic);
    printf_P(PSTR("Sub: %s\n"), _sub_topic);

    char client_id[32];
    char clean_mac[13] = {0};
    uint8_t j = 0;
    for (uint8_t i = 0; i < strlen(_device_mac) && j < 12; i++)
        if (_device_mac[i] != ':')
            clean_mac[j++] = _device_mac[i];
    snprintf(client_id, sizeof(client_id), "plantpot_%s", clean_mac);

    printf_P(PSTR("SSL connecting...\n"));
    _rx_buf[0] = '\0';
    _rx_ready = 0;

    uint8_t ssl_ok = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        printf_P(PSTR("SSL attempt %d\n"), i + 1);
        WIFI_ERROR_MESSAGE_t ssl = wifi_command_create_SSL_connection(
            MQTT_BROKER_HOST, MQTT_BROKER_PORT, tcp_rx_callback, _rx_buf);
        printf_P(PSTR("SSL result: %d\n"), ssl);
        if (ssl == WIFI_OK)
        {
            ssl_ok = 1;
            printf_P(PSTR("SSL OK\n"));
            break;
        }
        _delay_ms(3000);
    }

    if (!ssl_ok)
    {
        printf_P(PSTR("SSL all failed\n"));
        return 0;
    }

    pkt_len = mqtt_build_connect(pkt, sizeof(pkt), client_id,
                                 MQTT_USERNAME, MQTT_PASSWORD);
    if (!pkt_len)
    {
        printf_P(PSTR("CONNECT build fail\n"));
        return 0;
    }

    _rx_ready = 0;
    _rx_buf[0] = '\0';

    printf_P(PSTR("Sending CONNECT (%d bytes)\n"), pkt_len);
    printf_P(PSTR("CONNECT hex:\n"));
    for (uint8_t i = 0; i < pkt_len; i++)
        printf_P(PSTR("%02X "), pkt[i]);
    printf_P(PSTR("\n"));
    wifi_command_TCP_transmit(pkt, pkt_len);

    for (uint16_t i = 0; i < 300; i++)
    {
        _delay_ms(10);
        if (_rx_ready)
            break;
    }

    if (!_rx_ready)
    {
        printf_P(PSTR("CONNACK timeout\n"));
        return 0;
    }

    printf_P(PSTR("CONNACK: 0x%02X 0x%02X 0x%02X 0x%02X\n"),
             (uint8_t)_rx_buf[0], (uint8_t)_rx_buf[1],
             (uint8_t)_rx_buf[2], (uint8_t)_rx_buf[3]);

    if ((uint8_t)_rx_buf[0] != 0x20)
    {
        printf_P(PSTR("Not a CONNACK (0x%02X)\n"), (uint8_t)_rx_buf[0]);
        return 0;
    }
    if ((uint8_t)_rx_buf[3] != 0x00)
    {
        printf_P(PSTR("CONNACK rejected, code: 0x%02X\n"), (uint8_t)_rx_buf[3]);
        return 0;
    }
    printf_P(PSTR("CONNACK OK\n"));

    _rx_ready = 0;
    _rx_buf[0] = '\0';

    pkt_len = mqtt_build_subscribe(pkt, sizeof(pkt), _sub_topic, 1);
    if (!pkt_len)
    {
        printf_P(PSTR("SUBSCRIBE build fail\n"));
        return 0;
    }

    printf_P(PSTR("Subscribing: %s\n"), _sub_topic);
    wifi_command_TCP_transmit(pkt, pkt_len);

    for (uint16_t i = 0; i < 200; i++)
    {
        _delay_ms(10);
        if (_rx_ready)
            break;
    }

    if (_rx_ready && (uint8_t)_rx_buf[0] == 0x90)
        printf_P(PSTR("SUBACK OK\n"));
    else
        printf_P(PSTR("No SUBACK (continuing anyway)\n"));

    printf_P(PSTR("MQTT connected\n"));
    _mqtt_connected = 1;
    _seconds_since_ping = 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Publish                                                             */
/* ------------------------------------------------------------------ */

uint8_t mqtt_raw_publish(const char *payload)
{
    uint8_t pkt[198];
    uint8_t pkt_len = mqtt_build_publish(pkt, sizeof(pkt), _pub_topic, payload);
    if (pkt_len == 0)
    {
        printf_P(PSTR("Publish build fail\n"));
        return 0;
    }

    printf_P(PSTR("Publishing %d bytes\n"), pkt_len);
    WIFI_ERROR_MESSAGE_t result = wifi_command_TCP_transmit(pkt, pkt_len);
    if (result != WIFI_OK)
    {
        _mqtt_connected = 0;
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Handle incoming MQTT messages                                       */
/* ------------------------------------------------------------------ */

void mqtt_handle_incoming(void)
{
    if (!_rx_ready)
        return;
    _rx_ready = 0;

    uint8_t *data = (uint8_t *)_rx_buf;
    printf_P(PSTR("Incoming: 0x%02X\n"), data[0]);

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
            printf_P(PSTR("CMD: %s\n"), payload_str);

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
        printf_P(PSTR("CONNACK\n"));
    else if (data[0] == 0x90)
        printf_P(PSTR("SUBACK\n"));
    else if (data[0] == 0xD0)
        printf_P(PSTR("PINGRESP\n"));
    else
        printf_P(PSTR("Unknown: 0x%02X\n"), data[0]);

    _rx_buf[0] = '\0';
}

/* ------------------------------------------------------------------ */
/*  Ping / keepalive                                                    */
/* ------------------------------------------------------------------ */

void mqtt_send_ping(void)
{
    uint8_t ping[2];
    mqtt_build_pingreq(ping);
    printf_P(PSTR("PING\n"));
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

/* ------------------------------------------------------------------ */
/*  Simple test — plain TCP to public broker on port 80                */
/* ------------------------------------------------------------------ */

void mqtt_test_simple(void)
{
    printf_P(PSTR("=== Simple MQTT test ===\n"));

    printf_P(PSTR("AT...\n"));
    printf_P(PSTR("AT=%d\n"), wifi_command_AT());

    printf_P(PSTR("ATE0...\n"));
    printf_P(PSTR("ATE0=%d\n"), wifi_command_disable_echo());

    printf_P(PSTR("CWMODE=1...\n"));
    printf_P(PSTR("CWMODE=%d\n"), wifi_command("AT+CWMODE=1", 2));
    _delay_ms(500);

    printf_P(PSTR("Joining WiFi...\n"));
    WIFI_ERROR_MESSAGE_t join = wifi_command_join_AP("iPhone", "Print2021");
    printf_P(PSTR("Join=%d\n"), join);
    if (join != WIFI_OK)
    {
        printf_P(PSTR("WiFi join failed - stopping\n"));
        return;
    }
    _delay_ms(5000);

    printf_P(PSTR("Checking IP...\n"));
    printf_P(PSTR("CIFSR=%d\n"), wifi_command("AT+CIFSR", 3));

    printf_P(PSTR("Closing any existing connection...\n"));
    wifi_command("AT+CIPCLOSE", 3);
    _delay_ms(500);

    /* test.mosquitto.org listens for plain MQTT on port 80 */
    printf_P(PSTR("Connecting to: test.mosquitto.org:80\n"));
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"test.mosquitto.org\",80");
    WIFI_ERROR_MESSAGE_t tcp = wifi_command(cmd, 30);
    printf_P(PSTR("CIPSTART=%d\n"), tcp);
    if (tcp != WIFI_OK)
    {
        printf_P(PSTR("TCP connect failed - stopping\n"));
        return;
    }

    printf_P(PSTR("TCP connected!\n"));
    _delay_ms(500);

    /* Minimal MQTT CONNECT: client ID "t1", no credentials, clean session */
    uint8_t pkt[] = {
        0x10, 0x0C,
        0x00, 0x04,
        'M', 'Q', 'T', 'T',
        0x04,
        0x02,
        0x00, 0x3C,
        0x00, 0x02,
        't', '1'
    };

    printf_P(PSTR("Packet hex:\n"));
    for (uint8_t i = 0; i < sizeof(pkt); i++)
        printf_P(PSTR("%02X "), pkt[i]);
    printf_P(PSTR("\n"));

    printf_P(PSTR("Sending MQTT CONNECT...\n"));
    wifi_command_TCP_transmit_direct(pkt, sizeof(pkt));

    printf_P(PSTR("=== Test done ===\n"));
}