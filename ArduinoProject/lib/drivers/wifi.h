/**
 * @file wifi.h
 * @brief ESP8266 WiFi module interface using UART.
 */
#pragma once
#include <stdint.h>
#include "uart.h"

#define WIFI_MAX_RETRIES    3
#define WIFI_SSID_MAX_LEN   32
#define WIFI_PASS_MAX_LEN   64

typedef enum
{
    WIFI_OK,
    WIFI_FAIL,
    WIFI_ERROR_RECEIVED_ERROR,
    WIFI_ERROR_NOT_RECEIVING,
    WIFI_ERROR_RECEIVING_GARBAGE,
    WIFI_ERROR_NO_CREDENTIALS
} WIFI_ERROR_MESSAGE_t;

typedef void (*WIFI_TCP_Callback_t)();

void wifi_init(void);

WIFI_ERROR_MESSAGE_t wifi_command(const char *str, uint16_t timeOut_s);
WIFI_ERROR_MESSAGE_t wifi_command_AT(void);
WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void);
WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void);
WIFI_ERROR_MESSAGE_t wifi_command_set_to_single_Connection(void);
WIFI_ERROR_MESSAGE_t wifi_command_quit_AP(void);
WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection(void);

WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password);
WIFI_ERROR_MESSAGE_t wifi_reconnect(void);

/*  Reads the MAC address of the ESP8266 station interface.
    mac_address must point to a buffer of at least 18 bytes.
    Returns WIFI_OK and fills mac_address on success.                  */
WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address);

WIFI_ERROR_MESSAGE_t wifi_command_get_ip_from_URL(char *url, char *ip_address);

WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer);

/*  Transmit data over TCP.
    Restores the TCP callback BEFORE sending so the broker response
    (CONNACK, SUBACK) is not lost in the post-send window.            */
WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length);