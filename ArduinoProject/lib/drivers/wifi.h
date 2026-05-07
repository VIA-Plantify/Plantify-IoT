/**
 * @file wifi.h
 * @author Laurits Ivar / Erland Larsen
 * @brief ESP8266 WiFi module interface using UART.
 * @version 1.1
 * @date 2023-08-23
 * Revision history: 0.1 - Initial version
 *                   0.9 - 2026-03-01 Refactored to use common uart driver
 *                   1.0 - 2026-04-22 Added retry logic and SSL support
 *                   1.1 - 2026-05-07 Removed unused functions
 *
 * @copyright Copyright (c) 2026
 */
#pragma once
#include <stdint.h>
#include "uart.h"

typedef enum
{
    WIFI_OK,                     /**< Command successful. */
    WIFI_FAIL,                   /**< General failure. */
    WIFI_ERROR_RECEIVED_ERROR,   /**< Received ERROR from module. */
    WIFI_ERROR_NOT_RECEIVING,    /**< No data received. */
    WIFI_ERROR_RECEIVING_GARBAGE /**< Unintelligible data received. */
} WIFI_ERROR_MESSAGE_t;

typedef void (*WIFI_TCP_Callback_t)();

/** @brief Initialize the WiFi module. Takes up to 4s to become ready. */
void wifi_init(void);

/** @brief Send a raw AT command and wait for OK/ERROR. */
WIFI_ERROR_MESSAGE_t wifi_command(const char *str, uint16_t timeOut_s);

/** @brief Check if ESP8266 is responsive (AT). */
WIFI_ERROR_MESSAGE_t wifi_command_AT(void);

/** @brief Disable echo (ATE0). */
WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void);

/** @brief Set WiFi module to station mode. */
WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void);

/** @brief Join a WiFi access point. */
WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password);

/**
 * @brief Read the station MAC address from the ESP8266.
 * @param mac_address Buffer of at least 18 bytes for "xx:xx:xx:xx:xx:xx".
 */
WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address);

/**
 * @brief Establish a plain TCP connection.
 * @param IP IP address or hostname.
 * @param port Port number.
 * @param callback_when_message_received Called when data arrives.
 * @param received_message_buffer Buffer to hold incoming data.
 */
WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer);

/**
 * @brief Establish an SSL/TLS connection (required for HiveMQ Cloud port 8883).
 * @param IP Hostname or IP.
 * @param port Port number.
 * @param callback_when_message_received Called when data arrives.
 * @param received_message_buffer Buffer to hold incoming data.
 */
WIFI_ERROR_MESSAGE_t wifi_command_create_SSL_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer);

/**
 * @brief Transmit data over an established TCP/SSL connection.
 * @param data Pointer to data.
 * @param length Number of bytes.
 */
WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length);

/** @brief Close the current TCP/SSL connection. */
WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection(void);