/**
 * @file wifi.h
 * @author Laurits Ivar / Erland Larsen
 * @brief ESP8266 WiFi module interface using UART.
 * @version 1.0
 * @date 2023-08-23
 * Revision history: 0.1 - Initial version
 *                   0.9 - 2026-03-01 Refactored to use common uart driver
 *                   1.0 - 2026-04-22 Added retry logic and auto-reconnect
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once
#include <stdint.h>
#include "uart.h"

#define WIFI_MAX_RETRIES    3       // Number of times to retry joining an AP
#define WIFI_SSID_MAX_LEN   32      // Maximum length of SSID
#define WIFI_PASS_MAX_LEN   32      // Maximum length of password

/**
 * @brief Enumerated list of possible error messages from the WiFi module.
 */
typedef enum
{
    WIFI_OK,                        /**< Command successful. */
    WIFI_FAIL,                      /**< General failure or operation not successful. */
    WIFI_ERROR_RECEIVED_ERROR,      /**< Received an error message from the module. */
    WIFI_ERROR_NOT_RECEIVING,       /**< No data received from the module. */
    WIFI_ERROR_RECEIVING_GARBAGE,   /**< Received unintelligible data from the module. */
    WIFI_ERROR_NO_CREDENTIALS       /**< No credentials saved for reconnect. */
} WIFI_ERROR_MESSAGE_t;

/**
 * @brief Type definition for WiFi TCP data received callback.
 */
typedef void (*WIFI_TCP_Callback_t)();

/**
 * @brief Initialize the WiFi module. After initialization it can take up to
 *        4 seconds before it is ready.
 */
void wifi_init(void);

/**
 * @brief Send an AT command to the WiFi module to check if it's responsive.
 *
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_AT(void);

/**
 * @brief Command the WiFi module to join a specific Access Point (AP).
 *        Retries up to WIFI_MAX_RETRIES times on failure.
 *        Saves credentials for use with wifi_reconnect().
 *        NOTE: ESP8266 only supports 2.4GHz networks.
 *
 * @param ssid Network SSID to join (max WIFI_SSID_MAX_LEN characters).
 * @param password Password for the SSID (max WIFI_PASS_MAX_LEN characters).
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password);

/**
 * @brief Reconnect to the last joined Access Point using saved credentials.
 *        Call wifi_command_join_AP() at least once before using this.
 *
 * @return WIFI_ERROR_MESSAGE_t WIFI_ERROR_NO_CREDENTIALS if no credentials are saved,
 *         otherwise the result of the reconnect attempt.
 */
WIFI_ERROR_MESSAGE_t wifi_reconnect(void);

/**
 * @brief Disable echo from the WiFi module.
 *
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void);

/**
 * @brief Set the WiFi module to mode 1 (station mode).
 *
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void);

/**
 * @brief Set the WiFi module to operate in single connection mode.
 *
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_set_to_single_Connection(void);

/**
 * @brief Resolve an IP address from a URL.
 *
 * @param url The URL to resolve.
 * @param ip_address Buffer to store the resolved IP address (min 16 bytes).
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_get_ip_from_URL(char *url, char *ip_address);

/**
 * @brief Establish a TCP connection using the WiFi module.
 *
 * @param IP IP address to connect to.
 * @param port Port number to use for the connection.
 * @param callback_when_message_received Callback executed when a message is received.
 * @param received_message_buffer Buffer to hold the received message.
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port, WIFI_TCP_Callback_t callback_when_message_received, char *received_message_buffer);

/**
 * @brief Transmit data over an established TCP connection.
 *
 * @param data Pointer to the data to transmit.
 * @param length Length of the data to transmit.
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length);

/**
 * @brief Disconnect from the current Access Point (AP).
 *
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_quit_AP(void);

/**
 * @brief  Closes the TCP connection
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection();

/**
 * @brief Read the ESP8266 station MAC address via AT+CIFSR.
 *
 * @param mac_address Buffer of at least 18 bytes to receive the MAC string
 *                    in "aa:bb:cc:dd:ee:ff" format (null-terminated).
 * @return WIFI_ERROR_MESSAGE_t Error message based on the response from the module.
 */
WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address);
