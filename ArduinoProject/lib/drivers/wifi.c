/**
 * @file wifi.c
 * @author Laurits Ivar / Erland Larsen
 * @brief ESP8266 WiFi module implementation using UART.
 * @version 1.1
 * @date 2023-08-23
 * Revision history: 0.1 - Initial version
 *                   0.9 - 2026-03-01 Refactored to use common uart driver
 *                   1.0 - 2026-04-22 Added SSL connection and MAC read
 *                   1.1 - 2026-05-07 Removed unused functions
 *
 * @copyright Copyright (c) 2026
 */
#include "wifi.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include "uart.h"

#define WIFI_DATABUFFERSIZE 128
#define IPD_PREFIX "+IPD,"
#define PREFIX_LENGTH 5

static uint8_t wifi_dataBuffer[WIFI_DATABUFFERSIZE];
static uint8_t wifi_dataBufferIndex;
static uint32_t wifi_baudrate;
static void (*_callback)(uint8_t byte);

static WIFI_TCP_Callback_t callback_when_message_received_static;
static char *received_message_buffer_static_pointer;

/* Forward declaration */
static void wifi_TCP_callback(uint8_t byte);

/*---------------------------------------------------------------------------
 * Internal helpers
 *--------------------------------------------------------------------------*/
static void wifi_callback(uint8_t received_byte)
{
    if (NULL != _callback)
        _callback(received_byte);
}

static void wifi_command_callback(uint8_t received_byte)
{
    if (wifi_dataBufferIndex < WIFI_DATABUFFERSIZE - 1)
    {
        wifi_dataBuffer[wifi_dataBufferIndex] = received_byte;
        wifi_dataBufferIndex++;
    }
}

static void wifi_clear_databuffer_and_index(void)
{
    for (uint16_t i = 0; i < WIFI_DATABUFFERSIZE; i++)
        wifi_dataBuffer[i] = 0;
    wifi_dataBufferIndex = 0;
}

/*---------------------------------------------------------------------------
 * TCP receive state machine — parses +IPD,<len>:<data>
 *--------------------------------------------------------------------------*/
static void wifi_TCP_callback(uint8_t byte)
{
    static enum { IDLE,
                  MATCH_PREFIX,
                  LENGTH,
                  DATA } state = IDLE;
    static int length = 0, index = 0, prefix_index = 0;

    switch (state)
    {
    case IDLE:
        if (byte == IPD_PREFIX[0])
        {
            state = MATCH_PREFIX;
            prefix_index = 1;
        }
        break;

    case MATCH_PREFIX:
        if (byte == IPD_PREFIX[prefix_index])
        {
            if (prefix_index == PREFIX_LENGTH - 1)
                state = LENGTH;
            else
                prefix_index++;
        }
        else
        {
            state = IDLE;
            prefix_index = 0;
        }
        break;

    case LENGTH:
        if (byte >= '0' && byte <= '9')
            length = length * 10 + (byte - '0');
        else if (byte == ':')
        {
            state = DATA;
            index = 0;
        }
        else
        {
            state = IDLE;
            length = 0;
        }
        break;

    case DATA:
        if (index < length)
            received_message_buffer_static_pointer[index++] = byte;
        if (index == length)
        {
            received_message_buffer_static_pointer[index] = '\0';
            state = IDLE;
            length = 0;
            index = 0;
            wifi_clear_databuffer_and_index();
            callback_when_message_received_static();
        }
        break;
    }
}

/*---------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/
void wifi_init(void)
{
    wifi_baudrate = 115200;
    uart_init(UART2_ID, wifi_baudrate, wifi_callback, 0);
}

WIFI_ERROR_MESSAGE_t wifi_command(const char *str, uint16_t timeOut_s)
{
    void *callback_state = _callback;
    _callback = wifi_command_callback;

    char sendbuffer[128];
    strcpy(sendbuffer, str);
    uart_send_string_blocking(UART2_ID, strcat(sendbuffer, "\r\n"));

    for (uint16_t i = 0; i < timeOut_s * 100UL; i++)
    {
        _delay_ms(10);
        if (strstr((char *)wifi_dataBuffer, "OK\r\n") != NULL)
            break;
    }

    WIFI_ERROR_MESSAGE_t error;
    if (wifi_dataBufferIndex == 0)
        error = WIFI_ERROR_NOT_RECEIVING;
    else if (strstr((char *)wifi_dataBuffer, "OK") != NULL)
        error = WIFI_OK;
    else if (strstr((char *)wifi_dataBuffer, "ERROR") != NULL)
        error = WIFI_ERROR_RECEIVED_ERROR;
    else if (strstr((char *)wifi_dataBuffer, "FAIL") != NULL)
        error = WIFI_FAIL;
    else
        error = WIFI_ERROR_RECEIVING_GARBAGE;

    wifi_clear_databuffer_and_index();
    _callback = callback_state;
    return error;
}

WIFI_ERROR_MESSAGE_t wifi_command_AT(void)
{
    return wifi_command("AT", 1);
}

WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void)
{
    return wifi_command("ATE0", 1);
}

WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void)
{
    return wifi_command("AT+CWMODE=1", 1);
}

WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password)
{
    char sendbuffer[128];
    strcpy(sendbuffer, "AT+CWJAP=\"");
    strcat(sendbuffer, ssid);
    strcat(sendbuffer, "\",\"");
    strcat(sendbuffer, password);
    strcat(sendbuffer, "\"");
    return wifi_command(sendbuffer, 20);
}

WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address)
{
    void *callback_state = _callback;
    _callback = wifi_command_callback;

    uart_send_string_blocking(UART2_ID, "AT+CIFSR\r\n");

    for (uint16_t i = 0; i < 5 * 100UL; i++)
    {
        _delay_ms(10);
        if (strstr((char *)wifi_dataBuffer, "OK\r\n") != NULL)
            break;
    }

    WIFI_ERROR_MESSAGE_t error;
    if (wifi_dataBufferIndex == 0)
        error = WIFI_ERROR_NOT_RECEIVING;
    else if (strstr((char *)wifi_dataBuffer, "OK") != NULL)
        error = WIFI_OK;
    else if (strstr((char *)wifi_dataBuffer, "ERROR") != NULL)
        error = WIFI_ERROR_RECEIVED_ERROR;
    else
        error = WIFI_ERROR_RECEIVING_GARBAGE;

    if (error == WIFI_OK)
    {
        char *macStart = strstr((char *)wifi_dataBuffer, "STAMAC,\"");
        if (macStart != NULL)
        {
            macStart += strlen("STAMAC,\"");
            char *macEnd = strchr(macStart, '"');
            if (macEnd != NULL && (macEnd - macStart) <= 17)
            {
                strncpy(mac_address, macStart, macEnd - macStart);
                mac_address[macEnd - macStart] = '\0';
            }
        }
    }

    wifi_clear_databuffer_and_index();
    _callback = callback_state;
    return error;
}

WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection(void)
{
    return wifi_command("AT+CIPCLOSE", 5);
}

WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer)
{
    received_message_buffer_static_pointer = received_message_buffer;
    callback_when_message_received_static = callback_when_message_received;

    char sendbuffer[128];
    char portString[7];

    strcpy(sendbuffer, "AT+CIPSTART=\"TCP\",\"");
    strcat(sendbuffer, IP);
    strcat(sendbuffer, "\",");
    sprintf(portString, "%u", port);
    strcat(sendbuffer, portString);

    WIFI_ERROR_MESSAGE_t errorMessage = wifi_command(sendbuffer, 20);
    if (errorMessage == WIFI_OK)
        _callback = wifi_TCP_callback;

    wifi_clear_databuffer_and_index();
    return errorMessage;
}

WIFI_ERROR_MESSAGE_t wifi_command_create_SSL_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer)
{
    received_message_buffer_static_pointer = received_message_buffer;
    callback_when_message_received_static = callback_when_message_received;

    char sendbuffer[128];
    char portString[7];

    /* Enable SSL buffer — required before SSL handshake */
    wifi_command("AT+CIPSSLSIZE=4096", 5);

    strcpy(sendbuffer, "AT+CIPSTART=\"SSL\",\"");
    strcat(sendbuffer, IP);
    strcat(sendbuffer, "\",");
    sprintf(portString, "%u", port);
    strcat(sendbuffer, portString);

    /* SSL handshake takes longer — 30s timeout */
    WIFI_ERROR_MESSAGE_t errorMessage = wifi_command(sendbuffer, 30);
    if (errorMessage == WIFI_OK)
        _callback = wifi_TCP_callback;

    wifi_clear_databuffer_and_index();
    return errorMessage;
}

WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length)
{
    char sendbuffer[128];
    char portString[7];
    strcpy(sendbuffer, "AT+CIPSEND=");
    sprintf(portString, "%u", length);
    strcat(sendbuffer, portString);

    WIFI_ERROR_MESSAGE_t errorMessage = wifi_command(sendbuffer, 20);
    if (errorMessage != WIFI_OK)
        return errorMessage;

    uart_write_bytes(UART2_ID, data, length);
    return WIFI_OK;
}