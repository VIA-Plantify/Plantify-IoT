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

static void wifi_TCP_callback(uint8_t byte);

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
 * TCP receive state machine
 * Handles both client mode: +IPD,<len>:<data>
 * and server mode:          +IPD,<conn_id>,<len>:<data>
 *--------------------------------------------------------------------------*/
static void wifi_TCP_callback(uint8_t byte)
{
    static enum { IDLE, MATCH_PREFIX, LENGTH, DATA } state = IDLE;
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
        else if (byte == ',')
        {
            // server mode: this is the connection ID comma
            // reset length and keep reading the actual length
            length = 0;
        }
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

    wifi_command("AT+CIPSSLSIZE=4096", 5);

    strcpy(sendbuffer, "AT+CIPSTART=\"SSL\",\"");
    strcat(sendbuffer, IP);
    strcat(sendbuffer, "\",");
    sprintf(portString, "%u", port);
    strcat(sendbuffer, portString);

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

/* Scan-specific buffer state */
static char     *scan_buffer_ptr;
static uint16_t  scan_buffer_size;
static uint16_t  scan_buffer_index;

static void wifi_scan_callback(uint8_t received_byte)
{
    if (scan_buffer_index < scan_buffer_size - 1)
    {
        scan_buffer_ptr[scan_buffer_index] = received_byte;
        scan_buffer_index++;
        scan_buffer_ptr[scan_buffer_index] = '\0';
    }
    // if buffer nearly full, null terminate and stop — better than overflow
}

WIFI_ERROR_MESSAGE_t wifi_command_scan(char *result_buffer, uint16_t buffer_size)
{
    void *callback_state = _callback;

    scan_buffer_ptr   = result_buffer;
    scan_buffer_size  = buffer_size;
    scan_buffer_index = 0;
    result_buffer[0]  = '\0';

    _callback = wifi_scan_callback;

    uart_send_string_blocking(UART2_ID, "AT+CWLAP\r\n");

    for (uint16_t i = 0; i < 15 * 100UL; i++)
    {
        _delay_ms(10);
        if (strstr(result_buffer, "OK\r\n") != NULL)
            break;
    }

    WIFI_ERROR_MESSAGE_t error;
    if (scan_buffer_index == 0)
        error = WIFI_ERROR_NOT_RECEIVING;
    else if (strstr(result_buffer, "OK") != NULL)
        error = WIFI_OK;
    else if (strstr(result_buffer, "ERROR") != NULL)
        error = WIFI_ERROR_RECEIVED_ERROR;
    else
        error = WIFI_ERROR_RECEIVING_GARBAGE;

    _callback = callback_state;
    return error;
}

WIFI_ERROR_MESSAGE_t wifi_command_start_TCP_server(WIFI_TCP_Callback_t callback_when_message_received,
                                                    char *received_message_buffer)
{
    received_message_buffer_static_pointer = received_message_buffer;
    callback_when_message_received_static = callback_when_message_received;

    wifi_command("AT+CIPMUX=1", 2);
    wifi_command("AT+CIPSERVER=1,80", 2);

    _callback = wifi_TCP_callback;

    wifi_clear_databuffer_and_index();
    return WIFI_OK;
}

WIFI_ERROR_MESSAGE_t wifi_command_TCP_server_transmit(uint8_t conn_id, uint8_t *data, uint16_t length)
{
    char sendbuffer[128];
    char portString[7];
    sprintf(sendbuffer, "AT+CIPSEND=%u,", conn_id);
    sprintf(portString, "%u", length);
    strcat(sendbuffer, portString);

    WIFI_ERROR_MESSAGE_t errorMessage = wifi_command(sendbuffer, 20);
    if (errorMessage != WIFI_OK)
        return errorMessage;

    uart_write_bytes(UART2_ID, data, length);
    return WIFI_OK;
}