/**
 * @file wifi.c
 * @brief ESP8266 WiFi module implementation using UART.
 */
#include "wifi.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/delay.h>
#include "uart.h"

#define WIFI_DATABUFFERSIZE 128
#define IPD_PREFIX          "+IPD,"
#define PREFIX_LENGTH       5

static uint8_t  wifi_dataBuffer[WIFI_DATABUFFERSIZE];
static uint8_t  wifi_dataBufferIndex;
static uint32_t wifi_baudrate;
static void     (*_callback)(uint8_t byte);

static char _ssid[WIFI_SSID_MAX_LEN]     = {0};
static char _password[WIFI_PASS_MAX_LEN] = {0};

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
    if      (wifi_dataBufferIndex == 0)                        error = WIFI_ERROR_NOT_RECEIVING;
    else if (strstr((char *)wifi_dataBuffer, "OK")    != NULL) error = WIFI_OK;
    else if (strstr((char *)wifi_dataBuffer, "ERROR") != NULL) error = WIFI_ERROR_RECEIVED_ERROR;
    else if (strstr((char *)wifi_dataBuffer, "FAIL")  != NULL) error = WIFI_FAIL;
    else                                                        error = WIFI_ERROR_RECEIVING_GARBAGE;

    wifi_clear_databuffer_and_index();
    _callback = callback_state;
    return error;
}

WIFI_ERROR_MESSAGE_t wifi_command_AT(void)            { return wifi_command("AT", 1); }
WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void)  { return wifi_command("ATE0", 1); }
WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void) { return wifi_command("AT+CWMODE=1", 1); }
WIFI_ERROR_MESSAGE_t wifi_command_set_to_single_Connection(void) { return wifi_command("AT+CIPMUX=0", 1); }
WIFI_ERROR_MESSAGE_t wifi_command_quit_AP(void)       { return wifi_command("AT+CWQAP", 5); }
WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection(void) { return wifi_command("AT+CIPCLOSE", 5); }

WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password)
{
    strncpy(_ssid, ssid, WIFI_SSID_MAX_LEN - 1);
    _ssid[WIFI_SSID_MAX_LEN - 1] = '\0';
    strncpy(_password, password, WIFI_PASS_MAX_LEN - 1);
    _password[WIFI_PASS_MAX_LEN - 1] = '\0';

    char sendbuffer[128];
    strcpy(sendbuffer, "AT+CWJAP=\"");
    strcat(sendbuffer, ssid);
    strcat(sendbuffer, "\",\"");
    strcat(sendbuffer, password);
    strcat(sendbuffer, "\"");

    WIFI_ERROR_MESSAGE_t error;
    for (uint8_t i = 0; i < WIFI_MAX_RETRIES; i++)
    {
        error = wifi_command(sendbuffer, 20);
        if (error == WIFI_OK) return WIFI_OK;
        _delay_ms(1000);
    }
    return error;
}

WIFI_ERROR_MESSAGE_t wifi_reconnect(void)
{
    if (_ssid[0] == '\0') return WIFI_ERROR_NO_CREDENTIALS;
    WIFI_ERROR_MESSAGE_t error = wifi_command_AT();
    if (error != WIFI_OK) return error;
    return wifi_command_join_AP(_ssid, _password);
}

/*  Read the station MAC address from AT+CIFSR response.
    Parses: +CIFSR:STAMAC,"xx:xx:xx:xx:xx:xx"                         */
WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address)
{
    void *callback_state = _callback;
    _callback = wifi_command_callback;

    uart_send_string_blocking(UART2_ID, "AT+CIFSR\r\n");

    for (uint16_t i = 0; i < 500; i++)
    {
        _delay_ms(10);
        if (strstr((char *)wifi_dataBuffer, "OK\r\n") != NULL) break;
    }

    WIFI_ERROR_MESSAGE_t error = WIFI_ERROR_NOT_RECEIVING;
    if (wifi_dataBufferIndex > 0)
    {
        if      (strstr((char *)wifi_dataBuffer, "OK")    != NULL) error = WIFI_OK;
        else if (strstr((char *)wifi_dataBuffer, "ERROR") != NULL) error = WIFI_ERROR_RECEIVED_ERROR;
        else                                                        error = WIFI_ERROR_RECEIVING_GARBAGE;
    }

    if (error == WIFI_OK)
    {
        char *macStart = strstr((char *)wifi_dataBuffer, "STAMAC,\"");
        if (macStart)
        {
            macStart += strlen("STAMAC,\"");
            char *macEnd = strchr(macStart, '"');
            if (macEnd && (macEnd - macStart) <= 17)
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

WIFI_ERROR_MESSAGE_t wifi_command_get_ip_from_URL(char *url, char *ip_address)
{
    char sendbuffer[128];
    strcpy(sendbuffer, "AT+CIPDOMAIN=\"");
    strcat(sendbuffer, url);
    strcat(sendbuffer, "\"");

    void *callback_state = _callback;
    _callback = wifi_command_callback;
    uart_send_string_blocking(UART2_ID, strcat(sendbuffer, "\r\n"));

    for (uint16_t i = 0; i < 500; i++)
    {
        _delay_ms(10);
        if (strstr((char *)wifi_dataBuffer, "OK\r\n") != NULL) break;
    }

    WIFI_ERROR_MESSAGE_t error;
    if      (wifi_dataBufferIndex == 0)                        error = WIFI_ERROR_NOT_RECEIVING;
    else if (strstr((char *)wifi_dataBuffer, "OK")    != NULL) error = WIFI_OK;
    else if (strstr((char *)wifi_dataBuffer, "ERROR") != NULL) error = WIFI_ERROR_RECEIVED_ERROR;
    else if (strstr((char *)wifi_dataBuffer, "FAIL")  != NULL) error = WIFI_FAIL;
    else                                                        error = WIFI_ERROR_RECEIVING_GARBAGE;

    char *ipStart = strstr((char *)wifi_dataBuffer, "CIPDOMAIN:");
    if (ipStart)
    {
        ipStart += strlen("CIPDOMAIN:");
        char *ipEnd = strchr(ipStart, '\r');
        if (ipEnd && (ipEnd - ipStart) < 16)
        {
            strncpy(ip_address, ipStart, ipEnd - ipStart);
            ip_address[ipEnd - ipStart] = '\0';
        }
    }

    wifi_clear_databuffer_and_index();
    _callback = callback_state;
    return error;
}

/* ------------------------------------------------------------------ */
/*  TCP receive state machine                                           */
/* ------------------------------------------------------------------ */
WIFI_TCP_Callback_t callback_when_message_received_static;
char *received_message_buffer_static_pointer;

static void wifi_TCP_callback(uint8_t byte)
{
    static enum { IDLE, MATCH_PREFIX, LENGTH, DATA } state = IDLE;
    static int length = 0, index = 0, prefix_index = 0;

    switch (state)
    {
    case IDLE:
        if (byte == IPD_PREFIX[0]) { state = MATCH_PREFIX; prefix_index = 1; }
        break;

    case MATCH_PREFIX:
        if (byte == IPD_PREFIX[prefix_index])
        {
            if (prefix_index == PREFIX_LENGTH - 1) state = LENGTH;
            else prefix_index++;
        }
        else { state = IDLE; prefix_index = 0; }
        break;

    case LENGTH:
        if      (byte >= '0' && byte <= '9') length = length * 10 + (byte - '0');
        else if (byte == ':') { state = DATA; index = 0; }
        else                  { state = IDLE; length = 0; }
        break;

    case DATA:
        if (index < length)
            received_message_buffer_static_pointer[index++] = byte;
        if (index == length)
        {
            received_message_buffer_static_pointer[index] = '\0';
            state = IDLE; length = 0; index = 0;
            wifi_clear_databuffer_and_index();
            callback_when_message_received_static();
        }
        break;
    }
}

WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer)
{
    received_message_buffer_static_pointer = received_message_buffer;
    callback_when_message_received_static  = callback_when_message_received;

    char sendbuffer[128];
    char portString[7];
    strcpy(sendbuffer, "AT+CIPSTART=\"TCP\",\"");
    strcat(sendbuffer, IP);
    strcat(sendbuffer, "\",");
    sprintf(portString, "%u", port);
    strcat(sendbuffer, portString);

    WIFI_ERROR_MESSAGE_t err = wifi_command(sendbuffer, 20);
    if (err == WIFI_OK)
        _callback = wifi_TCP_callback;

    wifi_clear_databuffer_and_index();
    return err;
}

WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length)
{
    void (*saved_callback)(uint8_t) = _callback;
    _callback = wifi_command_callback;

    char sendbuffer[32];
    char portString[7];
    strcpy(sendbuffer, "AT+CIPSEND=");
    sprintf(portString, "%u", length);
    strcat(sendbuffer, portString);

    /*  Send AT+CIPSEND and wait for the '> ' prompt                   */
    uart_send_string_blocking(UART2_ID, strcat(sendbuffer, "\r\n"));

    uint8_t got_prompt = 0;
    for (uint16_t i = 0; i < 200; i++)
    {
        _delay_ms(10);
        if (wifi_dataBufferIndex > 1 &&
            wifi_dataBuffer[wifi_dataBufferIndex - 2] == '>' &&
            wifi_dataBuffer[wifi_dataBufferIndex - 1] == ' ')
        {
            got_prompt = 1;
            break;
        }
    }

    wifi_clear_databuffer_and_index();

    /*  Restore TCP callback BEFORE sending bytes so the broker's
        response (CONNACK/SUBACK) is parsed by the right handler.     */
    _callback = saved_callback;

    if (!got_prompt)
        return WIFI_FAIL;

    uart_write_bytes(UART2_ID, data, length);
    return WIFI_OK;
}