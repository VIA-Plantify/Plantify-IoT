#pragma once
#include <stdint.h>
#include "uart.h"

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
void wifi_set_raw_callback(void (*cb)(uint8_t));
uint8_t wifi_get_last_conn_id(void);

WIFI_ERROR_MESSAGE_t wifi_command(const char *str, uint16_t timeOut_s);
WIFI_ERROR_MESSAGE_t wifi_command_AT(void);
WIFI_ERROR_MESSAGE_t wifi_command_disable_echo(void);
WIFI_ERROR_MESSAGE_t wifi_command_set_mode_to_1(void);
WIFI_ERROR_MESSAGE_t wifi_command_join_AP(char *ssid, char *password);
WIFI_ERROR_MESSAGE_t wifi_command_get_mac(char *mac_address);
WIFI_ERROR_MESSAGE_t wifi_command_close_TCP_connection(void);
WIFI_ERROR_MESSAGE_t wifi_command_create_TCP_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer);
WIFI_ERROR_MESSAGE_t wifi_command_create_SSL_connection(char *IP, uint16_t port,
                                                        WIFI_TCP_Callback_t callback_when_message_received,
                                                        char *received_message_buffer);
WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit(uint8_t *data, uint16_t length);
WIFI_ERROR_MESSAGE_t wifi_command_TCP_transmit_direct(uint8_t *data, uint16_t length);
WIFI_ERROR_MESSAGE_t wifi_command_scan(char *result_buffer, uint16_t buffer_size);
WIFI_ERROR_MESSAGE_t wifi_command_start_TCP_server(WIFI_TCP_Callback_t callback_when_message_received,
                                                   char *received_message_buffer,
                                                   uint16_t buffer_size);
WIFI_ERROR_MESSAGE_t wifi_command_TCP_server_transmit(uint8_t conn_id, uint8_t *data, uint16_t length);
void wifi_print_gmr(void);