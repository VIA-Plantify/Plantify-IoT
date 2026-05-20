#include "captive_portal.h"
#include "wifi.h"
#include "eeprom_storage.h"
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"

#define PORTAL_SSID "PlantPot-Setup"
#define PORTAL_RX_SIZE 768

static char portal_rx_buf[PORTAL_RX_SIZE];
static volatile uint8_t portal_rx_ready = 0;
static uint8_t waiting_for_body = 0;

static void close_connection(void);

static void portal_rx_callback(void)
{
    portal_rx_ready = 1;
}

void software_reset(void)
{
    wdt_enable(WDTO_15MS);
    while (1)
        ;
}

static void parse_body(char *body)
{
    char ssid[32] = {0};
    char password[64] = {0};

    printf_P(PSTR("Parsing body: '%s'\n"), body);

    char *ssid_start = strstr(body, "ssid=");
    if (ssid_start)
    {
        ssid_start += 5;
        char *ssid_end = strchr(ssid_start, '&');
        if (ssid_end)
        {
            uint8_t len = ssid_end - ssid_start;
            if (len > 31)
                len = 31;
            strncpy(ssid, ssid_start, len);
        }
        else
        {
            uint8_t len = strlen(ssid_start);
            if (len > 31)
                len = 31;
            strncpy(ssid, ssid_start, len);
        }
    }
    else
        printf_P(PSTR("ERROR: ssid= not found\n"));

    char *pass_start = strstr(body, "pass=");
    if (pass_start)
    {
        pass_start += 5;
        uint8_t len = strlen(pass_start);
        if (len > 63)
            len = 63;
        strncpy(password, pass_start, len);
    }
    else
        printf_P(PSTR("ERROR: pass= not found\n"));

    printf_P(PSTR("SSID: '%s'\n"), ssid);
    printf_P(PSTR("Pass: '%s'\n"), password);

    if (ssid[0] != '\0')
    {
        save_credentials(ssid, password);

        char v_ssid[32] = {0};
        char v_pass[64] = {0};
        load_credentials(v_ssid, v_pass);
        printf_P(PSTR("Verified SSID: '%s'\n"), v_ssid);
        printf_P(PSTR("Verified Pass: '%s'\n"), v_pass);

        static const char success[] =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: 40\r\n"
            "\r\n"
            "<html><body><h2>Saved!</h2></body></html>";

        wifi_command_TCP_server_transmit(wifi_get_last_conn_id(),
                                         (uint8_t *)success, strlen(success));
        _delay_ms(500);
        close_connection();
        _delay_ms(1000);
        printf_P(PSTR("Rebooting...\n"));
        software_reset();
    }
    else
    {
        printf_P(PSTR("SSID empty\n"));
        close_connection();
    }
}

static void send_html(void)
{
    /* Send in two chunks — wifi buffer is only 128 bytes.
       Chunk 1: HTTP headers + start of body
       Chunk 2: rest of body                              */
    uint8_t conn = wifi_get_last_conn_id();

    static const char chunk1[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: 195\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<html><body><h2>Plant Pot</h2>"
        "<form method=\'POST\' action=\'/save\'>"
        "SSID:<input name=\'ssid\'><br>";

    static const char chunk2[] =
        "Pass:<input name=\'pass\' type=\'password\'><br>"
        "<input type=\'submit\' value=\'Connect\'>"
        "</form></body></html>";

    printf_P(PSTR("Sending chunk1 (%u) to conn %d\n"),
             (uint16_t)strlen(chunk1), conn);
    wifi_command_TCP_server_transmit(conn, (uint8_t *)chunk1, strlen(chunk1));
    _delay_ms(200);

    printf_P(PSTR("Sending chunk2 (%u) to conn %d\n"),
             (uint16_t)strlen(chunk2), conn);
    wifi_command_TCP_server_transmit(conn, (uint8_t *)chunk2, strlen(chunk2));
    _delay_ms(200);
}

static void close_connection(void)
{
    printf_P(PSTR("Closing conn %d\n"), wifi_get_last_conn_id());
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", wifi_get_last_conn_id());
    uart_send_string_blocking(UART2_ID, cmd);
    _delay_ms(500);
}

void start_captive_portal(void)
{
    printf_P(PSTR("Starting portal...\n"));

    /* Close any open connections without resetting ESP */
    wifi_command("AT+CIPCLOSE=0", 2);
    wifi_command("AT+CIPCLOSE=1", 2);
    wifi_command("AT+CIPSERVER=0", 2);
    _delay_ms(500);

    wifi_command_AT();
    wifi_command_disable_echo();
    wifi_command("AT+CWAUTOCONN=0", 2);
    wifi_command("AT+CWQAP", 2);
    _delay_ms(500);

    printf_P(PSTR("CWMODE=3...\n"));
    wifi_command("AT+CWMODE=3", 3);
    _delay_ms(2000);

    wifi_command("AT+CWDHCP=1,1", 2);
    wifi_command("AT+CIPMUX=1", 2);

    printf_P(PSTR("CWSAP...\n"));
    uart_send_string_blocking(UART2_ID, "AT+CWSAP=\"PlantPot-Setup\",,\"\",5,0\r\n");
    _delay_ms(3000);

    printf_P(PSTR("Opening port 80...\n"));
    wifi_command("AT+CIPSERVER=1,80", 5);
    _delay_ms(500);

    printf_P(PSTR("TCP server...\n"));
    wifi_command_start_TCP_server(portal_rx_callback, portal_rx_buf, PORTAL_RX_SIZE);
    _delay_ms(1000);

    printf_P(PSTR("Portal running - connect to '%s'\n"), PORTAL_SSID);
    printf_P(PSTR("Open browser: http://192.168.4.1\n"));

    portal_rx_ready = 0;
    waiting_for_body = 0;

    while (1)
    {
        if (portal_rx_ready)
        {
            portal_rx_ready = 0;
            printf_P(PSTR("Request received\n"));
            printf_P(PSTR("Buf: '%.60s'\n"), portal_rx_buf);

            /* Always check for ssid= first — body may arrive in any packet */
            if (strstr(portal_rx_buf, "ssid=") != NULL)
            {
                char *body = strstr(portal_rx_buf, "ssid=");
                waiting_for_body = 0;
                printf_P(PSTR("Parsing credentials...\n"));
                parse_body(body);
            }
            else if (strstr(portal_rx_buf, "POST /save") != NULL)
            {
                printf_P(PSTR("Waiting for body...\n"));
                waiting_for_body = 1;
            }
            else if (portal_rx_buf[0] == '\0')
            {
                /* empty packet — ignore */
            }
            else if (strstr(portal_rx_buf, "GET /favicon.ico") != NULL)
            {
                close_connection();
            }
            else if (strstr(portal_rx_buf, "GET /") != NULL)
            {
                send_html();
                _delay_ms(1000);
                close_connection();
            }

            portal_rx_buf[0] = '\0';
        }
    }
}