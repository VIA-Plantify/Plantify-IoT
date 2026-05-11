#include "captive_portal.h"
#include "wifi.h"
#include "eeprom_storage.h"
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"

#define PORTAL_SSID "PlantPot-Setup"
#define PORTAL_RX_SIZE 768

static char portal_rx_buf[PORTAL_RX_SIZE];
static char portal_html[400];
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

    printf("Parsing body: '%s'\n", body);

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
        printf("ERROR: ssid= not found\n");

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
        printf("ERROR: pass= not found\n");

    printf("SSID: '%s'\n", ssid);
    printf("Pass: '%s'\n", password);

    if (ssid[0] != '\0')
    {
        save_credentials(ssid, password);

        char v_ssid[32] = {0};
        char v_pass[64] = {0};
        load_credentials(v_ssid, v_pass);
        printf("Verified SSID: '%s'\n", v_ssid);
        printf("Verified Pass: '%s'\n", v_pass);

        char *success =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: 40\r\n"
            "\r\n"
            "<html><body><h2>Saved!</h2></body></html>";

        wifi_command_TCP_server_transmit(wifi_get_last_conn_id(),
                                         (uint8_t *)success, strlen(success));
        close_connection();
        _delay_ms(1000);
        printf("Rebooting...\n");
        software_reset();
    }
    else
    {
        printf("SSID empty\n");
        close_connection();
    }
}

static void send_html(void)
{
    static char response[400];

    static const char body[] =
        "<html><body>"
        "<h2>Plant Pot</h2>"
        "<form method='POST' action='/save'>"
        "SSID:<input name='ssid'><br>"
        "Pass:<input name='pass' type='password'><br>"
        "<input type='submit' value='Connect'>"
        "</form></body></html>";

    uint16_t body_len = strlen(body);
    uint16_t pos = snprintf(response, sizeof(response),
                            "HTTP/1.1 200 OK\r\n"
                            "Content-Type: text/html\r\n"
                            "Content-Length: %u\r\n"
                            "\r\n"
                            "%s",
                            body_len, body);

    printf("Sending %u bytes to conn %d\n", pos, wifi_get_last_conn_id());
    wifi_command_TCP_server_transmit(
        wifi_get_last_conn_id(), (uint8_t *)response, pos);
}

static void close_connection(void)
{
    printf("Closing conn %d\n", wifi_get_last_conn_id());
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", wifi_get_last_conn_id());
    uart_send_string_blocking(UART2_ID, cmd);
    _delay_ms(500);
}

void start_captive_portal(void)
{
    printf("Starting portal...\n");

    wifi_command_AT();
    wifi_command_disable_echo();

    // disable auto-connect and disconnect from any current network
    wifi_command("AT+CWAUTOCONN=0", 2);
    wifi_command("AT+CWQAP", 2);
    _delay_ms(500);

    uint8_t ap_ok = 0;
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        printf("AP setup attempt %d\n", attempt + 1);

        wifi_command("AT+CWMODE=3", 3);
        _delay_ms(2000);

        uart_send_string_blocking(UART2_ID, "AT+CWSAP=\"PlantPot-Setup\",\"\",5,0\r\n");
        _delay_ms(3000);

        if (wifi_command("AT+CWMODE?", 2) == WIFI_OK)
        {
            ap_ok = 1;
            printf("AP setup OK\n");
            break;
        }
        _delay_ms(1000);
    }

    if (!ap_ok)
    {
        printf("AP setup failed - resetting\n");
        software_reset();
    }

    wifi_command("AT+CIPSERVER=0", 2);
    _delay_ms(500);

    wifi_command_start_TCP_server(portal_rx_callback, portal_rx_buf, PORTAL_RX_SIZE);
    _delay_ms(1000);

    printf("Portal running - connect to '%s'\n", PORTAL_SSID);
    printf("Open browser: 192.168.4.1\n");

    portal_rx_ready  = 0;
    waiting_for_body = 0;

    while (1)
    {
        if (portal_rx_ready)
        {
            portal_rx_ready = 0;
            printf("Request received\n");

            if (waiting_for_body)
            {
                if (strstr(portal_rx_buf, "ssid=") != NULL)
                {
                    waiting_for_body = 0;
                    printf("Found body: '%s'\n", portal_rx_buf);
                    parse_body(portal_rx_buf);
                }
                else
                {
                    printf("Still waiting for body...\n");
                }
            }
            else if (strstr(portal_rx_buf, "POST /save") != NULL)
            {
                if (strstr(portal_rx_buf, "ssid=") != NULL)
                {
                    char *body = strstr(portal_rx_buf, "ssid=");
                    printf("Body in same packet\n");
                    parse_body(body);
                }
                else
                {
                    printf("Waiting for body...\n");
                    waiting_for_body = 1;
                }
            }
            else if (strstr(portal_rx_buf, "GET /favicon.ico") != NULL)
                close_connection();
            else if (strstr(portal_rx_buf, "GET /") != NULL)
            {
                send_html();
                close_connection();
            }

            portal_rx_buf[0] = '\0';
        }
    }
}