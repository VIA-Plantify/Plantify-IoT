#include "captive_portal.h"
#include "wifi.h"
#include "eeprom_storage.h"
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define PORTAL_SSID "PlantPot-Setup"

static char portal_rx_buf[256];
static char portal_html[512];
static volatile uint8_t portal_rx_ready = 0;

static void portal_rx_callback(void)
{
    portal_rx_ready = 1;
}

void software_reset(void)
{
    wdt_enable(WDTO_15MS);
    while (1);
}

static void send_html(void)
{
    uint16_t pos = 0;
    pos += snprintf(portal_html + pos, sizeof(portal_html) - pos,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n\r\n"
        "<html><body><h2>Plant Pot Setup</h2>"
        "<form method='POST' action='/save'>"
        "SSID: <input name='ssid'><br>"
        "Password: <input name='pass' type='password'><br>"
        "<button type='submit'>Connect</button>"
        "</form></body></html>");

    wifi_command_TCP_server_transmit(0, (uint8_t *)portal_html, pos);
}

static void parse_and_save(char *request)
{
    char *body = strstr(request, "\r\n\r\n");
    if (body == NULL) return;
    body += 4;

    char ssid[32] = {0};
    char password[64] = {0};

    char *ssid_start = strstr(body, "ssid=");
    if (ssid_start)
    {
        ssid_start += 5;
        char *ssid_end = strchr(ssid_start, '&');
        if (ssid_end)
        {
            uint8_t len = ssid_end - ssid_start;
            if (len > 31) len = 31;
            strncpy(ssid, ssid_start, len);
        }
    }

    char *pass_start = strstr(body, "pass=");
    if (pass_start)
    {
        pass_start += 5;
        char *pass_end = strchr(pass_start, ' ');
        uint8_t len = pass_end ? (pass_end - pass_start) : strlen(pass_start);
        if (len > 63) len = 63;
        strncpy(password, pass_start, len);
    }

    printf("Got SSID: %s\n", ssid);
    printf("Got Pass: %s\n", password);

    if (ssid[0] != '\0')
    {
        save_credentials(ssid, password);

        char *success = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                        "<html><body><h2>Saved! Rebooting...</h2></body></html>";
        wifi_command_TCP_server_transmit(0, (uint8_t *)success, strlen(success));

        _delay_ms(1000);
        software_reset();
    }
}

void start_captive_portal(void)
{
    printf("Starting captive portal...\n");

    wifi_command_AT();
    wifi_command_disable_echo();

    wifi_command("AT+CWMODE=3", 2);
    _delay_ms(1000);

    wifi_command("AT+CWSAP=\"" PORTAL_SSID "\",\"\",5,0", 5);
    _delay_ms(1000);

    wifi_command_start_TCP_server(portal_rx_callback, portal_rx_buf);
    _delay_ms(1000);

    printf("Portal running - connect to '%s'\n", PORTAL_SSID);
    printf("Open browser: 192.168.4.1\n");

    portal_rx_ready = 0;

    while (1)
    {
        if (portal_rx_ready)
        {
            portal_rx_ready = 0;

            if (strstr(portal_rx_buf, "POST /save") != NULL)
                parse_and_save(portal_rx_buf);
            else if (strstr(portal_rx_buf, "GET /favicon.ico") != NULL)
                wifi_command("AT+CIPCLOSE=0", 2);
            else if (strstr(portal_rx_buf, "GET /") != NULL)
            {
                send_html();
                _delay_ms(500);
                wifi_command("AT+CIPCLOSE=0", 2);
            }

            portal_rx_buf[0] = '\0';
        }
    }
}