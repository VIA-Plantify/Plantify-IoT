/**
 * data_server.c  –  Plantify-IoT  TCP data export server
 *
 * Activated when BUTTON 3 is held during boot.
 * The device starts its own WiFi Access Point ("PlantPot-Data", no password),
 * opens a TCP server on port 9000, and streams one JSON line per second
 * containing all real sensor readings.
 *
 * The PC-side companion script (plantify_logger.py) connects to
 *   192.168.4.1:9000  and writes the data to a CSV file.
 *
 * HOW TO INTEGRATE:
 *   1. Add  #include "data_server.h"  to main.c
 *   2. In main(), after button_init(), add the block shown below.
 *   3. Add data_server.c to your src/ folder (PlatformIO picks it up automatically).
 *
 *   // ---- paste into main(), right after button_init() ----
 *   if (button_get(3))
 *       data_server_run();   // never returns while server is active
 *   // -------------------------------------------------------
 */

#include "data_server.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "wifi.h"
#include "uart.h"
#include "uart_stdio.h"
#include "led.h"
#include "button.h"
#include "display.h"
#include "dht11.h"
#include "light.h"
#include "soil.h"
#include "proximity.h"
#include "pir.h"
#include "captive_portal.h" /* for software_reset() */

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */
#define DATA_AP_SSID "PlantPot-Data"
#define DATA_SERVER_PORT 9000 /* TCP port the PC connects to */
#define SEND_INTERVAL_MS 1000 /* ms between readings          */

#define RX_BUF_SIZE 256

/* ------------------------------------------------------------------ */
/*  Internal state                                                      */
/* ------------------------------------------------------------------ */
static char _rx_buf[RX_BUF_SIZE];
static volatile uint8_t _client_connected = 0;
static volatile uint8_t _rx_ready = 0;

/* ------------------------------------------------------------------ */
/*  WiFi TCP-server receive callback                                    */
/*  Called by the wifi driver when a complete +IPD packet arrives.      */
/* ------------------------------------------------------------------ */
static void _on_client_data(void)
{
    /* A client sent something – treat it as a keep-alive / hello.      */
    _client_connected = 1;
    _rx_ready = 1;
}

/* ------------------------------------------------------------------ */
/*  Send one JSON line to every connected client                        */
/* ------------------------------------------------------------------ */
static void _send_sensor_json(void)
{
    uint8_t hum_i = 0, hum_d = 0, tmp_i = 0, tmp_d = 0;
    uint16_t light_val, soil_val, dist_mm;
    uint8_t motion;

    dht11_get(&hum_i, &hum_d, &tmp_i, &tmp_d);
    light_val = light_measure_raw();
    soil_val = soil_measure_percentage(ADC_PK0);
    dist_mm = proximity_measure();
    motion = (pir_get_state() != PIR_NO_MOTION) ? 1 : 0;

    /* Build JSON line – newline-terminated so the PC can split easily  */
    char line[160];
    snprintf(line, sizeof(line),
             "{\"temp\":%u.%u,\"hum\":%u.%u,"
             "\"light\":%u,\"soil\":%u,"
             "\"dist\":%u,\"motion\":%u}\n",
             tmp_i, tmp_d,
             hum_i, hum_d,
             light_val, soil_val,
             dist_mm, motion);

    uint16_t len = (uint16_t)strlen(line);
    uint8_t conn_id = wifi_get_last_conn_id();

    /* wifi_command_TCP_server_transmit handles AT+CIPSEND internally    */
    wifi_command_TCP_server_transmit(conn_id, (uint8_t *)line, len);

    /* Also echo to serial so you can watch in a terminal               */
    printf("%s", line);
}

/* ------------------------------------------------------------------ */
/*  Start the data-export AP + TCP server, then loop forever           */
/* ------------------------------------------------------------------ */
void data_server_run(void)
{
    /* ---- Peripheral init (subset needed for sensors) ---- */
    led_init();
    display_init();
    light_init();
    soil_init(ADC_PK0);
    pir_init(NULL); /* NULL callback – we poll manually        */
    /* adc_init() not needed – handled inside soil_init() and light_init() */
    /* sei() not needed – already called in main() before data_server_run() */

    if (UART_OK != uart_stdio_init(115200))
    {
        led_on(4);
        while (1)
        {
        }
    }

    printf_P(PSTR("\n[DataServer] Button 3 held - starting data export mode\n"));

    /* ---- WiFi setup ---- */
    wifi_init();
    _delay_ms(500);
    wifi_command_AT();
    wifi_command_disable_echo();
    wifi_command("AT+CWAUTOCONN=0", 2);
    wifi_command("AT+CWQAP", 2);
    _delay_ms(500);

    /* Switch to AP+STA mode so the module can host its own network      */
    wifi_command("AT+CWMODE=2", 3);
    _delay_ms(1500);

    /* Create open access-point on channel 6                             */
    uart_send_string_blocking(UART2_ID,
                              "AT+CWSAP=\"" DATA_AP_SSID "\",\"\",6,0\r\n");
    _delay_ms(3000);

    printf_P(PSTR("[DataServer] AP ready: %s\n"), DATA_AP_SSID);
    printf_P(PSTR("[DataServer] Connect PC to that WiFi, then run plantify_logger.py\n"));

    /* Enable multiple connections + open TCP server on DATA_SERVER_PORT */
    wifi_command("AT+CIPMUX=1", 2);
    {
        char srv_cmd[40];
        snprintf(srv_cmd, sizeof(srv_cmd),
                 "AT+CIPSERVER=1,%u\r\n", DATA_SERVER_PORT);
        uart_send_string_blocking(UART2_ID, srv_cmd);
        _delay_ms(1000);
    }

    /* Register receive callback (re-uses TCP server receive path)       */
    wifi_command_start_TCP_server(_on_client_data, _rx_buf, RX_BUF_SIZE);

    printf_P(PSTR("[DataServer] Listening on 192.168.4.1:%u\n"),
             DATA_SERVER_PORT);

    /* Blink LED 3 to show we are in data-server mode                   */
    led_on(3);

    /* ---- Main loop: send data every SEND_INTERVAL_MS ---- */
    uint32_t tick = 0;
    while (1)
    {
        /* Service any incoming byte from the client (ACK / disconnect)  */
        if (_rx_ready)
        {
            _rx_ready = 0;
            _rx_buf[0] = '\0'; /* flush buffer */
        }

        _send_sensor_json();

        /* Simple delay loop – _delay_ms() max is ~262ms on 16 MHz      */
        for (uint16_t i = 0; i < (SEND_INTERVAL_MS / 100); i++)
            _delay_ms(100);

        tick++;

        /* Heartbeat blink every 5 seconds                               */
        if (tick % 5 == 0)
        {
            led_off(3);
            _delay_ms(50);
            led_on(3);
        }
    }
}