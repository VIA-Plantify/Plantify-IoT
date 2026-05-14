#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

// ─── WiFi Configuration ───────────────────────────────────────────────────────
#define WIFI_SSID      "Ricky"
#define WIFI_PASSWORD  "r89uuios"

// ─── Broker Configuration ─────────────────────────────────────────────────────
#define MQTT_BROKER_HOST  "46.62.140.54"
#define MQTT_BROKER_PORT  1883
#define MQTT_USERNAME     "Plantify"
#define MQTT_PASSWORD     "Password123"

// ─── Ping Configuration ───────────────────────────────────────────────────────
#define PING_INTERVAL_S   45

// ─── RX buffer ────────────────────────────────────────────────────────────────
#define MQTT_RX_BUF_SIZE  256

// ─── Public API ───────────────────────────────────────────────────────────────
uint8_t mqtt_raw_connect(void);
uint8_t mqtt_raw_publish(const char *payload);
void    mqtt_handle_incoming(void);
void    mqtt_send_ping(void);
uint8_t mqtt_is_connected(void);
void    mqtt_tick(uint16_t elapsed_seconds);
char   *mqtt_get_device_mac(void);

#endif