#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

// ─── WiFi Configuration ───────────────────────────────────────────────────────
#define WIFI_SSID      "Ricky"
#define WIFI_PASSWORD  "r89uuios"

// ─── Broker Configuration ─────────────────────────────────────────────────────
#define MQTT_BROKER_HOST  "676d176f8a7b4b33ab4d8f1733d48d3d.s1.eu.hivemq.cloud"
#define MQTT_BROKER_PORT  8883
#define MQTT_USERNAME     "Plantify"
#define MQTT_PASSWORD     "Password123"
#define MQTT_CLIENT_ID    "mega2560_client"
#define MQTT_PUB_TOPIC    "arduino"
#define MQTT_SUB_TOPIC    "iot/mega/commands"
// ─── Ping Configuration ───────────────────────────────────────────────────────
#define PING_INTERVAL_S   45

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief Connect to WiFi, open SSL, send MQTT CONNECT and SUBSCRIBE.
 * @return 1 on success, 0 on failure.
 */
uint8_t mqtt_raw_connect(void);

/**
 * @brief Publish a JSON payload to MQTT_PUB_TOPIC.
 * @param payload Null-terminated string to publish.
 * @return 1 on success, 0 on failure.
 */
uint8_t mqtt_raw_publish(const char *payload);

/**
 * @brief Process any incoming MQTT messages.
 *        Call this periodically to stay responsive.
 */
void mqtt_handle_incoming(void);

/**
 * @brief Send a MQTT PINGREQ to keep the connection alive.
 */
void mqtt_send_ping(void);

/**
 * @brief Returns 1 if MQTT is currently connected, 0 otherwise.
 */
uint8_t mqtt_is_connected(void);

/**
 * @brief Should be called every ~3 seconds to track ping interval.
 *        Sends a ping automatically when PING_INTERVAL_S is reached.
 * @param elapsed_seconds How many seconds have passed since last call.
 */
void mqtt_tick(uint8_t elapsed_seconds);

#endif // MQTT_H