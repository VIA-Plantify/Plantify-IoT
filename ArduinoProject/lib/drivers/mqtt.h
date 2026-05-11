#ifndef MQTT_H
#define MQTT_H

#include <stdint.h>

#define MQTT_BROKER_HOST      "676d176f8a7b4b33ab4d8f1733d48d3d.s1.eu.hivemq.cloud"
#define MQTT_BROKER_PORT      8883
#define MQTT_USERNAME         "Plantify"
#define MQTT_PASSWORD         "Suka1234"
#define MQTT_CLIENT_ID        "plantpot_001"
#define PING_INTERVAL_S       45
#define MQTT_DEFAULT_CLIENT_ID "mega2560_client"

uint8_t mqtt_raw_connect_with_credentials(char *ssid, char *password);
uint8_t mqtt_raw_publish(const char *payload);
void mqtt_handle_incoming(void);
void mqtt_send_ping(void);
uint8_t mqtt_is_connected(void);
void mqtt_tick(uint8_t elapsed_seconds);

#endif