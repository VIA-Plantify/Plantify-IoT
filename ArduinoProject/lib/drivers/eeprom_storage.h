#pragma once
#include <stdint.h>

#define EEPROM_SSID_ADDR      0
#define EEPROM_PASSWORD_ADDR  32

void save_credentials(char *ssid, char *password);
void load_credentials(char *ssid, char *password);