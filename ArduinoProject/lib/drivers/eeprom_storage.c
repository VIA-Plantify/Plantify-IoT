#include "eeprom_storage.h"
#include <avr/eeprom.h>

void save_credentials(char *ssid, char *password)
{
    eeprom_write_block(ssid,     (void *)EEPROM_SSID_ADDR,     32);
    eeprom_write_block(password, (void *)EEPROM_PASSWORD_ADDR, 64);
}

void load_credentials(char *ssid, char *password)
{
    eeprom_read_block(ssid,     (void *)EEPROM_SSID_ADDR,     32);
    eeprom_read_block(password, (void *)EEPROM_PASSWORD_ADDR, 64);
}