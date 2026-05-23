#pragma once

#include <stdint.h>

typedef enum
{
    DHT11_OK,   /**< Reading successful */
    DHT11_FAIL, /**< Checksum mismatch or insufficient bits */
} DHT11_ERROR_MESSAGE_t;

/**
 * @brief Read temperature and humidity from DHT11 sensor.
 *
 * Can only be called once every 2 seconds to avoid sensor lock-up.
 * Pass NULL for any value you don't need.
 *
 * @param humidity_integer    Pointer to store humidity whole part
 * @param humidity_decimal    Pointer to store humidity decimal part
 * @param temperature_integer Pointer to store temperature whole part
 * @param temperature_decimal Pointer to store temperature decimal part
 * @return DHT11_OK on success, DHT11_FAIL otherwise
 */
DHT11_ERROR_MESSAGE_t dht11_get(uint8_t *humidity_integer, uint8_t *humidity_decimal,
                                uint8_t *temperature_integer, uint8_t *temperature_decimal);
DHT11_ERROR_MESSAGE_t dht11_get_reliable(uint8_t *humidity_integer, uint8_t *humidity_decimal,
                                         uint8_t *temperature_integer, uint8_t *temperature_decimal);