#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>

// ─── Pin Configuration ────────────────────────────────────────────────────────
#define DATA_BIT    PL1
#define DATA_PIN    PINL
#define DATA_DDR    DDRL
#define DATA_PORT   PORTL

// ─── Timing Constants ─────────────────────────────────────────────────────────
#define DHT11_START_SIGNAL_MS   18
#define DHT11_RESPONSE_WAIT_US  40
#define DHT11_BIT_THRESHOLD_US  26
#define DHT11_TIMEOUT_COUNT     255
#define DHT11_EXPECTED_BITS     40
#define DHT11_SKIP_TRANSITIONS  4
#define MAX_TIMINGS             85

// ─── Error Codes ──────────────────────────────────────────────────────────────
typedef enum {
    DHT11_OK,       // Reading successful
    DHT11_FAIL,     // Checksum mismatch or insufficient bits read
    DHT11_TIMEOUT   // Pin did not transition in time
} DHT11_ERROR_MESSAGE_t;

// ─── Public API ───────────────────────────────────────────────────────────────

/**
 * @brief Read temperature and humidity from DHT11 sensor (single attempt).
 *
 * @param humidity_integer    Pointer to store humidity whole part (e.g. 55 for 55.3%)
 * @param humidity_decimal    Pointer to store humidity decimal part
 * @param temperature_integer Pointer to store temperature whole part (e.g. 23 for 23.1°C)
 * @param temperature_decimal Pointer to store temperature decimal part
 *
 * @return DHT11_OK      on success
 *         DHT11_FAIL    on checksum error or too few bits
 *         DHT11_TIMEOUT if the sensor stopped responding mid-read
 *
 * @note Any pointer can be NULL — that value will simply be skipped.
 */
DHT11_ERROR_MESSAGE_t dht11_get(
    uint8_t* humidity_integer,
    uint8_t* humidity_decimal,
    uint8_t* temperature_integer,
    uint8_t* temperature_decimal
);

/**
 * @brief Read temperature and humidity with up to 3 retry attempts.
 *
 * Retries on failure with a 1-second delay between attempts (DHT11 minimum
 * sampling interval). Prefer this over dht11_get() in production code.
 *
 * @param humidity_integer    Pointer to store humidity whole part
 * @param humidity_decimal    Pointer to store humidity decimal part
 * @param temperature_integer Pointer to store temperature whole part
 * @param temperature_decimal Pointer to store temperature decimal part
 *
 * @return DHT11_OK if any attempt succeeded, otherwise the last error code.
 */
DHT11_ERROR_MESSAGE_t dht11_get_reliable(
    uint8_t* humidity_integer,
    uint8_t* humidity_decimal,
    uint8_t* temperature_integer,
    uint8_t* temperature_decimal
);

#endif // DHT11_H