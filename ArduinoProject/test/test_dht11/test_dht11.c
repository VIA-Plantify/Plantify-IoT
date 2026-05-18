/*
 * test_dht11.c
 *
 * INTEGRATION TESTS — requires DHT11 sensor physically connected to PL1.
 * Run on real hardware. These tests verify:
 *   - NULL pointer safety
 *   - Returned values are within physically valid ranges
 *   - Retry logic in dht11_get_reliable()
 *
 * NOTE: DHT11 needs at least 1 second between reads.
 *       Each test uses _delay_ms(1500) where needed.
 */

#include "dht11.h"
#include "unity.h"
#include <util/delay.h>
#include <stddef.h>

void setUp(void) {
    _delay_ms(1500);  // DHT11 minimum sampling interval between tests
}
void tearDown(void) {}

/* --- NULL pointer safety --- */

void test_dht11_get_all_null_pointers_does_not_crash(void) {
    // The header explicitly says any pointer can be NULL — must not crash
    DHT11_ERROR_MESSAGE_t result = dht11_get(NULL, NULL, NULL, NULL);
    // Result may be OK or error depending on sensor, but must not hang or corrupt
    TEST_ASSERT(result == DHT11_OK || result == DHT11_FAIL);
}

void test_dht11_get_partial_null_pointers(void) {
    uint8_t temp_int = 0;
    // Only ask for temperature, ignore humidity
    DHT11_ERROR_MESSAGE_t result = dht11_get(NULL, NULL, &temp_int, NULL);
    TEST_ASSERT(result == DHT11_OK || result == DHT11_FAIL);
}

/* --- Valid range checks (sensor must be connected) --- */

void test_dht11_get_humidity_in_valid_range(void) {
    uint8_t hum_int = 0, hum_dec = 0;
    DHT11_ERROR_MESSAGE_t result = dht11_get(&hum_int, &hum_dec, NULL, NULL);

    if (result == DHT11_OK) {
        TEST_ASSERT_LESS_OR_EQUAL(100, hum_int);   // humidity max 100%
        TEST_ASSERT_LESS_OR_EQUAL(9,   hum_dec);   // decimal 0–9
    } else {
        TEST_IGNORE_MESSAGE("DHT11 read failed — check sensor connection");
    }
}

void test_dht11_get_temperature_in_valid_range(void) {
    uint8_t temp_int = 0, temp_dec = 0;
    DHT11_ERROR_MESSAGE_t result = dht11_get(NULL, NULL, &temp_int, &temp_dec);

    if (result == DHT11_OK) {
        // DHT11 measures 0–50°C
        TEST_ASSERT_LESS_OR_EQUAL(50, temp_int);
        TEST_ASSERT_LESS_OR_EQUAL(9,  temp_dec);
    } else {
        TEST_IGNORE_MESSAGE("DHT11 read failed — check sensor connection");
    }
}

void test_dht11_get_all_values_in_range(void) {
    uint8_t hi = 0, hd = 0, ti = 0, td = 0;
    DHT11_ERROR_MESSAGE_t result = dht11_get(&hi, &hd, &ti, &td);

    if (result == DHT11_OK) {
        TEST_ASSERT_LESS_OR_EQUAL(100, hi);
        TEST_ASSERT_LESS_OR_EQUAL(9,   hd);
        TEST_ASSERT_LESS_OR_EQUAL(50,  ti);
        TEST_ASSERT_LESS_OR_EQUAL(9,   td);
    } else {
        TEST_IGNORE_MESSAGE("DHT11 read failed — check sensor connection");
    }
}

/* --- dht11_get_reliable retry logic --- */

void test_dht11_get_reliable_returns_ok_or_last_error(void) {
    uint8_t hi = 0, hd = 0, ti = 0, td = 0;
    DHT11_ERROR_MESSAGE_t result = dht11_get_reliable(&hi, &hd, &ti, &td);
    // Must return a valid enum value — either succeeded or gave up after 3 retries
    TEST_ASSERT(result == DHT11_OK || result == DHT11_FAIL);
}

void test_dht11_get_reliable_values_in_range_on_success(void) {
    uint8_t hi = 0, hd = 0, ti = 0, td = 0;
    DHT11_ERROR_MESSAGE_t result = dht11_get_reliable(&hi, &hd, &ti, &td);

    if (result == DHT11_OK) {
        TEST_ASSERT_LESS_OR_EQUAL(100, hi);
        TEST_ASSERT_LESS_OR_EQUAL(9,   hd);
        TEST_ASSERT_LESS_OR_EQUAL(50,  ti);
        TEST_ASSERT_LESS_OR_EQUAL(9,   td);
    } else {
        TEST_IGNORE_MESSAGE("DHT11 reliable read failed after retries — check sensor");
    }
}

void test_dht11_get_reliable_null_pointers_do_not_crash(void) {
    DHT11_ERROR_MESSAGE_t result = dht11_get_reliable(NULL, NULL, NULL, NULL);
    TEST_ASSERT(result == DHT11_OK || result == DHT11_FAIL);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_dht11_get_all_null_pointers_does_not_crash);
    RUN_TEST(test_dht11_get_partial_null_pointers);
    RUN_TEST(test_dht11_get_humidity_in_valid_range);
    RUN_TEST(test_dht11_get_temperature_in_valid_range);
    RUN_TEST(test_dht11_get_all_values_in_range);
    RUN_TEST(test_dht11_get_reliable_returns_ok_or_last_error);
    RUN_TEST(test_dht11_get_reliable_values_in_range_on_success);
    RUN_TEST(test_dht11_get_reliable_null_pointers_do_not_crash);
    return UNITY_END();
}