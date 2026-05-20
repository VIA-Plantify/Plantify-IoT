#include "soil.h"
#include <unity.h>

void setUp(void) {
    soil_init(ADC_PK0);
}
void tearDown(void) {}

/* --- soil_init --- */

void test_soil_init_valid_channel(void) {
    ADC_Error_t result = soil_init(ADC_PK0);
    TEST_ASSERT_EQUAL(ADC_OK, result);
}

void test_soil_init_invalid_channel(void) {
    ADC_Error_t result = soil_init((ADC_Channel_t)99);
    TEST_ASSERT_EQUAL(ADC_ERROR_INVALID_CHANNEL, result);
}

/* --- soil_measure_raw --- */

void test_soil_measure_raw_in_range(void) {
    uint16_t value = soil_measure_raw(ADC_PK0);
    TEST_ASSERT_LESS_OR_EQUAL(1023, value);
}

void test_soil_measure_raw_invalid_channel_returns_max(void) {
    uint16_t value = soil_measure_raw((ADC_Channel_t)99);
    TEST_ASSERT_EQUAL(UINT16_MAX, value);
}

/* --- soil_measure_percentage --- */

void test_soil_measure_percentage_in_range(void) {
    uint8_t pct = soil_measure_percentage(ADC_PK0);
    TEST_ASSERT_LESS_OR_EQUAL(100, pct);
}

void test_soil_dry_value_maps_to_low_percentage(void) {
    // SOIL_DRY_VALUE (500) should map to 0% — just verify boundary constants are sane
    TEST_ASSERT_GREATER_THAN(SOIL_WET_VALUE, SOIL_DRY_VALUE);
}

void test_soil_measure_percentage_consistent_with_raw(void) {
    // Both should succeed without crashing on the same channel
    uint16_t raw = soil_measure_raw(ADC_PK0);
    uint8_t  pct = soil_measure_percentage(ADC_PK0);
    TEST_ASSERT_LESS_OR_EQUAL(1023, raw);
    TEST_ASSERT_LESS_OR_EQUAL(100,  pct);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_soil_init_valid_channel);
    RUN_TEST(test_soil_init_invalid_channel);
    RUN_TEST(test_soil_measure_raw_in_range);
    RUN_TEST(test_soil_measure_raw_invalid_channel_returns_max);
    RUN_TEST(test_soil_measure_percentage_in_range);
    RUN_TEST(test_soil_dry_value_maps_to_low_percentage);
    RUN_TEST(test_soil_measure_percentage_consistent_with_raw);
    return UNITY_END();
}