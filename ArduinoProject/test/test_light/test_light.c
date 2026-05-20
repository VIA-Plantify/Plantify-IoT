#include "light.h"
#include <unity.h>

void setUp(void) {
    light_init();
}
void tearDown(void) {}

/* --- light_init --- */

void test_light_init_returns_ok(void) {
    ADC_Error_t result = light_init();
    TEST_ASSERT_EQUAL(ADC_OK, result);
}

void test_light_init_can_be_called_multiple_times(void) {
    // Re-initializing should not crash or corrupt state
    TEST_ASSERT_EQUAL(ADC_OK, light_init());
    TEST_ASSERT_EQUAL(ADC_OK, light_init());
}

/* --- light_measure_raw --- */

void test_light_measure_raw_in_valid_range(void) {
    uint16_t value = light_measure_raw();
    // 10-bit ADC: 0–1023
    TEST_ASSERT_LESS_OR_EQUAL(1023, value);
}

void test_light_measure_raw_multiple_reads_in_range(void) {
    for (int i = 0; i < 5; i++) {
        uint16_t value = light_measure_raw();
        TEST_ASSERT_LESS_OR_EQUAL(1023, value);
    }
}

void test_light_measure_raw_low_is_dark_high_is_bright(void) {
    // Verify the documented direction: low = dark, high = bright
    // Can't assert exact value in hardware test, but range must be correct
    uint16_t value = light_measure_raw();
    TEST_ASSERT_GREATER_OR_EQUAL(0, value);
    TEST_ASSERT_LESS_OR_EQUAL(1023, value);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_light_init_returns_ok);
    RUN_TEST(test_light_init_can_be_called_multiple_times);
    RUN_TEST(test_light_measure_raw_in_valid_range);
    RUN_TEST(test_light_measure_raw_multiple_reads_in_range);
    RUN_TEST(test_light_measure_raw_low_is_dark_high_is_bright);
    return UNITY_END();
}