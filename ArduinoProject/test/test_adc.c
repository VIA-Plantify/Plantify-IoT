#include "adc.h"

void setUp(void) {}
void tearDown(void) {}

/* --- adc_create --- */

void test_adc_create_valid_channel_and_reference(void) {
    ADC_Error_t result = adc_create(ADC_PK0, ADC_REF_5V);
    TEST_ASSERT_EQUAL(ADC_OK, result);
}

void test_adc_create_all_channels(void) {
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK0, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK1, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK2, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK3, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK4, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK7, ADC_REF_5V));
}

void test_adc_create_all_references(void) {
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK0, ADC_REF_5V));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK0, ADC_REF_1V1));
    TEST_ASSERT_EQUAL(ADC_OK, adc_create(ADC_PK0, ADC_REF_2V56));
}

void test_adc_create_invalid_channel(void) {
    ADC_Error_t result = adc_create((ADC_Channel_t)99, ADC_REF_5V);
    TEST_ASSERT_EQUAL(ADC_ERROR_INVALID_CHANNEL, result);
}

void test_adc_create_invalid_reference(void) {
    ADC_Error_t result = adc_create(ADC_PK0, (ADC_Reference_t)99);
    TEST_ASSERT_EQUAL(ADC_ERROR_INVALID_REFERENCE, result);
}

/* --- adc_measure --- */

void test_adc_measure_returns_value_in_range(void) {
    adc_create(ADC_PK0, ADC_REF_5V);
    uint16_t value = adc_measure(ADC_PK0);
    TEST_ASSERT_LESS_OR_EQUAL(1023, value);
}

void test_adc_measure_invalid_channel_returns_max(void) {
    uint16_t value = adc_measure((ADC_Channel_t)99);
    TEST_ASSERT_EQUAL(UINT16_MAX, value);
}

void test_adc_measure_consistent_reads(void) {
    adc_create(ADC_PK0, ADC_REF_5V);
    uint16_t a = adc_measure(ADC_PK0);
    uint16_t b = adc_measure(ADC_PK0);
    // Both readings must be in valid 10-bit range
    TEST_ASSERT_LESS_OR_EQUAL(1023, a);
    TEST_ASSERT_LESS_OR_EQUAL(1023, b);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_adc_create_valid_channel_and_reference);
    RUN_TEST(test_adc_create_all_channels);
    RUN_TEST(test_adc_create_all_references);
    RUN_TEST(test_adc_create_invalid_channel);
    RUN_TEST(test_adc_create_invalid_reference);
    RUN_TEST(test_adc_measure_returns_value_in_range);
    RUN_TEST(test_adc_measure_invalid_channel_returns_max);
    RUN_TEST(test_adc_measure_consistent_reads);
    return UNITY_END();
}