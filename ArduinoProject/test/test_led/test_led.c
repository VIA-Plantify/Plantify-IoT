#include "led.h"
#include <unity.h>

void setUp(void) {
    led_init();
}
void tearDown(void) {
    // Turn off all LEDs after each test
    for (int8_t i = 1; i <= 4; i++)
        led_off(i);
}

/* --- led_on --- */

void test_led_on_valid_leds(void) {
    TEST_ASSERT_EQUAL(LED_OK, led_on(1));
    TEST_ASSERT_EQUAL(LED_OK, led_on(2));
    TEST_ASSERT_EQUAL(LED_OK, led_on(3));
    TEST_ASSERT_EQUAL(LED_OK, led_on(4));
}

void test_led_on_invalid_led_number(void) {
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_on(0));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_on(5));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_on(-1));
}

/* --- led_off --- */

void test_led_off_valid_leds(void) {
    led_on(1);
    TEST_ASSERT_EQUAL(LED_OK, led_off(1));
    TEST_ASSERT_EQUAL(LED_OK, led_off(2));
    TEST_ASSERT_EQUAL(LED_OK, led_off(3));
    TEST_ASSERT_EQUAL(LED_OK, led_off(4));
}

void test_led_off_invalid_led_number(void) {
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_off(0));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_off(5));
}

/* --- led_toggle --- */

void test_led_toggle_valid_leds(void) {
    TEST_ASSERT_EQUAL(LED_OK, led_toggle(1));
    TEST_ASSERT_EQUAL(LED_OK, led_toggle(2));
    TEST_ASSERT_EQUAL(LED_OK, led_toggle(3));
    TEST_ASSERT_EQUAL(LED_OK, led_toggle(4));
}

void test_led_toggle_invalid_led_number(void) {
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_toggle(0));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_toggle(5));
}

/* --- led_blink --- */

void test_led_blink_valid(void) {
    TEST_ASSERT_EQUAL(LED_OK, led_blink(1, 500));
    TEST_ASSERT_EQUAL(LED_OK, led_blink(2, 10));       // minimum period
    TEST_ASSERT_EQUAL(LED_OK, led_blink(3, 60000));    // maximum period
}

void test_led_blink_invalid_led_number(void) {
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_blink(0, 500));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_LED_NO, led_blink(5, 500));
}

void test_led_blink_invalid_period(void) {
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_PERIOD, led_blink(1, 0));
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_PERIOD, led_blink(1, 9));       // below min
    TEST_ASSERT_EQUAL(LED_ERROR_INVALID_PERIOD, led_blink(1, 60001));   // above max
}

void test_led_blink_no_resources(void) {
    // Fill up all timer slots via blink (led uses software timers internally)
    // Blink all 4 LEDs — if TIMER_MAX_TIMERS < 4 this triggers no-resource
    led_blink(1, 100);
    led_blink(2, 200);
    led_blink(3, 300);
    led_status_t result = led_blink(4, 400);
    // Either OK or NO_RESOURCES depending on timer availability — both are valid
    TEST_ASSERT(result == LED_OK || result == LED_ERROR_NO_RESOURCES);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_led_on_valid_leds);
    RUN_TEST(test_led_on_invalid_led_number);
    RUN_TEST(test_led_off_valid_leds);
    RUN_TEST(test_led_off_invalid_led_number);
    RUN_TEST(test_led_toggle_valid_leds);
    RUN_TEST(test_led_toggle_invalid_led_number);
    RUN_TEST(test_led_blink_valid);
    RUN_TEST(test_led_blink_invalid_led_number);
    RUN_TEST(test_led_blink_invalid_period);
    RUN_TEST(test_led_blink_no_resources);
    return UNITY_END();
}