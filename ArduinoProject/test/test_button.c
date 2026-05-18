#include "button.h"

void setUp(void) {
    button_init();
}
void tearDown(void) {}

/* --- button_get --- */

void test_button_get_valid_buttons_return_bool(void) {
    // Just verify it returns without crashing and gives a bool value
    bool b1 = button_get(1);
    bool b2 = button_get(2);
    bool b3 = button_get(3);
    TEST_ASSERT(b1 == true || b1 == false);
    TEST_ASSERT(b2 == true || b2 == false);
    TEST_ASSERT(b3 == true || b3 == false);
}

void test_button_get_invalid_number_returns_false(void) {
    // Button 0 and 4+ are out of range — should return false, not crash
    TEST_ASSERT_FALSE(button_get(0));
    TEST_ASSERT_FALSE(button_get(4));
    TEST_ASSERT_FALSE(button_get(255));
}

/* --- button_scan --- */

void test_button_scan_returns_zero_or_valid_number(void) {
    uint8_t pressed = button_scan();
    // Must be 0 (none pressed) or 1-3 (valid button)
    TEST_ASSERT(pressed <= 3);
}

void test_button_scan_consistent_with_button_get(void) {
    uint8_t scanned = button_scan();
    if (scanned > 0) {
        // If scan reports a button, button_get must agree
        TEST_ASSERT_TRUE(button_get(scanned));
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_button_get_valid_buttons_return_bool);
    RUN_TEST(test_button_get_invalid_number_returns_false);
    RUN_TEST(test_button_scan_returns_zero_or_valid_number);
    RUN_TEST(test_button_scan_consistent_with_button_get);
    return UNITY_END();
}