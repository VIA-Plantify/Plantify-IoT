#include "pump.h"
#include <unity.h>

void setUp(void) {
    pump_init();
    pump_off();  // always start from known off state
}
void tearDown(void) {
    pump_off();  // safety: never leave pump running after a test
}

/* --- pump_on / pump_off --- */

void test_pump_on_sets_running(void) {
    pump_on();
    TEST_ASSERT_TRUE(pump_is_running());
}

void test_pump_off_clears_running(void) {
    pump_on();
    pump_off();
    TEST_ASSERT_FALSE(pump_is_running());
}

void test_pump_initially_off(void) {
    // After init + off in setUp, pump must be off
    TEST_ASSERT_FALSE(pump_is_running());
}

void test_pump_on_idempotent(void) {
    // Calling pump_on() twice should not crash or change state unexpectedly
    pump_on();
    pump_on();
    TEST_ASSERT_TRUE(pump_is_running());
}

void test_pump_off_idempotent(void) {
    pump_off();
    pump_off();
    TEST_ASSERT_FALSE(pump_is_running());
}

/* --- pump_is_running --- */

void test_pump_is_running_reflects_state(void) {
    pump_on();
    TEST_ASSERT_TRUE(pump_is_running());
    pump_off();
    TEST_ASSERT_FALSE(pump_is_running());
    pump_on();
    TEST_ASSERT_TRUE(pump_is_running());
}

/* --- pump_run_for --- */

void test_pump_run_for_turns_off_after_duration(void) {
    // Run for 50ms — short enough not to slow tests much
    pump_run_for(50);
    TEST_ASSERT_FALSE(pump_is_running());
}

void test_pump_run_for_zero_does_not_leave_pump_on(void) {
    pump_run_for(0);
    TEST_ASSERT_FALSE(pump_is_running());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pump_on_sets_running);
    RUN_TEST(test_pump_off_clears_running);
    RUN_TEST(test_pump_initially_off);
    RUN_TEST(test_pump_on_idempotent);
    RUN_TEST(test_pump_off_idempotent);
    RUN_TEST(test_pump_is_running_reflects_state);
    RUN_TEST(test_pump_run_for_turns_off_after_duration);
    RUN_TEST(test_pump_run_for_zero_does_not_leave_pump_on);
    return UNITY_END();
}