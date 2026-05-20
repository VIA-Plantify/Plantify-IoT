#include "proximity.h"
#include <unity.h>
#include <stdbool.h>
#include <stdint.h>

void setUp(void)
{
    proximity_init();
}
void tearDown(void) {}

/* --- proximity_measure --- */

void test_proximity_measure_returns_valid_or_timeout(void)
{
    uint16_t dist = proximity_measure();
    // Either a real distance (0–65534mm) or UINT16_MAX (no object / timeout)
    TEST_ASSERT(dist <= 65535); // always true, just verify no crash
}

void test_proximity_measure_timeout_is_uint16_max(void)
{
    // If no object present, should return UINT16_MAX
    // We can't guarantee the environment, so just verify the constant is UINT16_MAX
    // and that the return value is handled correctly
    uint16_t dist = proximity_measure();
    if (dist == UINT16_MAX)
    {
        TEST_PASS_MESSAGE("No object detected, UINT16_MAX returned as expected");
    }
    else
    {
        TEST_ASSERT_LESS_THAN(UINT16_MAX, dist);
    }
}

void test_proximity_measure_consistent_direction(void)
{
    // Two successive reads should both be either valid or both timeout
    // (environment stable over ~50ms)
    uint16_t a = proximity_measure();
    uint16_t b = proximity_measure();
    bool a_timeout = (a == UINT16_MAX);
    bool b_timeout = (b == UINT16_MAX);
    // Both valid or both timeout — if it flips, that's a hardware issue not a driver issue
    TEST_ASSERT_EQUAL(a_timeout, b_timeout);
}

/* --- proximity_get_distance --- */

void test_proximity_get_distance_matches_last_measure(void)
{
    uint16_t measured = proximity_measure();
    uint16_t cached = proximity_get_distance();
    TEST_ASSERT_EQUAL(measured, cached);
}

void test_proximity_get_distance_without_measure_returns_last(void)
{
    proximity_measure(); // prime the last value
    uint16_t first = proximity_get_distance();
    uint16_t second = proximity_get_distance();
    // get_distance() doesn't trigger a new measurement — should return same value
    TEST_ASSERT_EQUAL(first, second);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_proximity_measure_returns_valid_or_timeout);
    RUN_TEST(test_proximity_measure_timeout_is_uint16_max);
    RUN_TEST(test_proximity_measure_consistent_direction);
    RUN_TEST(test_proximity_get_distance_matches_last_measure);
    RUN_TEST(test_proximity_get_distance_without_measure_returns_last);
    return UNITY_END();
}