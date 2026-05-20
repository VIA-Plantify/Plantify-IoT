#include "uart.h"
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

/* --- uart_init --- */

void test_uart_init_valid_uart0(void) {
    uart_t result = uart_init(UART0_ID, 115200, NULL, 0);
    TEST_ASSERT_EQUAL(UART_OK, result);
}

void test_uart_init_valid_baud_rates(void) {
    TEST_ASSERT_EQUAL(UART_OK, uart_init(UART0_ID, 9600,   NULL, 0));
    TEST_ASSERT_EQUAL(UART_OK, uart_init(UART0_ID, 19200,  NULL, 0));
    TEST_ASSERT_EQUAL(UART_OK, uart_init(UART0_ID, 38400,  NULL, 0));
    TEST_ASSERT_EQUAL(UART_OK, uart_init(UART0_ID, 57600,  NULL, 0));
    TEST_ASSERT_EQUAL(UART_OK, uart_init(UART0_ID, 115200, NULL, 0));
}

void test_uart_init_invalid_baud_rate(void) {
    uart_t result = uart_init(UART0_ID, 99999, NULL, 0);
    TEST_ASSERT_EQUAL(UART_ERROR_INVALID_BAUD_RATE, result);
}

void test_uart_init_invalid_uart_id(void) {
    uart_t result = uart_init((uart_id_t)99, 115200, NULL, 0);
    TEST_ASSERT_EQUAL(UART_ERROR_INVALID_ID, result);
}

void test_uart_init_with_rx_buffer(void) {
    uart_t result = uart_init(UART0_ID, 115200, NULL, 32);
    TEST_ASSERT_EQUAL(UART_OK, result);
}

/* --- uart_write_byte --- */

void test_uart_write_byte_valid(void) {
    uart_init(UART0_ID, 115200, NULL, 0);
    uart_t result = uart_write_byte(UART0_ID, 'A');
    TEST_ASSERT_EQUAL(UART_OK, result);
}

void test_uart_write_byte_invalid_id(void) {
    uart_t result = uart_write_byte((uart_id_t)99, 'A');
    TEST_ASSERT_EQUAL(UART_ERROR_INVALID_ID, result);
}

/* --- uart_read_byte --- */

void test_uart_read_byte_no_data_returns_no_data(void) {
    uart_init(UART0_ID, 115200, NULL, 32);
    uint8_t byte = 0;
    uart_t result = uart_read_byte(UART0_ID, &byte);
    TEST_ASSERT_EQUAL(UART_NO_DATA_AVAILABLE, result);
}

void test_uart_read_byte_invalid_id(void) {
    uint8_t byte = 0;
    uart_t result = uart_read_byte((uart_id_t)99, &byte);
    TEST_ASSERT_EQUAL(UART_ERROR_INVALID_ID, result);
}

/* --- uart_write_bytes --- */

void test_uart_write_bytes_valid(void) {
    uart_init(UART0_ID, 115200, NULL, 0);
    uint8_t data[] = { 'H', 'i', '\n' };
    uart_t result = uart_write_bytes(UART0_ID, data, sizeof(data));
    TEST_ASSERT_EQUAL(UART_OK, result);
}

void test_uart_write_bytes_invalid_id(void) {
    uint8_t data[] = { 'X' };
    uart_t result = uart_write_bytes((uart_id_t)99, data, 1);
    TEST_ASSERT_EQUAL(UART_ERROR_INVALID_ID, result);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_uart_init_valid_uart0);
    RUN_TEST(test_uart_init_valid_baud_rates);
    RUN_TEST(test_uart_init_invalid_baud_rate);
    RUN_TEST(test_uart_init_invalid_uart_id);
    RUN_TEST(test_uart_init_with_rx_buffer);
    RUN_TEST(test_uart_write_byte_valid);
    RUN_TEST(test_uart_write_byte_invalid_id);
    RUN_TEST(test_uart_read_byte_no_data_returns_no_data);
    RUN_TEST(test_uart_read_byte_invalid_id);
    RUN_TEST(test_uart_write_bytes_valid);
    RUN_TEST(test_uart_write_bytes_invalid_id);
    return UNITY_END();
}