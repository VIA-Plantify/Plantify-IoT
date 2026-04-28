#pragma once
#include <stdint.h>

uint8_t menu(void);
void pir_callback(void);
void led2_callback(uint8_t id);
void start_stop_timer(uint8_t id);
void wifi_line_callback(void);
int interactive_demo(void);