#pragma once

#include <avr/io.h>
#include <stdbool.h>

#define PUMP_PIN PC7

void pump_init(void);

void pump_on(void);

void pump_off(void);

bool pump_is_running(void);

void pump_run_for(uint32_t miliseconds);