#pragma once

#include "defs.h"

typedef int pin_id_logical;

typedef enum {
  PIN_MODE_INACTIVE = 0,
  PIN_MODE_OUTPUT = 1,
  PIN_MODE_INPUT = 2,
} pin_mode;

const char* pin_mode_to_str(pin_mode);

typedef struct Hal Hal;

#if defined(HAL_BACKEND_mock)
#include "mock.h"
#elif defined(HAL_BACKEND_teensy41)
#include "teensy41.h"
#else
#error "HAL_BACKEND must be defined as 'mock' or 'teensy41'"
#endif

void Hal_init_begin(Hal* hal);

void Hal_init_enable_gpio_output_logical(Hal* hal, const pin_id_logical pin);

void Hal_init_end(Hal* hal);

void Hal_set_gpio_logical(Hal* hal, const pin_id_logical pin, const bool state);

void Hal_blocking_sleep_ms(Hal*, int ms);

bool Hal_get_gpio(const Hal* hal, const pin_id_logical pin);
