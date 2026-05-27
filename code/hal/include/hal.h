#pragma once

#include "defs.h"

typedef enum pin_mode : unsigned char {
    PIN_MODE_INACTIVE = 0,
    PIN_MODE_OUTPUT = 1,
    PIN_MODE_INPUT = 2,
} pin_mode;

const char *pin_mode_to_str(pin_mode);

typedef int pin_id_logical;

typedef struct Hal Hal;

void Hal_init_begin(Hal *hal);

void Hal_init_enable_gpio_output_logical(Hal *hal, const pin_id_logical pin);

void Hal_init_end(Hal *hal);

void Hal_set_gpio_logical(Hal *hal, const pin_id_logical pin, const bool state);

void Hal_blocking_sleep_ms(Hal *, int ms);

bool Hal_get_gpio(const Hal *hal, const pin_id_logical pin);
