#pragma once

#include <string.h>

#include "hal.h"

#define PIN_LED_LOGICAL 0

#define MAX_MOCK_HAL_PIN_COUNT 64

typedef struct Hal {
    // DDR(Artur): Canary, is set to the pointer the HAL itself after
    // initialization. This is needed to avoid accidential reinitializations or
    // movements of the HAL.
    Hal *canary;
    pin_mode pin_modes[MAX_MOCK_HAL_PIN_COUNT];
    bool pin_values[MAX_MOCK_HAL_PIN_COUNT];
} Hal;
