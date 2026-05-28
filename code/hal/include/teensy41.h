#pragma once

#define TEENSY41_PIN_LED_LOGICAL ((pin_id_logical)13)

#define MAX_TEENSY41_PIN_COUNT 55

typedef struct Hal {
    Hal *canary;
    pin_mode pin_modes[MAX_TEENSY41_PIN_COUNT];
} Hal;

#define PIN_LED_LOGICAL TEENSY41_PIN_LED_LOGICAL
