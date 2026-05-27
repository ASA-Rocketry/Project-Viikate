#include "mock.h"

void Hal_init_begin(Hal* hal) {
  ASSERT(hal->canary != hal, "Canary failed, this HAL was already initialized");

  hal->canary = 0;
  for (int i = 0; i < MAX_MOCK_HAL_PIN_COUNT; i++) {
    hal->pin_modes[i] = PIN_MODE_INACTIVE;
    hal->pin_values[i] = false;
  }
}

void Hal_init_enable_gpio_output_logical(Hal* hal, const pin_id_logical pin) {
  ASSERT(pin <= MAX_MOCK_HAL_PIN_COUNT, "Only %d pins are supported",
         MAX_MOCK_HAL_PIN_COUNT);
  ASSERT(hal->pin_modes[pin] == PIN_MODE_INACTIVE,
         "Pin mode for %d is already set to %s", MAX_MOCK_HAL_PIN_COUNT,
         pin_mode_to_str(PIN_MODE_OUTPUT));

  hal->pin_modes[pin] = PIN_MODE_OUTPUT;
}

void Hal_init_end(Hal* hal) {
  ASSERT(hal->canary != hal, "Canary failed, this HAL was already initialized");
  ASSERT(hal->canary == 0, "Canary failed, this HAL was not being initialized");

  hal->canary = hal;
}

void Hal_set_gpio_logical(Hal* hal, const pin_id_logical pin,
                          const bool state) {
  ASSERT(pin <= MAX_MOCK_HAL_PIN_COUNT, "Only %d pins are supported",
         MAX_MOCK_HAL_PIN_COUNT);
  ASSERT(hal->pin_modes[pin] == PIN_MODE_OUTPUT,
         "Pin %d must have output mode %s to be written, but has %s instead",
         pin, pin_mode_to_str(PIN_MODE_OUTPUT),
         pin_mode_to_str(hal->pin_modes[pin]));

  hal->pin_values[pin] = state;
}

bool Hal_get_gpio_logical(Hal* hal, pin_id_logical pin) {
  ASSERT(pin <= MAX_MOCK_HAL_PIN_COUNT, "Only %d pins are supported",
         MAX_MOCK_HAL_PIN_COUNT);

  return hal->pin_values[pin];
}

void Hal_blocking_sleep_ms(Hal* hal, int ms) {}
