#include "hal.h"

Hal make_hal(void) {
  Hal hal;

  Hal_init_begin(&hal);
  Hal_init_enable_gpio_output_logical(&hal, PIN_LED_LOGICAL);
  Hal_init_end(&hal);

  return hal;
}

int main(void) {
  Hal hal = make_hal();

  bool on = false;
  for (;;) {
    Hal_blocking_sleep_ms(&hal, 1000);
    Hal_set_gpio_logical(&hal, PIN_LED_LOGICAL, on);
    on = !on;
  }

  UNREACHABLE;
}
