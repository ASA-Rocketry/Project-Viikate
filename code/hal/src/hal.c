#include "hal.h"

const char* pin_mode_to_str(pin_mode pin) {
  switch (pin) {
    case PIN_MODE_OUTPUT:
      return "PIN_MODE_OUTPUT";
    case PIN_MODE_INACTIVE:
      return "PIN_MODE_INACTIVE";
    case PIN_MODE_INPUT:
      return "PIN_MODE_INPUT";
    default:
      PANIC("Pin mode number %d does not exist", pin);
      break;
  }
}
