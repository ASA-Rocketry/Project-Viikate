#ifndef FLIGHT_CODE_INCLUDE_CONTROL_HARDWARE_H_
#define FLIGHT_CODE_INCLUDE_CONTROL_HARDWARE_H_

#include <Arduino.h>
#include "Servo.h"
#include "constants.h"

/**
 * @class ControlHardware
 * @brief Hardware abstraction layer for canard servo control.
 *
 * Manages four servo motors for symmetric canard deflection control.
 * Provides initialization and angle command interfaces.
 */
class ControlHardware {
 public:
  ControlHardware();

  /** @brief Initializes all servo hardware. */
  bool Initialize();

  /**
   * @brief Sets the target angle for all canard servos.
   * @param angle_degrees Desired canard deflection in degrees.
   */
  void SetCanardAngle(float angle_degrees);

  void initialisationAnimation(); // Optional: Visual servo test pattern for pre-flight checks

 private:
  Servo servo1_;  /**< Servo for canard 1 */
  Servo servo2_;  /**< Servo for canard 2 */
  Servo servo3_;  /**< Servo for canard 3 */
  Servo servo4_;  /**< Servo for canard 4 */
};

#endif  // FLIGHT_CODE_INCLUDE_CONTROL_HARDWARE_H_