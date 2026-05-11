
#ifndef FLIGHT_CODE_INCLUDE_CONTROL_H_
#define FLIGHT_CODE_INCLUDE_CONTROL_H_

#include "control_hardware.h"

class Control {
 public:
  Control();
  bool Initialize();

  /**
   * @brief Executes the PID control loop.
   * @param set_angle Desired target angle in degrees.
   * @param current_angle Current measured angle in degrees.
   */
  void PID(float set_angle, float current_angle);

  float get_error() const { return previous_error_; } // Accessor for testing and debugging
  
  void SetCanardAngle(float angle);// Accessor for testing and debugging

    /**
     * @brief Checks if the control system is initialized.
     * @return true if initialized, false otherwise.
     */
    bool IsInitialized() const;

 private:
  bool initialized_ = false;
  // PID state variables
  float previous_error_;      // Previous angular error for derivative calculation
  float integral_error_;      // Accumulated error for integral term
  unsigned long previous_time_ms_;  // Timestamp of previous PID iteration
  float previous_set_angle_; // Previous set angle for setting the derivative error to 0 when angle changes

  // Hardware interface
  ControlHardware control_hardware_;  // Servo control abstraction
};

#endif  // FLIGHT_CODE_INCLUDE_CONTROL_H_