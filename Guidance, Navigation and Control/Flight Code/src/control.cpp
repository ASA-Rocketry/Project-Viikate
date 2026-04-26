#include "control.h"
#include "constants.h"
#include "control_hardware.h"  // Include for ControlHardware class

/**
 * @brief Constructs a Control object.
 * Initializes member variables to safe defaults.
 */
Control::Control() :
    previous_error_(0.0f),
    integral_error_(0.0f),
    previous_time_ms_(0),
    control_hardware_() {}

/**
 * @brief Initializes the control system.
 * Resets PID state variables and initializes hardware.
 * @return true if initialization successful, false otherwise.
 */
bool Control::Initialize() {
  // Reset PID state to ensure clean start
  previous_error_ = 0.0f;
  integral_error_ = 0.0f;
  previous_time_ms_ = millis();

  // Initialize servo hardware (attaches pins, calibrates to 90°)
  // NOTE: Assumes control_hardware_ member exists and has Initialize() method
  control_hardware_.Initialize();

  return true;
}

/**
 * @brief Executes the PID control loop.
 * Takes current angular velocity and computes control output.
 * @param set_angle Desired angle in degrees.
 * @param current_angle Current angle in degrees.
 */
void Control::PID(float set_angle_deg, float current_angle_deg) {
  // PID parameters from constants.h
  float Kp = constants::kProportional;
  float Ki = constants::kIntegrator;
  float Kd = constants::kDerivative;

  // Calculate angular error
  float angular_error = set_angle_deg - current_angle_deg;

  // Maximum and Minimum output values from constants.h.
  float max_output = constants::kMaxControlAngle;
  float min_output = constants::kMinControlAngle;

  // Calculate time delta for derivative and integral terms
  unsigned long current_time_ms = millis();
  float dt = (current_time_ms - previous_time_ms_) * 0.001f;  // Convert ms to seconds

  // Prevent division by zero and ensure minimum dt for numerical stability
  if (dt <= 0.0f) {
    dt = 0.001f;  // Minimum 1ms timestep
  }

  // Proportional term
  float p_term = Kp * angular_error;  

  // Integral term with anti-windup clamping.
  float candidate_integral = integral_error_ + angular_error * dt;
  float i_term = Ki * candidate_integral;

  // Derivative term
  float d_term = Kd * ((angular_error - previous_error_) / dt);

    //if current angle is small, D output = 0
  if (abs(angular_error - previous_error_) < 10.0f) {
    d_term = 0.00f;
  }

  if (previous_error_ > 0.0 && previous_error_ < 30.0f) {
    Kd = 0.01f;
  } else if (previous_error_ < 0 && previous_error_ > -30.0f) {
    Kd = 0.01f;
  }

  // Combine PID terms for raw output
  float output = p_term + i_term + d_term;

  Serial.print("PID Values: ");
  Serial.print("P: ");
  Serial.print(p_term);
  Serial.print(" I: ");
  Serial.print(i_term);
  Serial.print(" D: ");
  Serial.println(d_term);

  // Apply output saturation to prevent actuator damage (output range [-30, 30] degrees)
  float saturated_output = constrain(output, min_output, max_output);

  // Anti-windup: accumulate integral only when:
  // 1. Output is not saturated, OR
  // 2. Error is driving output back toward unsaturated region (recovery direction)
  bool should_accumulate_integral = (saturated_output == output) ||
                                    (output > max_output && angular_error < 0.0f) ||
                                    (output < min_output && angular_error > 0.0f);

  if (should_accumulate_integral) {
    integral_error_ = candidate_integral;
  }

  // Update state for next iteration.
  previous_error_ = angular_error;
  previous_time_ms_ = current_time_ms;

  // Command servos: maps [-30,30]° to [60,120]° servo range inside of the ControlHardware class
  control_hardware_.SetCanardAngle(saturated_output);
}