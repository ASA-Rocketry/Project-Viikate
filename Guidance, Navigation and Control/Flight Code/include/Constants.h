#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <Arduino.h>

// All constants are defined here.
namespace constants {

// Physics
constexpr float kGravity = 9.81f;

// Pins
constexpr int kRbfPin = 2;
constexpr int kCsPin = 10;
constexpr int kServo1Pin = 0;
constexpr int kServo2Pin = 0;
constexpr int kServo3Pin = 0;
constexpr int kServo4Pin = 0;

// PID parameters
constexpr float kProportional = 0.0f;
constexpr float kIntegrator = 0.0f;
constexpr float kDerivative = 0.0f;

}  // namespace constants

/**
 * @struct FlightData
 * @brief Telemetry snapshot.
 *
 * This structure acts as a data container populated by the main loop using
 * filtered sensor inputs. It ensures that the StateMachine, Controller, and
 * DataLogger operate on synchronized data points.
 */
struct FlightData {
  /** @brief [ms] System time since microcontroller boot. */
  unsigned long time_ms;

  /** @brief [m] Current altitude relative to ground level (barometric). */
  float altitude;

  /** @brief [m/s] Vertical velocity component (positive values indicate ascent). */
  float vertical_velocity;

  /** @brief [m/s^2] Linear acceleration along the rocket's Z-axis (gravity compensated). */
  float accel_z;

  /** @brief [rad/s] Angular velocity around the rocket's longitudinal (Z) axis. */
  float rot_z;

  /** @brief [m/s^2] Total magnitude of the acceleration vector (G-force). */
  float accel_magnitude;

  /** @brief Hardware interlock state: true if the "Remove Before Flight" pin is pulled. */
  bool rbf_removed;
};

/**
 * @enum LogType
 * @brief Severity levels for system events and error reporting.
 *
 * Used by the DataLogger to categorize messages. Numerical values increase
 * with the severity of the event.
 */
enum class LogType : uint8_t {
  kInfo = 0,      /**< General system status or phase transition info (Level 0). */
  kWarning = 1,   /**< Non-critical anomaly; system can still proceed (Level 1). */
  kError = 2,     /**< Recoverable error; a specific function might be degraded (Level 2). */
  kCritical = 3   /**< System-critical failure; flight safety or data integrity is at risk (Level 3). */
};

#endif  // CONSTANTS_H_