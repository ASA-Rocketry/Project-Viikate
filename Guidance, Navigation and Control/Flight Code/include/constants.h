
#ifndef FLIGHT_CODE_INCLUDE_CONSTANTS_H_
#define FLIGHT_CODE_INCLUDE_CONSTANTS_H_

#include <Arduino.h>

#include <cstddef>

// All constants are defined here
namespace constants {
// Physics
static constexpr float kGravity = 9.81f;

// Pins
static constexpr int kServo1Pin = 2;
static constexpr int kServo2Pin = 3;
static constexpr int kServo3Pin = 4;
static constexpr int kServo4Pin = 5;
static constexpr int kUARTTxPin = 35;
static constexpr int kUARTRxPin = 34;

static constexpr int kLEDPin = 24;

static constexpr int kRbfPin = 2;
static constexpr int kImuCsPin = 10;
static constexpr int kMISOPin = 12;
static constexpr int kMOSIPin = 11;
static constexpr int kSCKPin = 13;

// Z rotation PID parameters
static float kZProportional = 0.1745f; // as per Taneli's simulations
static float kZIntegrator = 0.1f; // as per wind tunnel tests
static float kZDerivative = 0.06f;  // as per Taneli's simulations

// Taneli the magician's Z rotation PID parameters:
//static float kZProportional = 0.1745f;
//static float kZIntegrator = 0.0f;
//static float kZDerivative = 0.06f;

// XY rotation PID parameters
static float kXYProportional = 0.2f;
static float kXYIntegrator = 0.1f;
static float kXYDerivative = 0.06f;

// PID output limits
static constexpr float kMaxControlAngle =
    30.0f;  // Max canard deflection in degrees
static constexpr float kMinControlAngle =
    -30.0f;  // Min canard deflection in degrees
static constexpr float kNeutralControlAngle =
    0.0f;  // Neutral canard deflection in degrees

// PID effective input range
static constexpr float kPidMinAngle = -30.0f;
static constexpr float kPidMaxAngle = 30.0f;

// Servo mapping limits (for ControlHardware)
static constexpr float kServoMinAngle =
    60.0f;  // Servo minimum command angle in degrees
static constexpr float kServoMaxAngle =
    120.0f;  // Servo maximum command angle in degrees
static constexpr float kServoNeutralAngle =
    90.0f;  // Servo neutral command angle

// servo trimming parameters
static float kServoTrim1 = -10.0f;
static float kServoTrim2 = -10.0f;
static float kServoTrim3 = -10.0f;
static float kServoTrim4 = -10.0f;

static const unsigned long CALIBRATION_DURATION_MS = 5000;
}  // namespace constants

/**
 * @struct FlightData
 * @brief Telemetry snapshot.
 * * This structure acts as a data container populated by the main loop using 
 * filtered sensor inputs. It ensures that the StateMachine, Controller and DataLogger 
 * operate on synchronized data points.
 */
struct FlightData {
    /** @brief [ms] System time since microcontroller boot. */
    unsigned long timeMs;

    /** @brief [m] Current altitude relative to ground level (barometric). */
    float altitude;

    /** @brief [m/s] Vertical velocity component (positive values indicate ascent). */
    float verticalVelocity;

    /** @brief [m/s^2] Linear acceleration along the rocket's X-axis. */
    float accX;

    /** @brief [m/s^2] Linear acceleration along the rocket's Y-axis. */
    float accY;

    /** @brief [m/s^2] Linear acceleration along the rocket's Z-axis (gravity compensated). */
    float accZ;

    /** @brief [rad/s] Angular velocity around the rocket's X-axis. */
    float rotX;

    /** @brief [rad/s] Angular velocity around the rocket's Y-axis. */
    float rotY;

    /** @brief [rad/s] Angular velocity around the rocket's longitudinal (Z) axis. */
    float rotZ;

    /** @brief [rad] Orientation around the rocket's X-axis. */
    float oriX;

    /** @brief [rad] Orientation around the rocket's Y-axis. */
    float oriY;

    /** @brief [rad] Orientation around the rocket's longitudinal (Z) axis. */
    float oriZ;

    /** @brief [\muT] Strength of magnetic field on the rocket's X-Axis */
    double magX;

    /** @brief [\muT] Strength of magnetic field on the rocket's Y-Axis */
    double magY;

    /** @brief [\muT] Strength of magnetic field on the rocket's Z-Axis */
    double magZ;

    /** @brief [rad] Magnetic heading relative to true north. */
    double heading;

    /** @brief [m/s^2] Total magnitude of the acceleration vector (G-force). */
    float accelMagnitude;

    /** @brief Hardware interlock state: True if the "Remove Before Flight" pin is pulled. */
    bool rbfRemoved;

    void SerializeJson(char *outputBuffer, std::size_t outputBufferSize);
};
/**
 * @enum LogType
 * @brief Severity levels for system events and error reporting.
 * * Used by the DataLogger to categorize messages. Numerical values increase 
 * with the severity of the event.
 */
enum class LogType : uint8_t {
    /** @brief General system status or phase transition info (Level 0). */
    kInfo = 0,

    /** @brief Non-critical anomaly; system can still proceed (Level 1). */
    kWarning = 1,

    /** @brief Recoverable error; a specific function might be degraded (Level 2). */
    kError = 2,

    /** @brief System-critical failure; flight safety or data integrity is at risk (Level 3). */
    kCritical = 3
};

#endif  // FLIGHT_CODE_INCLUDE_CONSTANTS_H_
