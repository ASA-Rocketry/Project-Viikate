#ifndef CONSTANTS
#define CONSTANTS

#include <Arduino.h>
// All constants are defined here 
namespace Constants {
    // Physics
    static constexpr float GRAVITY = 9.81f;

    // Pins
    static constexpr int RBF_PIN = 2;
    static constexpr int CS_PIN = 10;
    static constexpr int SERVO1_PIN = 0;
    static constexpr int SERVO2_PIN = 0;
    static constexpr int SERVO3_PIN = 0;
    static constexpr int SERVO4_PIN = 0;

    // PID parameters
    static constexpr float PROPORTIONAL = 0;
    static constexpr float INTEGRATOR = 0;
    static constexpr float DERIVATIVE = 0;
}

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
    
    /** @brief [m/s^2] Linear acceleration along the rocket's Z-axis (gravity compensated). */
    float accelZ;            
    
    /** @brief [rad/s] Angular velocity around the rocket's longitudinal (Z) axis. */
    float rotatZ;            
    
    /** @brief [m/s^2] Total magnitude of the acceleration vector (G-force). */
    float accelMagnitude;    
    
    /** @brief Hardware interlock state: True if the "Remove Before Flight" pin is pulled. */
    bool rbfRemoved;         
};

/**
 * @enum LogType
 * @brief Severity levels for system events and error reporting.
 * * Used by the DataLogger to categorize messages. Numerical values increase 
 * with the severity of the event.
 */
enum class LogType : uint8_t {
    /** @brief General system status or phase transition info (Level 0). */
    INFO = 0,     
    
    /** @brief Non-critical anomaly; system can still proceed (Level 1). */
    WARNING = 1,  
    
    /** @brief Recoverable error; a specific function might be degraded (Level 2). */
    ERROR = 2,    
    
    /** @brief System-critical failure; flight safety or data integrity is at risk (Level 3). */
    CRITICAL = 3  
};

#endif