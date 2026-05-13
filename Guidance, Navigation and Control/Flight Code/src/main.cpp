/*
 * src/main.cpp
 *
 * This is the Arduino application entry point for the flight control system.
 * It creates the core objects for logging, flight state management, sensor
 * acquisition, control actuation, and executes a simple setpoint test loop.
 *
 * The main loop reads live flight data, records telemetry, performs PID control
 * on the current orientation setpoint, and sends telemetry to an auxiliary serial
 * port for monitoring.
 */

#include "Arduino.h"
#include "constants.h"
#include "control.h"
#include "control_hardware.h"
#include "data_logger.h"
#include "flight_data.h"
#include "sensors.h"
#include "state_machine.h"

#ifdef PRODUCTION_FLIGHT_MODE
    #define MODE_NAME "PRODUCTION_FLIGHT_MODE"
#elif defined(TEST_STATE_MACHINE_MODE)
    #define MODE_NAME "TEST_STATE_MACHINE_MODE"
    #define ENABLE_TEST_PRINTS
#elif defined(TEST_PID_AND_CALIBRATION_MODE)
    #define MODE_NAME "TEST_PID_AND_CALIBRATION_MODE"
    #define ENABLE_TEST_PRINTS
#endif

#ifdef ENABLE_TEST_PRINTS
    #define DEBUG_BEGIN(baud) Serial.begin(baud)
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else // Do nothing if not testing
    #define DEBUG_BEGIN(baud) 
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif


// Global subsystem instances used throughout the flight control loop.
DataLogger data_logger;
Sensors sensors(data_logger);
Control control;

// Main variables
float error; // Control error used for status reporting and LED state.
constexpr float setpoints[] = {0.0f, 90.0f}; // Predefined setpoints for PID testing (0° and 90° roll).

#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)
    StateMachine state_machine(data_logger, control); // Only needed in production flight mode and state machine testing mode
    bool coastManeuverStarted = false;
    unsigned long coastStartTime = 0;
#endif

// Timing variables for setpoint switching in TEST_PID_AND_CALIBRATION_MODE.
#ifdef TEST_PID_AND_CALIBRATION_MODE
    static unsigned long lastSwitchTime = 0;
    static constexpr unsigned long interval = 5000;
#endif

#if defined(TEST_STATE_MACHINE_MODE) || defined(TEST_PID_AND_CALIBRATION_MODE)
    // Binary-packed telemetry packet structure sent over Serial8.
    struct TelemetryData {
        float altitude;
        float verticalVelocity;
        float accZ;
        float rotZ;
        float error;
        uint32_t timestamp;
    } __attribute__((packed));

    /**
     * @brief Send a packed telemetry packet to a serial port.
     *
     * This function converts the selected FlightData values and the current
     * control error into a compact binary packet for external logging or analysis.
     * It uses Serial8 as the dedicated telemetry transport path.
     */
    void sendToSerial(Print &serial, FlightData &data, Control &control) {
        TelemetryData telem;
        telem.altitude = data.altitude;
        telem.verticalVelocity = data.verticalVelocity;
        telem.accZ = data.accZ;
        telem.rotZ = data.oriZ;
        telem.error = control.get_error();
        telem.timestamp = micros();

        serial.write((uint8_t *)&telem, sizeof(telem));
    }
#endif

void setup() {
#if defined(TEST_STATE_MACHINE_MODE) || defined(TEST_PID_AND_CALIBRATION_MODE)
    // Initialize the two serial ports used for debugging and telemetry.
    Serial.begin(9600);
    Serial8.begin(9600);
    delay(1000);  // Give serial time to initialize
#endif

Serial.println("\n\n=== " MODE_NAME " STARTING ===");

    // Initialize core subsystems in a safe order.
    // Data logger first so that any failures during sensors/control init can be recorded.
    if (!data_logger.Initialize()) {
        Serial.println("Failed to initialize DataLogger!");
        return;
    }
    if (!sensors.Initialize()) {
        Serial.println("Failed to initialize Sensors!");
        return;
    }
    if (!control.Initialize()) {
        Serial.println("Failed to initialize Control!");
        return;
    }
    data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");
    pinMode(constants::kLEDPin, OUTPUT); 
    pinMode(constants::kRbfPin, INPUT_PULLUP);  // Hardware interlock pin

#ifdef TEST_STATE_MACHINE_MODE
    readSimulatedData(); // Function in flightData
#endif 

Serial.println("\n\n=== " MODE_NAME " INITIALIZED ===");
}
/**
 * @brief Main Arduino loop that executes the flight control cycle.
 *
 * Each iteration reads the latest flight sensor values, stores them in the data
 * logger, updates the PID controller on the current test setpoint, and sends
 * out telemetry over both serial outputs.
 */
void loop() {
    FlightData inputData = sensors.ReadFlightData(); // Actual live hardware data is always read.

#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)
    #ifdef TEST_STATE_MACHINE_MODE
    static bool simulation_started = false; // Static so that the value is not reset on each loop cycle 
    
    // Start simulation once launchpad state is reached.
    if ((state_machine.GetState() == State::kLaunchpad) && !simulation_started) {
        //delay(30000); // Short delay to ensure stable transition before starting simulation
        simulation_started = true;
        Serial.println("Simulation started");
    }

    if (simulation_started) {
        FlightData simulatedData = getSimulatedFlightData();

        // Hardware-in-the-loop inputs remain live.
        simulatedData.oriZ = inputData.oriZ; // Live orientation data is saved
        simulatedData.rbfRemoved = inputData.rbfRemoved; // Live rbf data is saved
        inputData = simulatedData; // Simulated data with actual oriZ and rbf data
    }
    #endif

    data_logger.LogFlightData(inputData);
    State currentState = state_machine.Update(inputData);
    const unsigned long currentTime = millis();
    float coastSetpoint = 0.0f; 
    // Execute control logic based on the current state.
    // CHANGE THIS LATER TO BE STATIC
    switch (currentState) {
        case State::kLiftoff: {
            control.SetCanardAngle(0.0f); // Maintain a neutral servo angle to ensure stable ascent.
            coastManeuverStarted = false;
            break;
        }
        case State::kCoast: {
            if (!coastManeuverStarted) {
                coastManeuverStarted = true;
                coastStartTime = currentTime;
            }
            coastSetpoint = ((currentTime - coastStartTime) < 4000) ? 180.0f : 0.0f; // Perform a roll maneuver to 180 degrees and back to neutral.
            control.PID(coastSetpoint, inputData.oriZ);
            break;
        }
        case State::kApogee:{
            control.SetCanardAngle(0.0f); // Neutral canard angle
            break;
        }
        case State::kRDD:{
            control.SetCanardAngle(0.0f);  // Neutral canard angle
            break;
        }
        case State::kGround:{
            control.SetCanardAngle(0.0f);  // Neutral canard angle
            delay(2000); // Wait for 2 seconds to ensure all flight data is logged before closing files
            data_logger.Close(); // Close SD card
            break;
        } 
        default: break;
    }

    // Debug output
    Serial.print("Current state: ");
    Serial.println(StateToString(currentState));
    Serial.print("Time (s): ");
    Serial.println(currentTime / 1000.0f);
    Serial.print("Altitude (m): ");
    Serial.println(inputData.altitude);
    Serial.print("Vertical velocity (m/s): ");
    Serial.println(inputData.verticalVelocity);
    Serial.print("Vertical acceleration (m/s²): ");
    Serial.println(inputData.accZ);
    Serial.print("Total acceleration (m/s²): ");
    Serial.println(inputData.accelMagnitude);
    Serial.print("RBF Removed: ");
    Serial.println(inputData.rbfRemoved);
    Serial.print("Orientation Z (live): ");
    Serial.println(inputData.oriZ);
    Serial.print("Current roll setpoint: ");
    Serial.println(coastSetpoint);
    Serial.println("--------------------");
#endif

#if defined(TEST_PID_AND_CALIBRATION_MODE)
    data_logger.LogFlightData(inputData);
    const unsigned long currentTime = millis();
    constexpr size_t numSetpoints = std::size(setpoints); 
    static size_t setpointIndex = 0; // Static so that the value is not reset on each loop cycle

    if ((currentTime - lastSwitchTime) >= interval) {
        lastSwitchTime = currentTime;
        setpointIndex = (setpointIndex + 1) % numSetpoints; // Counter is reset if the last index is reached
    }

    const float currentSetpoint = setpoints[setpointIndex];
    control.PID(currentSetpoint, inputData.oriZ);

    // Print the current sensor and control state for local debugging.
    Serial.print("Altitude: ");
    Serial.println(inputData.altitude);
    Serial.print("Vertical Velocity: ");
    Serial.println(inputData.verticalVelocity);
    Serial.print("AccelZ: ");
    Serial.println(inputData.accZ);
    Serial.print("RotZ: ");
    Serial.println(inputData.rotZ);
    Serial.print("AccelMagnitude: ");
    Serial.println(inputData.accelMagnitude);
    Serial.print("RBF Removed: ");
    Serial.println(inputData.rbfRemoved);
    Serial.print("Acc (x, y, z): ");
    Serial.print(inputData.accX);
    Serial.print(", ");
    Serial.print(inputData.accY);
    Serial.print(", ");
    Serial.println(inputData.accZ);
    Serial.print("Gyro Angular Rate (x, y, z): ");
    Serial.print(inputData.rotX);
    Serial.print(", ");
    Serial.print(inputData.rotY);
    Serial.print(", ");
    Serial.println(inputData.rotZ);
    Serial.print("Orientation (x, y, z): ");
    Serial.print(inputData.oriX);
    Serial.print(", ");
    Serial.print(inputData.oriY);
    Serial.print(", ");
    Serial.println(inputData.oriZ);
    Serial.print("Mag: ");
    Serial.print(inputData.magX);
    Serial.print(", ");
    Serial.print(inputData.magY);
    Serial.print(", ");
    Serial.println(inputData.magZ);
    Serial.print("Heading: ");
    Serial.println(inputData.heading);
    Serial.println("--------------------");

    // Display the current control error and active setpoint.
    Serial.println("Control error:");
    float error = control.get_error();
    Serial.println(error);
    Serial.print("Current setpoint: ");
    Serial.println(setpoints[setpointIndex]);

    // Use the onboard LED to indicate whether the controller is within tolerance.
    if (abs(error) < 5.0f) { 
        digitalWrite(constants::kLEDPin, HIGH);
    } 
    else {
        digitalWrite(constants::kLEDPin, LOW);
        delay(50);
    }
    sendToSerial(Serial8, inputData, control);  // Send the compact telemetry packet to the secondary serial port.
#endif
}
