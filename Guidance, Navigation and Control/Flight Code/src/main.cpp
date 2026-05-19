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
#include "debug_prints.h"

// Global subsystem instances used throughout the flight control loop.
DataLogger data_logger;
Sensors sensors(data_logger);
Control control;

// Main variables
float error; // Control error used for status reporting and LED state.
constexpr float setpoints[] = {0.0f, 90.0f}; // Predefined setpoints for PID testing (0° and 90° roll).

#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)
    StateMachine state_machine(data_logger, sensors, control); // Only needed in production flight mode and state machine testing mode
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

DEBUG_PRINTLN("\n\n=== " MODE_NAME " STARTING ===");

    // Initialize core subsystems in a safe order.
    // Data logger first so that any failures during sensors/control init can be recorded.
    if (!data_logger.Initialize()) {
        DEBUG_PRINTLN("Failed to initialize DataLogger!");
        return;
    }
    if (!sensors.Initialize()) {
        DEBUG_PRINTLN("Failed to initialize Sensors!");
        return;
    }
    if (!control.Initialize()) {
        DEBUG_PRINTLN("Failed to initialize Control!");
        return;
    }
    data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");
    pinMode(constants::kLEDPin, OUTPUT); 
    pinMode(constants::kRbfPin, INPUT_PULLUP);  // Hardware interlock pin

#ifdef TEST_STATE_MACHINE_MODE
    readSimulatedData(); // Function in flightData
#endif 

DEBUG_PRINTLN("\n\n=== " MODE_NAME " INITIALIZED ===");
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
        DEBUG_PRINTLN("Simulation started");
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

            const unsigned long elapsedTime = currentTime - coastStartTime;

            // Hold 0° for the first 2 seconds.
            // Command 180° for the next 4 seconds.
            // Return to 0° afterwards.
            if (elapsedTime < 2000) {
                coastSetpoint = 0.0f;
            }
            else if (elapsedTime < 4000) {
                coastSetpoint = 90.0f;
            }
            else {
                coastSetpoint = 0.0f;
            }
            
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
    DEBUG_PRINT("Current state: ");
    DEBUG_PRINTLN(StateToString(currentState));
    DEBUG_PRINT("Time (s): ");
    DEBUG_PRINTLN(currentTime / 1000.0f);
    DEBUG_PRINT("Altitude (m): ");
    DEBUG_PRINTLN(inputData.altitude);
    DEBUG_PRINT("Vertical velocity (m/s): ");
    DEBUG_PRINTLN(inputData.verticalVelocity);
    DEBUG_PRINT("Vertical acceleration (m/s²): ");
    DEBUG_PRINTLN(inputData.accZ);
    DEBUG_PRINT("Total acceleration (m/s²): ");
    DEBUG_PRINTLN(inputData.accelMagnitude);
    DEBUG_PRINT("RBF Removed: ");
    DEBUG_PRINTLN(inputData.rbfRemoved);
    DEBUG_PRINT("Orientation Z (live): ");
    DEBUG_PRINTLN(inputData.oriZ);
    DEBUG_PRINT("Current roll setpoint: ");
    DEBUG_PRINTLN(coastSetpoint);
    DEBUG_PRINTLN("--------------------");
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
    DEBUG_PRINT("Altitude: ");
    DEBUG_PRINTLN(inputData.altitude);
    DEBUG_PRINT("Vertical Velocity: ");
    DEBUG_PRINTLN(inputData.verticalVelocity);
    DEBUG_PRINT("AccelZ: ");
    DEBUG_PRINTLN(inputData.accZ);
    DEBUG_PRINT("RotZ: ");
    DEBUG_PRINTLN(inputData.rotZ);
    DEBUG_PRINT("AccelMagnitude: ");
    DEBUG_PRINTLN(inputData.accelMagnitude);
    DEBUG_PRINT("RBF Removed: ");
    DEBUG_PRINTLN(inputData.rbfRemoved);
    DEBUG_PRINT("Acc (x, y, z): ");
    DEBUG_PRINT(inputData.accX);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(inputData.accY);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(inputData.accZ);
    DEBUG_PRINT("Gyro Angular Rate (x, y, z): ");
    DEBUG_PRINT(inputData.rotX);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(inputData.rotY);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(inputData.rotZ);
    DEBUG_PRINT("Orientation (x, y, z): ");
    DEBUG_PRINT(inputData.oriX);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(inputData.oriY);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(inputData.oriZ);
    DEBUG_PRINT("Mag: ");
    DEBUG_PRINT(inputData.magX);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(inputData.magY);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(inputData.magZ);
    DEBUG_PRINT("Heading: ");
    DEBUG_PRINTLN(inputData.heading);
    DEBUG_PRINTLN("--------------------");

    // Display the current control error and active setpoint.
    DEBUG_PRINTLN("Control error:");
    float error = control.get_error();
    DEBUG_PRINTLN(error);
    DEBUG_PRINT("Current setpoint: ");
    DEBUG_PRINTLN(setpoints[setpointIndex]);

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
