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

#include <vector>

// Global subsystem instances used throughout the flight control loop.
DataLogger data_logger;
Sensors sensors(data_logger);
ControlHardware control_hardware;
Control control;

// State machine is only needed in production flight mode and state machine testing mode.
#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)
    StateMachine state_machine(data_logger);
#endif

// Simulation state variables for TEST_STATE_MACHINE_MODE
#ifdef TEST_STATE_MACHINE_MODE
    bool simulation_started = false;
#endif

#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)
    bool coastManeuverStarted = false;
    unsigned long coastStartTime = 0;
#endif

// Last computed control error used for status reporting and LED state.
float error;

// Timing variables for setpoint switching in TEST_PID_AND_CALIBRATION_MODE.
#ifdef TEST_PID_AND_CALIBRATION_MODE
    static unsigned long lastSwitchTime = 0;
    static constexpr unsigned long interval = 5000;
#endif

// Predefined setpoints for PID testing (0° and 90° roll).
// Using constexpr array for fixed setpoints known at compile time.
// This is more efficient for embedded systems than dynamic structures like std::vector.
constexpr float setpoints[] = {0.0f, 90.0f};
constexpr size_t numSetpoints = sizeof(setpoints) / sizeof(setpoints[0]);

static constexpr size_t setpointIndexInitial = 0;
static size_t setpointIndex = setpointIndexInitial;

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
    void sendToSerial(Print &serial, FlightData data, Control control) {
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

#ifdef PRODUCTION_FLIGHT_MODE
    Serial.println("\n\n=== PRODUCTION_FLIGHT_MODE STARTING ===");    
#elif TEST_STATE_MACHINE_MODE
    Serial.println("\n\n=== TEST_STATE_MACHINE_MODE STARTING ===");
#elif TEST_PID_AND_CALIBRATION_MODE
    Serial.println("\n\n=== TEST_PID_AND_CALIBRATION_MODE STARTING ===");
#endif

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
    readSimulatedData();
#endif

#ifdef PRODUCTION_FLIGHT_MODE    
    Serial.println("\n\n=== PRODUCTION_FLIGHT_MODE INITIALIZED ===");    
#elif TEST_STATE_MACHINE_MODE
    Serial.println("\n\n=== TEST_STATE_MACHINE_MODE INITIALIZED ===");
#elif TEST_PID_AND_CALIBRATION_MODE
    Serial.println("\n\n=== TEST_PID_AND_CALIBRATION_MODE INITIALIZED ===");
#endif
}

/**
 * @brief Main Arduino loop that executes the flight control cycle.
 *
 * Each iteration reads the latest flight sensor values, stores them in the data
 * logger, updates the PID controller on the current test setpoint, and sends
 * out telemetry over both serial outputs.
 */
void loop() {

// Initializing data input.
#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE) || defined(TEST_PID_AND_CALIBRATION_MODE)

    // Read live hardware data.
    FlightData liveData = sensors.ReadFlightData();

    // Default processing input.
    FlightData inputData = liveData;

#endif

// State machine and control logic for production flight and state machine testing modes.
#if defined(PRODUCTION_FLIGHT_MODE) || defined(TEST_STATE_MACHINE_MODE)

    // Update the state machine with the latest flight data and get the current state.
    #ifdef TEST_STATE_MACHINE_MODE

    // Start simulation once launchpad state is reached.
    if ((state_machine.GetState() == State::kLaunchpad) && !simulation_started) {
        simulation_started = true;
        Serial.println("Simulation started");
    }

    // Replace selected flight values with simulated data.
    if (simulation_started) {
        FlightData simulatedData = getSimulatedFlightData();

        // Hardware-in-the-loop inputs remain live.
        simulatedData.oriZ = liveData.oriZ;
        simulatedData.rbfRemoved = liveData.rbfRemoved;
        inputData = simulatedData;
    }

    #endif

    // Log the current flight data to the data logger for persistent storage.
    data_logger.LogFlightData(inputData);

    // Update the state machine and get the current flight state.
    State currentState = state_machine.Update(inputData);
    
    // Variable to store current time for control timing and state-based logic.
    const unsigned long currentTime = millis();

    // Execute control logic based on the current state.
    // CHANGE THIS LATER TO BE STATIC
    switch (currentState) {
        // ### kLiftoff ###
        // During liftoff, we want to maintain a neutral servo angle to ensure stable ascent.
        case State::kLiftoff: {
            control.SetCanardAngle(0.0f);
            coastManeuverStarted = false;
            break;
        }

        /// ### kCoast ###
        // In the coast phase, we perform a roll maneuver to 90 degrees and back to neutral.
        case State::kCoast: {
            float coastSetpoint = 0.0f;


            if (!coastManeuverStarted) {

                coastManeuverStarted = true;
                coastStartTime = currentTime;
            }

            if ((currentTime - coastStartTime) < 2000) {
                coastSetpoint = 90.0f;
            }
            else {
                coastSetpoint = 0.0f;
            }

            control.PID(
                coastSetpoint,
                inputData.oriZ
            );

            break;
        }

        /// ### kApogee ###
        // At apogee, we set the canard angle to neutral to prepare for descent and recovery device deployment.
        case State::kApogee:{
          control.SetCanardAngle(0.0f);
          break;
        }

        /// ### kRDD ###
        case State::kRDD:{
          control.SetCanardAngle(0.0f);
          break;
        }

        /// ### kGround ###
        case State::kGround:{
          control.SetCanardAngle(0.0f);
          break;
        } 

        default:
            break;
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
    Serial.println(setpoints[setpointIndex]);

    Serial.println("--------------------");
    //delay(500);
#endif

#if defined(TEST_PID_AND_CALIBRATION_MODE)

    // Log the current flight data to the data logger for persistent storage. 
    data_logger.LogFlightData(inputData);

    // Variable to store current time for control timing and state-based logic.
    const unsigned long currentTime = millis();

    if ((currentTime - lastSwitchTime) >= interval) {
        lastSwitchTime = currentTime;
        setpointIndex++;

        if (setpointIndex >= numSetpoints) {
            setpointIndex = 0;
        }
    }

    const float currentSetpoint = setpoints[setpointIndex];

    control.PID(
        currentSetpoint,
        inputData.oriZ
    );

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
    } else {
        digitalWrite(constants::kLEDPin, LOW);
        delay(50);
    }

    // Send the compact telemetry packet to the secondary serial port.
    sendToSerial(Serial8, inputData, control);
#endif
}
