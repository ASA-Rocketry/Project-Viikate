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
StateMachine state_machine(data_logger);
Sensors sensors(data_logger);
ControlHardware control_hardware;
Control control;

// Simulation state variables for TEST_STATE_MACHINE_MODE
bool simulation_started = false;
bool coastManeuverStarted = false;
unsigned long coastStartTime = 0;

// Last computed control error used for status reporting and LED state.
float error;

// PID test pattern: alternate between two orientation setpoints every 10 seconds.
unsigned long lastSwitchTime = 0;
const unsigned long interval = 5000;

// Setpoints for the orientation test. The system toggles between these values.
std::vector<float> setpoints = {0.0f, 90.0f};
int setpoint_index = 0;

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

void setup() {
#ifdef PRODUCTION_FLIGHT_MODE
    // Initialize the two serial ports used for debugging and telemetry.
    Serial.begin(9600);
    Serial8.begin(9600);
    delay(1000);  // Give serial time to initialize
    Serial.println("\n\n=== PRODUCTION_FLIGHT_MODE STARTING ===");    

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
    
    Serial.println("=== PRODUCTION FLIGHT MODE ===\n");

    pinMode(constants::kLEDPin, OUTPUT);
    pinMode(constants::kRbfPin, INPUT_PULLUP);  // Hardware interlock pin

#elif TEST_PID_AND_CALIBRATION_MODE
    // Initialize the two serial ports used for debugging and telemetry.
    Serial.begin(9600);
    Serial8.begin(9600);
    delay(1000);  // Give serial time to initialize
    Serial.println("\n\n=== TEST_PID_AND_CALIBRATION_MODE STARTING ===");

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

    Serial.println("\n\n=== TEST_PID_AND_CALIBRATION_MODE ===");

    pinMode(constants::kLEDPin, OUTPUT);

#elif TEST_STATE_MACHINE_MODE
    // Initialize the primary debug serial port.
    Serial.begin(9600);
    delay(1000);  // Give serial time to initialize
    Serial.println("\n\n=== TEST STATE MACHINE MODE STARTING ===");
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
    readSimulatedData();

    Serial.println("\n\n=== TEST STATE MACHINE MODE ===");

    pinMode(constants::kLEDPin, OUTPUT);
    pinMode(constants::kRbfPin, INPUT_PULLDOWN);  // Hardware interlock pin

#else
    //Initializing serial printing for debugging
    Serial.begin(9600);
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
#ifdef PRODUCTION_FLIGHT_MODE
  return;
#elif TEST_PID_AND_CALIBRATION_MODE
    // Acquire the latest sensor and attitude measurements.
    FlightData data = sensors.ReadFlightData();
    data_logger.LogFlightData(data);

    unsigned long currentTime = millis();

    // Advance the test setpoint every fixed interval to verify controller response.
    if (currentTime - lastSwitchTime >= interval) {
        lastSwitchTime = currentTime;
        setpoint_index = (setpoint_index + 1) % setpoints.size();
    }
    
    // Run the PID controller against the current orientation setpoint.
    control.PID(
        setpoints[setpoint_index],
        data.oriZ
    );

    // Print the current sensor and control state for local debugging.
    Serial.print("Altitude: ");
    Serial.println(data.altitude);
    Serial.print("Vertical Velocity: ");
    Serial.println(data.verticalVelocity);
    Serial.print("AccelZ: ");
    Serial.println(data.accZ);
    Serial.print("RotZ: ");
    Serial.println(data.rotZ);
    Serial.print("AccelMagnitude: ");
    Serial.println(data.accelMagnitude);
    Serial.print("RBF Removed: ");
    Serial.println(data.rbfRemoved);
    Serial.print("Acc (x, y, z): ");
    Serial.print(data.accX);
    Serial.print(", ");
    Serial.print(data.accY);
    Serial.print(", ");
    Serial.println(data.accZ);
    Serial.print("Gyro Angular Rate (x, y, z): ");
    Serial.print(data.rotX);
    Serial.print(", ");
    Serial.print(data.rotY);
    Serial.print(", ");
    Serial.println(data.rotZ);
    Serial.print("Orientation (x, y, z): ");
    Serial.print(data.oriX);
    Serial.print(", ");
    Serial.print(data.oriY);
    Serial.print(", ");
    Serial.println(data.oriZ);
    Serial.print("Mag: ");
    Serial.print(data.magX);
    Serial.print(", ");
    Serial.print(data.magY);
    Serial.print(", ");
    Serial.println(data.magZ);
    Serial.print("Heading: ");
    Serial.println(data.heading);
    Serial.println("--------------------");

    // Display the current control error and active setpoint.
    Serial.println("Control error:");
    error = control.get_error();
    Serial.println(error);
    Serial.print("Current setpoint: ");
    Serial.println(setpoints[setpoint_index]);

    // Use the onboard LED to indicate whether the controller is within tolerance.
    if (abs(error) < 5.0f) {
        digitalWrite(constants::kLEDPin, HIGH);
    } else {
        digitalWrite(constants::kLEDPin, LOW);
        delay(50);
    }

    // Send the compact telemetry packet to the secondary serial port.
    sendToSerial(Serial8, data, control);
#elif TEST_STATE_MACHINE_MODE
    /* 
    ### kCalibration ### 
    In this state the setup takes care of the initialization of the subsystems and loading of the simulated data. 
    */
    // Read real hardware data
    FlightData live_data = sensors.ReadFlightData();
    FlightData state_input_data = live_data;

    /* 
    ### kIdle ### 
    The setup and calibration is done, but the launch interlock is still engaged. 
    The state machine should not transition out of this state until the RBF pin is pulled.
    */ 

    // Start simulation AFTER entering launchpad state
    if (state_machine.GetState() == State::kLaunchpad &&
        !simulation_started) {

        simulation_started = true;

        Serial.println("Simulation started");
    }

    /* 
    ### kLaunchpad ### 
    The launch interlock has been removed, but the rocket is still on the launchpad.
    The state machine should transition to liftoff once the vertical acceleration exceeds the desired threshold.
    */ 
    
    /*
    ### kLiftoff ###
    The rocket has left the launchpad and is accelerating upwards.
    */

    if (simulation_started) {

        FlightData simulated_data = getSimulatedFlightData();

        // Hardware-in-the-loop inputs
        simulated_data.oriZ = live_data.oriZ;
        simulated_data.rbfRemoved = live_data.rbfRemoved;

        state_input_data = simulated_data;
    }

    // Update state machine
    State current_state = state_machine.Update(state_input_data);

    unsigned long currentTime = millis();

    // Control logic
    switch (current_state) {
        case State::kLiftoff:

        // Force neutral during liftoff
        control.PID(
            0.0f,
            state_input_data.oriZ
        );

        // Reset coast logic so it starts fresh later
        coastManeuverStarted = false;

        break;
        /* 
        ### kCoast ### 
        At the beginning of coast phase, perform the roll maneuver to 90 deg and back.
        */
        case State::kCoast:

            // Start maneuver once when entering coast
            if (!coastManeuverStarted) {
                coastManeuverStarted = true;
                coastStartTime = currentTime;
            }

            float coastSetpoint;

            // First 2 seconds: roll to 90°
            if (currentTime - coastStartTime < 2000) {
                coastSetpoint = 90.0f;
            }
            // After 2 seconds: return to neutral
            else {
                coastSetpoint = 0.0f;
            }

            control.PID(
                coastSetpoint,
                state_input_data.oriZ
            );

            break;

        /* 
        ### kApogee ### 
        The PID control can be turned of before recovery device deployment.
        */

        case State::kApogee:

            control.SetCanardAngle(0.0f);
            break;

        /* 
        ### kRDD ### 
        In the current implementation, the RDD is passive.
        */

        case State::kRDD:

            control.SetCanardAngle(0.0f);
            break;

            /*
            ### kGround ###
            The vehicle has hit the ground, and data recording can be stoped
            after a short delay.
            */

        case State::kGround:

            control.SetCanardAngle(0.0f);
            break;

        default:
            break;
    }

    // Debug output
    Serial.print("Current state: ");
    Serial.println(StateToString(current_state));

    Serial.print("Time (s): ");
    Serial.println(state_input_data.timeMs / 1000.0f);

    Serial.print("Altitude (m): ");
    Serial.println(state_input_data.altitude);

    Serial.print("Vertical velocity (m/s): ");
    Serial.println(state_input_data.verticalVelocity);

    Serial.print("Vertical acceleration (m/s²): ");
    Serial.println(state_input_data.accZ);

    Serial.print("Total acceleration (m/s²): ");
    Serial.println(state_input_data.accelMagnitude);

    Serial.print("RBF Removed: ");
    Serial.println(state_input_data.rbfRemoved);

    Serial.print("Orientation Z (live): ");
    Serial.println(state_input_data.oriZ);

    Serial.print("Current roll setpoint: ");
    Serial.println(setpoints[setpoint_index]);

    Serial.println("--------------------");

    delay(500);

#else
    Serial.println("=== UNKNOWN MODE ===\n");
#endif
}
