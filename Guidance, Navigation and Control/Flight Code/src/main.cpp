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
    FlightData simulated_data = getSimulatedFlightData();
    FlightData live_data = sensors.ReadFlightData();

    //Utilizing live Z angle for hardware in the loop simulation to get realistic control outputs and state transitions.
    simulated_data.oriZ = live_data.oriZ;

    state_machine.Update(simulated_data);
    // Advance the test setpoint every fixed interval to verify controller response.
    unsigned long currentTime = millis();
    if (currentTime - lastSwitchTime >= interval) {
        lastSwitchTime = currentTime;
        setpoint_index = (setpoint_index + 1) % setpoints.size();
    }

    // Run the PID controller against the current orientation setpoint.
    control.PID(
        setpoints[setpoint_index],
        simulated_data.oriZ
    );

    Serial.print("Current state: ");
    Serial.println(StateToString(state_machine.GetState()));
    Serial.print("Time (s): ");
    Serial.println(simulated_data.timeMs / 1000.0f);
    Serial.print("Altitude (m): ");
    Serial.println(simulated_data.altitude);
    Serial.print("Vertical velocity (m/s): ");
    Serial.println(simulated_data.verticalVelocity);
    Serial.print("Vertical acceleration (m/s²): ");
    Serial.println(simulated_data.accZ);
    Serial.print("Total acceleration (m/s²): ");
    Serial.println(simulated_data.accelMagnitude);
    Serial.print("RBF Removed: ");
    Serial.println(simulated_data.rbfRemoved);
    Serial.print("Orientation Z (live): ");
    Serial.println(simulated_data.oriZ);
    Serial.print("Gyro Z rate (live): ");
    Serial.println(simulated_data.rotZ);
    Serial.print("Current roll setpoint: ");
    Serial.println(setpoints[setpoint_index]);
    Serial.println("--------------------");
    delay(10);

#else
    Serial.println("=== UNKNOWN MODE ===\n");
#endif
}
