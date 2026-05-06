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
const unsigned long interval = 10000;

// Setpoints for the orientation test. The system toggles between these values.
std::vector<float> setpoints = {-90.0f, 90.0f};
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
#ifdef PRODUCTION_MODE
    // Initialize the two serial ports used for debugging and telemetry.
    Serial.begin(9600);
    Serial8.begin(9600);

    // Initialize core subsystems in a safe order.
    // Data logger first so that any failures during sensors/control init can be recorded.
    data_logger.Initialize();
    sensors.Initialize();
    control.Initialize();
    data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");

    pinMode(constants::kLEDPin, OUTPUT);
#elif TEST_MODE
    // Initialize the primary debug serial port.
    Serial.begin(9600);
    readSimulatedData();
    Serial.println("Test mode setup complete.");

#else
    return;
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
#ifdef PRODUCTION_MODE
    // Acquire the latest sensor and attitude measurements.
    FlightData data = sensors.ReadFlightData();
    data_logger.LogFlightData(data);

    unsigned long currentTime = millis();

    // Advance the test setpoint every fixed interval to verify controller response.
    if (currentTime - lastSwitchTime >= interval) {
        lastSwitchTime = currentTime;
        setpoint_index = (setpoint_index + 1) % setpoints.size();
    }
     //char serializedFlightData[512] = "";
     //data.SerializeJson(serializedFlightData, sizeof(serializedFlightData));
     //Serial8.println(serializedFlightData);

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
#elif TEST_MODE
    FlightData data = getSimulatedFlightData();
    state_machine.Update(data);

    Serial.print("Current state: ");
    Serial.println(StateToString(state_machine.GetState()));
    Serial.print("Time (s): ");
    Serial.println(data.timeMs / 1000.0f);
    Serial.print("Altitude (m): ");
    Serial.println(data.altitude);
    Serial.print("Vertical velocity (m/s): ");
    Serial.println(data.verticalVelocity);
    Serial.print("Vertical acceleration (m/s²): ");
    Serial.println(data.accZ);
    Serial.print("Total acceleration (m/s²): ");
    Serial.println(data.accelMagnitude);
    Serial.print("RBF Removed: ");
    Serial.println(data.rbfRemoved);
    Serial.println("--------------------");

    data_logger.LogFlightData(data);
#else
    return;
#endif
}
