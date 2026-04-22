#include "Arduino.h"
#include "data_logger.h"
#include "state_machine.h"
#include "sensors.h"
#include "control.h"
#include "constants.h"
#include "control_hardware.h"

DataLogger data_logger;
StateMachine state_machine(data_logger);
Sensors sensors(data_logger);
ControlHardware control_hardware;
Control control;

float error;

void sendToSerial(Print &serial, FlightData data, Control control) {
  // Print values to Serial
  serial.print("Altitude: "); serial.println(data.altitude);
  serial.print("Vertical Velocity: "); serial.println(data.verticalVelocity);
  serial.print("AccelZ: "); serial.println(data.accZ);
  serial.print("RotZ: "); serial.println(data.rotZ);
  serial.print("AccelMagnitude: "); serial.println(data.accelMagnitude);
  serial.print("RBF Removed: "); serial.println(data.rbfRemoved);
  serial.print("Acc (x, y, z): "); serial.print(data.accX); serial.print(", "); serial.print(data.accY); serial.print(", "); serial.println(data.accZ);
  serial.print("Gyro Angular Rate (x, y, z): "); serial.print(data.rotX); serial.print(", "); serial.print(data.rotY); serial.print(", "); serial.println(data.rotZ);
  serial.print("Orientation (x, y, z): "); serial.print(data.oriX); serial.print(", "); serial.print(data.oriY); serial.print(", "); serial.println(data.oriZ);
  serial.print("Mag: "); serial.print(data.magX); serial.print(", "); serial.print(data.magY); serial.print(", "); serial.println(data.magZ); serial.print("Heading: "); serial.println(data.heading);
  serial.println("--------------------");

  serial.println("Control error:");
  error = control.get_error();
  serial.println(error);
  if (abs(error) < 2.0f) { // Example threshold for critical error
    digitalWrite(constants::kLEDPin, HIGH); // Turn on LED if error is small (indicating good control)
  } else {
    digitalWrite(constants::kLEDPin, LOW); // Turn off LED if error is
  // delay(50);
  }
  // Controller.control(data)
}

void setup() {
  Serial.begin(9600); 
  Serial8.begin(9600); 
  data_logger.Initialize(); // Initialize this before anything else to log failures
  sensors.Initialize();
  //control_hardware.Initialize();
  control.Initialize(); 
  data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");

  pinMode(constants::kLEDPin, OUTPUT); // Set LED pin as output
}

/**
 * @brief Arduino main loop.
 *
 * Reads sensor data, logs flight telemetry, and performs control.
 */
void loop() {
  FlightData data = sensors.ReadFlightData();
  data_logger.LogFlightData(data);

  if (Serial8.available()) {
    char incomingChar = Serial8.read();
    Serial.println(incomingChar);  // Debug to USB
  }

  control.PID(0.0f, -data.oriZ); // Example: control to maintain 90 degrees orientation around Z-axis
  sendToSerial(Serial8, data, control);
  // Controller.control(data)
}