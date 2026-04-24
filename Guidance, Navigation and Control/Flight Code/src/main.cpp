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

struct TelemetryData {
  float altitude;
  float verticalVelocity;
  float accZ;
  float rotZ;
  float error;
  uint32_t timestamp;
} __attribute__((packed));

void sendToSerial(Print &serial, FlightData data, Control control) {
  TelemetryData telem;
  telem.altitude = data.altitude;
  telem.verticalVelocity = data.verticalVelocity;
  telem.accZ = data.accZ;
  telem.rotZ = data.rotZ;
  telem.error = control.get_error();
  telem.timestamp = micros();

  serial.write((uint8_t*)&telem, sizeof(telem));
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
  control.PID(0.0f, -data.oriZ); // Example: control to maintain 90 degrees orientation around Z-axis
  sendToSerial(Serial8, data, control);
  // Controller.control(data)

  // delay(50);
}