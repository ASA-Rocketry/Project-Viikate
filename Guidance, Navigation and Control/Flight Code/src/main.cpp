#include "Arduino.h"
#include "data_logger.h"
#include "state_machine.h"
#include "sensors.h"

DataLogger data_logger;
StateMachine state_machine;
Sensors sensors(data_logger);

void setup() {
  Serial.begin(115200); 
  data_logger.Initialize(); // Initialize this before anything else to log failures
  sensors.Initialize();
  // Controls_.initialize(); 
  data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");
}

/**
 * @brief Arduino main loop.
 *
 * Reads sensor data, logs flight telemetry, and performs control.
 */
void loop() {
  FlightData data = sensors.ReadFlightData();
  data_logger.LogFlightData(data);
  // Controller.control(data)
  delay(100);
}