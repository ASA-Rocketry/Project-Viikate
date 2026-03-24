#include "Arduino.h"
#include "data_logger.h"
#include "state_machine.h"
#include "sensors.h"

// Global objects
DataLogger data_logger;
StateMachine state_machine;
Sensors sensors(data_logger);

/**
 * @brief Arduino setup function.
 *
 * Initializes serial communication, sensors, and data logger.
 * Logs setup completion.
 */
void setup() {
  Serial.begin(115200);

  // Initialize data logger before other components to capture failures
  data_logger.Initialize();
  sensors.Initialize();

  // Log setup completion
  data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");
}

/**
 * @brief Arduino main loop.
 *
 * Reads sensor data, logs flight telemetry, and performs control.
 */
void loop() {
  FlightData flight_data = sensors.ReadFlightData();

  // Log flight telemetry
  data_logger.LogFlightData(flight_data);

  // Example control call (currently commented out)
  // controller.Control(flight_data);

  delay(100);
}