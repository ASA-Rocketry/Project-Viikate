#include "Arduino.h"
#include "data_logger.h"
#include "state_machine.h"
#include "sensors.h"

DataLogger data_logger;
StateMachine state_machine(data_logger);
Sensors sensors(data_logger);

void setup() {
  Serial.begin(115200); 
  while (!Serial);
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
  // Print values to Serial
  Serial.print("Altitude: "); Serial.println(data.altitude);
  Serial.print("Vertical Velocity: "); Serial.println(data.verticalVelocity);
  Serial.print("AccelZ: "); Serial.println(data.accZ);
  Serial.print("RotZ: "); Serial.println(data.rotZ);
  Serial.print("AccelMagnitude: "); Serial.println(data.accelMagnitude);
  Serial.print("RBF Removed: "); Serial.println(data.rbfRemoved);
  Serial.println("--------------------");
  // Controller.control(data)
  delay(100);
}