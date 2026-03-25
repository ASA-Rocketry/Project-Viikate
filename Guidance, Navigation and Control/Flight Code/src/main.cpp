#include "Arduino.h"
#include "data_logger.h"
#include "state_machine.h"
#include "sensors.h"

DataLogger DataLogger_;
StateMachine StateMachine_;
Sensors Sensors_(DataLogger_);

void setup() {
  Serial.begin(115200); 
  DataLogger_.initialize(); // Initialize this before anything else to log failures
  Sensors_.initialize();
  // Controls_.initialize(); 
  DataLogger_.logEvent(LogType::INFO, "SETUP COMPLETE");
}

void loop() {
  FlightData data = Sensors_.readFlightData();
  DataLogger_.logFlightData(data);
  // Controller.control(data)
  delay(100);
 
}