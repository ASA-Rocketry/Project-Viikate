#include <Arduino.h>
#include "Datalogger.h"
#include "StateMachine.h"
#include "Sensors.h"


DataLogger DataLogger_;
StateMachine StateMachine_;
Sensors Sensors_(DataLogger_);

void setup() {
  Serial.begin(115200); 
  DataLogger_.Initialize(); // Initialize this before anything else to log failures
  Sensors_.Initialize();
  // Controls_.Initialize(); 
  DataLogger_.LogEvent(LogType::INFO, "SETUP COMPLETE");
}

void loop() {
  FlightData data = Sensors_.readFlightData();
  DataLogger_.LogFlightData(data);
  // Controller.control(data)
  delay(100);
 
}