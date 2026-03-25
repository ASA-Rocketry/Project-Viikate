#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "data_logger.h"
#include "constants.h"

DataLogger::DataLogger(){}

void DataLogger::initialize(){
    pinMode(constants::kCsPin, OUTPUT);
    SD.begin(constants::kCsPin);

    // Find the next available file name
    int i = 0;
    while (SD.exists("flight" + String(i) + ".csv")) {
        i++;
    }
    flight_file = "flight"+ String(i) + ".csv";
    event_file = "event"+ String(i) + ".csv";

    File flight_data_file = SD.open(flight_file, FILE_WRITE); // Flight Data File
    if (flight_data_file){ // Writing headers for FDF
        flight_data_file.println("time_ms,altitude,vertical_velocity,accel_z,rot_z,accel_magnitude,rbf_removed");
        flight_data_file.close();
    }
   
    File event_data_file = SD.open(event_file, FILE_WRITE); // Event Data File
    if (event_data_file) {
        event_data_file.println("time_ms,severity,message");
        event_data_file.close();
        logEvent(LogType::kInfo, "LOG START");
    }
}

void DataLogger::logFlightData(const FlightData& data){
    File flight_data_file = SD.open(flight_file, FILE_WRITE); // Flight Data File
    if (flight_data_file){
        flight_data_file.print(data.time_ms);
        flight_data_file.print(",");
        flight_data_file.print(data.altitude);
        flight_data_file.print(",");
        flight_data_file.print(data.vertical_velocity);
        flight_data_file.print(",");
        flight_data_file.print(data.accel_z);
        flight_data_file.print(",");
        flight_data_file.print(data.rot_z);
        flight_data_file.print(",");
        flight_data_file.print(data.accel_magnitude);
        flight_data_file.print(",");
        flight_data_file.println(data.rbf_removed);
        flight_data_file.close();
    }
}


void DataLogger::logEvent(const LogType& type, const String& event){
    static String last_event = "";
    static unsigned long last_log_time = 0;
    
    // Skip writing if the message is the same as 50 milliseconds ago
    if (event == last_event && millis() - last_log_time < 50) return;
    last_event = event;
    last_log_time = millis();

    File event_data_file = SD.open(event_file, FILE_WRITE);
    if (event_data_file) {
        event_data_file.print(millis());
        event_data_file.print(",");
        event_data_file.print(static_cast<int>(type)); // Severity number
        event_data_file.print(",");
        event_data_file.println(event);
        event_data_file.close(); 
    }
}
