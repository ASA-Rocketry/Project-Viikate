#include <Arduino.h>
#include <SD.H>
#include <SPI.h>
#include "DataLogger.h"
#include "Constants.h"

DataLogger::DataLogger(){}

void DataLogger::Initialize(){
    pinMode(Constants::CS_PIN, OUTPUT);
    SD.begin(Constants::CS_PIN);

    // Find the next available file name
    int i = 0;
    while (SD.exists("flight" + String(i) + ".csv")) {
        i++;
    }
    FlightFile_ = "flight"+ String(i) + ".csv";
    EventFile_ = "event"+ String(i) + ".csv";

    File FDF = SD.open(FlightFile_, FILE_WRITE); // Flight Data File
    if (FDF){ // Writing headers for FDF
        FDF.println("timeMs,altitude,verticalVelocity,accelZ,rotatZ,accelMagnitude,rbfRemoved");
        FDF.close();
    }
   
    File EDF = SD.open(EventFile_, FILE_WRITE); // Event Data File
    if (EDF) {
        EDF.println("timeMs,severity,message");
        EDF.close();
        LogEvent(LogType::INFO, "LOG START");
    }
}

void DataLogger::LogFlightData(const FlightData& data){
    File FDF = SD.open(FlightFile_, FILE_WRITE); // Flight Data File
    if (FDF){
        FDF.print(data.timeMs);
        FDF.print(",");
        FDF.print(data.altitude);
        FDF.print(",");
        FDF.print(data.verticalVelocity);
        FDF.print(",");
        FDF.print(data.accelZ);
        FDF.print(",");
        FDF.print(data.rotatZ);
        FDF.print(",");
        FDF.print(data.accelMagnitude);
        FDF.print(",");
        FDF.println(data.rbfRemoved);
        FDF.close();
    }
}


void DataLogger::LogEvent(const LogType& type, const String& event){
    static String lastEvent = "";
    static unsigned long lastLogTime = 0;
    
    // Skip writing if the message is the same as 50 milliseconds ago
    if (event == lastEvent && millis() - lastLogTime < 50) return;
    lastEvent = event;
    lastLogTime = millis();

    File EDF = SD.open(EventFile_, FILE_WRITE);
    if (EDF) {
        EDF.print(millis());
        EDF.print(",");
        EDF.print((int)type); // Severity number
        EDF.print(",");
        EDF.println(event);
        EDF.close(); 
    }
}
