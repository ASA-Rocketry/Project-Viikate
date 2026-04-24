#include "data_logger.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "constants.h"

DataLogger::DataLogger() {}

void DataLogger::Initialize() {
    pinMode(constants::kCsPin, OUTPUT);
    SD.begin(constants::kCsPin);

    // Find the next available file name
    int i = 0;
    while (SD.exists("flight" + String(i) + ".csv")) {
        i++;
    }
    flight_file_ = "flight" + String(i) + ".csv";
    event_file_ = "event" + String(i) + ".csv";

    flight_data_file_ = SD.open(flight_file_, FILE_WRITE);  // Flight Data File
    if (flight_data_file_) {  // Writing headers for FDF
        flight_data_file_.println(
            "time_ms,altitude,vertical_velocity,accel_x,accel_y,accel_z,rot_x,rot_y,rot_z,mag_x,mag_y,mag_z,accel_magnitude,rbf_removed"
        );
        flight_data_file_.flush();
    }

    event_data_file_ = SD.open(event_file_, FILE_WRITE);  // Event Data File
    if (event_data_file_) {
        event_data_file_.println("time_ms,severity,message");
        event_data_file_.flush();
        LogEvent(LogType::kInfo, "LOG START");
    }
}

void DataLogger::LogFlightData(const FlightData &data) {
    if (flight_data_file_) {
        flight_data_file_.print(data.timeMs);
        flight_data_file_.print(",");
        flight_data_file_.print(data.altitude);
        flight_data_file_.print(",");
        flight_data_file_.print(data.verticalVelocity);
        flight_data_file_.print(",");
        flight_data_file_.print(data.accX);
        flight_data_file_.print(",");
        flight_data_file_.print(data.accY);
        flight_data_file_.print(",");
        flight_data_file_.print(data.accZ);
        flight_data_file_.print(",");
        flight_data_file_.print(data.rotX);
        flight_data_file_.print(",");
        flight_data_file_.print(data.rotY);
        flight_data_file_.print(",");
        flight_data_file_.print(data.rotZ);
        flight_data_file_.print(",");
        flight_data_file_.print(data.magX);
        flight_data_file_.print(",");
        flight_data_file_.print(data.magY);
        flight_data_file_.print(",");
        flight_data_file_.print(data.magZ);
        flight_data_file_.print(",");
        flight_data_file_.print(data.accelMagnitude);
        flight_data_file_.print(",");
        flight_data_file_.println(data.rbfRemoved);

        static unsigned long lastFlush = 0;
        if (millis() - lastFlush >= 1000) {  // Write to SD card every second
            flight_data_file_.flush();
            lastFlush = millis();
        }
    }
}

void DataLogger::LogEvent(const LogType &type, const String &event) {
    static String last_event = "";
    static unsigned long last_log_time = 0;

    // Skip writing if the message is the same as 250 milliseconds ago
    // to avoid flooding the system with the same error message
    if (event == last_event && millis() - last_log_time < 250)
        return;
    last_event = event;
    last_log_time = millis();

    if (event_data_file_) {
        event_data_file_.print(millis());
        event_data_file_.print(",");
        event_data_file_.print(static_cast<int>(type));  // Severity number
        event_data_file_.print(",");
        event_data_file_.println(event);
        event_data_file_.flush();
    }
}

void DataLogger::Close() {
    if (flight_data_file_) {
        flight_data_file_.close();
    }
    if (event_data_file_) {
        event_data_file_.close();
    }
}
