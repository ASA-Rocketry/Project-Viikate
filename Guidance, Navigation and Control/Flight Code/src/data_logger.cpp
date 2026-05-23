#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "constants.h"
#include "data_logger.h"
#include "debug_prints.h"

DataLogger::DataLogger() {}

bool DataLogger::Initialize() {
    
    // Reset initialization state before attempting setup
    initialized_ = false;

    // Initialize SD card interface
    if (!SD.begin(BUILTIN_SDCARD)) {
    DEBUG_PRINTLN("SD.begin failed");
    return false;
    }

    // Find the next available file name
    int i = 0;
    String path;
    do {
        path = "flight" + String(i) + ".csv";
        i++;
    } while (SD.exists(path.c_str()));

    // Store generated file names
    flight_file_ = path;
    event_file_ = "event" + String(i - 1) + ".csv";

    // Open flight data log file
    flight_data_file_ = SD.open(flight_file_.c_str(), FILE_WRITE);  // Flight Data File
    
    // Check if the file was opened successfully
    if (!flight_data_file_) {
    DEBUG_PRINTLN("ERROR: Failed to open flight data file");
    return false;
    }

    // Write CSV header for flight data
    flight_data_file_.println(
        "time_ms,altitude,vertical_velocity,"
        "accel_x,accel_y,accel_z,"
        "rot_x,rot_y,rot_z,"
        "mag_x,mag_y,mag_z,"
        "accel_magnitude,rbf_removed"
    );

    flight_data_file_.flush();

    // Open event log file
    event_data_file_ = SD.open(event_file_.c_str(), FILE_WRITE);  // Event Data File

    if (!event_data_file_) {
        DEBUG_PRINTLN("ERROR: Failed to open event log file");   
        flight_data_file_.close();// Clean up already-opened flight file

        return false;
    }

    // Write CSV header for event log
    event_data_file_.println("time_ms,severity,message");

    event_data_file_.flush();

    // Initial log entry
    LogEvent(LogType::kInfo, "LOG START");

    // Mark subsystem as successfully initialized
    initialized_ = true;

    DEBUG_PRINTLN("DataLogger initialized successfully!");

    return true;
}

bool DataLogger::IsInitialized() const {
    return initialized_;
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
