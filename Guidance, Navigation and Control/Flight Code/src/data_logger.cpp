#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

#include "data_logger.h"
#include "constants.h"

/**
 * @brief Constructs a new DataLogger object.
 */
DataLogger::DataLogger() {}

/**
 * @brief Initializes the SD card, prepares file names, and writes CSV headers.
 */
void DataLogger::Initialize() {
  pinMode(constants::kCsPin, OUTPUT);
  SD.begin(constants::kCsPin);

  // Find the next available file index
  int file_index = 0;
  while (SD.exists("flight" + String(file_index) + ".csv")) {
    ++file_index;
  }

  flight_file_ = "flight" + String(file_index) + ".csv";
  event_file_ = "event" + String(file_index) + ".csv";

  // Create flight data file with headers
  File flight_file = SD.open(flight_file_, FILE_WRITE);
  if (flight_file) {
    flight_file.println(
        "time_ms,altitude,vertical_velocity,accel_z,rot_z,accel_magnitude,rbf_removed");
    flight_file.close();
  }

  // Create event file with headers
  File event_file = SD.open(event_file_, FILE_WRITE);
  if (event_file) {
    event_file.println("time_ms,severity,message");
    event_file.close();
    LogEvent(LogType::kInfo, "LOG START");
  }
}

/**
 * @brief Logs high-frequency flight telemetry to the flight CSV file.
 * @param data FlightData structure containing current sensor measurements.
 */
void DataLogger::LogFlightData(const FlightData& data) {
  File flight_file = SD.open(flight_file_, FILE_WRITE);
  if (!flight_file) return;

  flight_file.print(data.time_ms);
  flight_file.print(",");
  flight_file.print(data.altitude);
  flight_file.print(",");
  flight_file.print(data.vertical_velocity);
  flight_file.print(",");
  flight_file.print(data.accel_z);
  flight_file.print(",");
  flight_file.print(data.rot_z);
  flight_file.print(",");
  flight_file.print(data.accel_magnitude);
  flight_file.print(",");
  flight_file.println(data.rbf_removed);

  flight_file.close();
}

/**
 * @brief Logs system events to the event CSV file with throttling.
 * @param type Severity of the event (kInfo, kWarning, kError, kCritical).
 * @param event Descriptive text message of the event.
 */
void DataLogger::LogEvent(const LogType& type, const String& event) {
  static String last_event = "";
  static unsigned long last_log_time = 0;

  // Skip writing if the message is identical to one logged in the last 50 ms
  if (event == last_event && millis() - last_log_time < 50) {
    return;
  }

  last_event = event;
  last_log_time = millis();

  File event_file = SD.open(event_file_, FILE_WRITE);
  if (!event_file) return;

  event_file.print(millis());
  event_file.print(",");
  event_file.print(static_cast<int>(type));  // Severity number
  event_file.print(",");
  event_file.println(event);

  event_file.close();
}