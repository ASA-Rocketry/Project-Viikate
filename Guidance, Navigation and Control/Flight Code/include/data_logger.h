/**
 * @file data_logger.h
 * @brief Data logging system for rocket flight telemetry and events.
 * This class handles sequential file naming, SD card interfacing, and provides
 * throttled logging for system events.
 */

#ifndef FLIGHT_CODE_INCLUDE_DATA_LOGGER_H_
#define FLIGHT_CODE_INCLUDE_DATA_LOGGER_H_

#include "constants.h"
#include <Arduino.h>
#include <SD.h>

/**
 * @class DataLogger
 * @brief Manages data persistence to an SD card.
 * * The DataLogger creates two separate CSV files per power cycle:
 * 1. Flight data
 * 2. Event logs (system status, errors, and flight phases)
 */
class DataLogger {
    public:
        /**
         * @brief Construct a new Data Logger object.
         * 
         */
        DataLogger();

        /**
         * @brief Initializes the hardware and file system.
         * * Sets up the SD card, finds the next available file index (e.g., flight3.csv),
         * and writes the CSV headers to both flight and event files.
         */
        bool Initialize();

        /**
         * @brief Checks if the data logger is initialized.
         * @return true if initialized, false otherwise.
         */
        bool IsInitialized() const;

        /**
         * @brief Records high-frequency flight telemetry.
         * @param data A constant reference to the FlightData struct containing current sensor values and the associated time stamp.
         */
        void LogFlightData(const FlightData& data);

        /**
         * @brief Records system events with severity levels and built-in throttling.
         * * To prevent system lag, identical messages are ignored if they occur 
         * within a 50ms window.
         * @param type The severity of the event (kInfo, kWarning, kError, kCritical).
         * @param event The descriptive text message to be logged.
         */
        void LogEvent(const LogType& type, const String& event);

        /**
         * @brief Closes the files safely at the end of flight.
         */
        void Close();

    private: 
        /** @brief Flag to enable or disable telemetry logging (Not in use). */
        bool log_flight_data_ = true; 

        /** @brief The dynamically generated filename for the telemetry log (e.g., "flight1.csv"). */
        String flight_file_;

        /** @brief The dynamically generated filename for the event log (e.g., "event1.csv"). */
        String event_file_;

        File flight_data_file_;

        File event_data_file_;
        
        bool initialized_ = false;
};

#endif  // FLIGHT_CODE_INCLUDE_DATA_LOGGER_H_