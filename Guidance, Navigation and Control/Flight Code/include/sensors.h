#ifndef FLIGHT_CODE_INCLUDE_SENSORS_H_
#define FLIGHT_CODE_INCLUDE_SENSORS_H_

#include "Arduino.h"
#include "MPU6500.h"
#include "data_logger.h"

class Sensors {
    public: 
        Sensors(DataLogger& datalogger);
        void initialize();
        FlightData readFlightData();

    private: 
        bfs::Mpu6500 imu;
        FlightData flight_data;
        float readAltitude();
        float computeVerticalVelocity();
        float readAccelZ();
        float readRotatZ();
        float readAccelMagnitude();
        DataLogger& data_logger;
};

#endif  // FLIGHT_CODE_INCLUDE_SENSORS_H_