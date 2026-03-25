#ifndef SENSORS
#define SENSORS

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
        FlightData data_;
        float readAltitude();
        float computeVerticalVelocity();
        float readAccelZ();
        float readRotatZ();
        float readAccelMagnitude();
        DataLogger& datalogger_;
};

#endif