#include <Arduino.h>
#include "Sensors.h"
#include "Constants.h"
#include "Wire.h"
#include "MPU6500.h"
 
Sensors::Sensors(DataLogger& datalogger) : imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM),
datalogger_(datalogger) {
    data_.altitude = 0.0;
    data_.verticalVelocity = 0.0;
    data_.accelZ = 0.0;
    data_.accelMagnitude = 0.0;
    data_.timeMs = 0.0;
    data_.rbfRemoved = false;
}

FlightData Sensors::readFlightData() {
    data_.timeMs           = millis(); // Time is always read and linked with the flight data here. Not at the time of writing.
    if (imu.Read()){
        data_.altitude         = readAltitude();
        data_.verticalVelocity = computeVerticalVelocity();
        data_.accelZ           = readAccelZ();
        data_.rotatZ           = readRotatZ();
        data_.accelMagnitude   = readAccelMagnitude();
        data_.rbfRemoved       = digitalRead(Constants::RBF_PIN);
    }
    else {
        datalogger_.LogEvent(LogType::ERROR, "IMU READ FAILURE");
    }
    return data_;
}

void Sensors::Initialize() {
    Wire.begin(); // Begin I2C transmission
    if (!imu.Begin()) {
        datalogger_.LogEvent(LogType::CRITICAL, "IMU INIT FAILURE");
    }
    else {
        datalogger_.LogEvent(LogType::INFO, "IMU INITIALIZED");
    }
}

float Sensors::readAltitude() {
    return 1.11;
}

float Sensors::computeVerticalVelocity() {
    return 2.22;
}

float Sensors::readAccelZ() {
    return imu.accel_z_mps2() + Constants::GRAVITY; // Remove gravity
}

float Sensors::readRotatZ() {
    return imu.gyro_z_radps();

}

float Sensors::readAccelMagnitude() {
    return 4.44;
}

















