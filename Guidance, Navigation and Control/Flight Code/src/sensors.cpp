#include "sensors.h"
#include "Arduino.h"
#include "constants.h"
#include "Wire.h"
#include "MPU6500.h"
 
Sensors::Sensors(DataLogger& data_logger)
    : imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM),
      data_logger(data_logger) {
    flight_data.altitude = 0.0;
    flight_data.vertical_velocity = 0.0;
    flight_data.accel_z = 0.0;
    flight_data.accel_magnitude = 0.0;
    flight_data.time_ms = 0.0;
    flight_data.rbf_removed = false;
}

FlightData Sensors::readFlightData() {
    flight_data.time_ms = millis(); // Time is always read and linked with the flight data here. Not at the time of writing.
    if (imu.Read()){
        flight_data.altitude = readAltitude();
        flight_data.vertical_velocity = computeVerticalVelocity();
        flight_data.accel_z = readAccelZ();
        flight_data.rot_z = readRotatZ();
        flight_data.accel_magnitude = readAccelMagnitude();
        flight_data.rbf_removed = digitalRead(constants::kRbfPin);
    }
    else {
        data_logger.logEvent(LogType::kError, "IMU READ FAILURE");
    }
    return flight_data;
}

void Sensors::initialize() {
    Wire.begin(); // Begin I2C transmission
    if (!imu.Begin()) {
        data_logger.logEvent(LogType::kCritical, "IMU INIT FAILURE");
    }
    else {
        data_logger.logEvent(LogType::kInfo, "IMU INITIALIZED");
    }
}

float Sensors::readAltitude() {
    return 1.11;
}

float Sensors::computeVerticalVelocity() {
    return 2.22;
}

float Sensors::readAccelZ() {
    return imu.accel_z_mps2() + constants::kGravity; // Remove gravity
}

float Sensors::readRotatZ() {
    return imu.gyro_z_radps();

}

float Sensors::readAccelMagnitude() {
    return 4.44;
}

















