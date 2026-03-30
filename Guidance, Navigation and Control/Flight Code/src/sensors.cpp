#include "sensors.h"
#include "Arduino.h"
#include "constants.h"
#include "Wire.h"
#include "MPU6500.h"

/**
 * @brief Constructs a Sensors object.
 * @param data_logger Reference to the DataLogger for event logging.
 */
Sensors::Sensors(DataLogger& data_logger)
    : imu(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM), data_logger_(data_logger) {
  flight_data.altitude = 0.0f;
  flight_data.verticalVelocity = 0.0f;
  flight_data.accZ = 0.0f;
  flight_data.rotZ = 0.0f;
  flight_data.accelMagnitude = 0.0f;
  flight_data.timeMs = 0;
  flight_data.rbfRemoved = false;
}

/**
 * @brief Reads current flight telemetry and populates FlightData.
 * @return FlightData structure with the latest sensor measurements.
 */
FlightData Sensors::ReadFlightData() {
  flight_data.timeMs = millis();  // Timestamp linked to flight data

  if (imu.Read()) {
    flight_data.altitude = readAltitude();
    flight_data.verticalVelocity = computeVerticalVelocity();
    flight_data.accZ = readAccelZ();
    flight_data.rotZ = readRotZ();
    flight_data.accelMagnitude = readAccelMagnitude();
    flight_data.rbfRemoved = digitalRead(constants::kRbfPin);
  } else {
    data_logger_.LogEvent(LogType::kError, "IMU READ FAILURE");
  }

  return flight_data;
}

/**
 * @brief Initializes the sensors, including I2C and IMU.
 */
void Sensors::Initialize() {
  Wire.begin();  // Begin I2C transmission
  if (!imu.Begin()) {
    data_logger_.LogEvent(LogType::kCritical, "IMU INIT FAILURE");
  } else {
    data_logger_.LogEvent(LogType::kInfo, "IMU INITIALIZED");
  }
}

/** @brief Reads the current altitude (stub implementation). */
float Sensors::readAltitude() {
  return 1.11f;
}

/** @brief Computes vertical velocity (stub implementation). */
float Sensors::computeVerticalVelocity() {
  return 2.22f;
}

/** @brief Reads linear acceleration along Z-axis, gravity-compensated. */
float Sensors::readAccelZ() {
  return imu.accel_z_mps2() + constants::kGravity;
}

/** @brief Reads angular velocity around Z-axis. */
float Sensors::readRotZ() {
  return imu.gyro_z_radps();
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::readAccelMagnitude() {
  return 4.44f;
}