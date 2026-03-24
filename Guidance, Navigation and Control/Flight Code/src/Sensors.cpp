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
  data_.altitude = 0.0f;
  data_.vertical_velocity = 0.0f;
  data_.accel_z = 0.0f;
  data_.rot_z = 0.0f;
  data_.accel_magnitude = 0.0f;
  data_.time_ms = 0;
  data_.rbf_removed = false;
}

/**
 * @brief Reads current flight telemetry and populates FlightData.
 * @return FlightData structure with the latest sensor measurements.
 */
FlightData Sensors::ReadFlightData() {
  data_.time_ms = millis();  // Timestamp linked to flight data

  if (imu.Read()) {
    data_.altitude = ReadAltitude();
    data_.vertical_velocity = ComputeVerticalVelocity();
    data_.accel_z = ReadAccelZ();
    data_.rot_z = ReadRotZ();
    data_.accel_magnitude = ReadAccelMagnitude();
    data_.rbf_removed = digitalRead(constants::kRbfPin);
  } else {
    data_logger_.LogEvent(LogType::kError, "IMU READ FAILURE");
  }

  return data_;
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
float Sensors::ReadAltitude() {
  return 1.11f;
}

/** @brief Computes vertical velocity (stub implementation). */
float Sensors::ComputeVerticalVelocity() {
  return 2.22f;
}

/** @brief Reads linear acceleration along Z-axis, gravity-compensated. */
float Sensors::ReadAccelZ() {
  return imu.accel_z_mps2() + constants::kGravity;
}

/** @brief Reads angular velocity around Z-axis. */
float Sensors::ReadRotZ() {
  return imu.gyro_z_radps();
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::ReadAccelMagnitude() {
  return 4.44f;
}