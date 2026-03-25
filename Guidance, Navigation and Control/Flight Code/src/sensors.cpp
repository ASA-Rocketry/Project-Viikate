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
  flight_data.vertical_velocity = 0.0f;
  flight_data.accel_z = 0.0f;
  flight_data.rotat_z = 0.0f;
  flight_data.accel_magnitude = 0.0f;
  flight_data.time_ms = 0;
  flight_data.rbf_removed = false;
}

/**
 * @brief Reads current flight telemetry and populates FlightData.
 * @return FlightData structure with the latest sensor measurements.
 */
FlightData Sensors::readFlightData() {
  flight_data.time_ms = millis();  // Timestamp linked to flight data

  if (imu.Read()) {
    flight_data.altitude = readAltitude();
    flight_data.vertical_velocity = computeVerticalVelocity();
    flight_data.accel_z = readAccelZ();
    flight_data.rotat_z = readRotatZ();
    flight_data.accel_magnitude = readAccelMagnitude();
    flight_data.rbf_removed = digitalRead(constants::kRbfPin);
  } else {
    data_logger_.logEvent(LogType::kError, "IMU READ FAILURE");
  }

  return flight_data;
}

/**
 * @brief Initializes the sensors, including I2C and IMU.
 */
void Sensors::initialize() {
  Wire.begin();  // Begin I2C transmission
  if (!imu.Begin()) {
    data_logger_.logEvent(LogType::kCritical, "IMU INIT FAILURE");
  } else {
    data_logger_.logEvent(LogType::kInfo, "IMU INITIALIZED");
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
float Sensors::readRotatZ() {
  return imu.gyro_z_radps();
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::readAccelMagnitude() {
  return 4.44f;
}

// Test