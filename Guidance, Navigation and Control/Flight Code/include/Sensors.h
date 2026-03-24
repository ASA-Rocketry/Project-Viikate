#ifndef SENSORS_H_
#define SENSORS_H_

#include <Arduino.h>
#include "MPU6500.h"
#include "data_logger.h"

/**
 * @class Sensors
 * @brief Handles sensor interfacing and data acquisition for flight telemetry.
 */
class Sensors {
 public:
  explicit Sensors(DataLogger& data_logger);

  void Initialize();
  FlightData ReadFlightData();

 private:
  bfs::Mpu6500 imu;
  FlightData data_;

  float ReadAltitude();
  float ComputeVerticalVelocity();
  float ReadAccelZ();
  float ReadRotZ();
  float ReadAccelMagnitude();

  DataLogger& data_logger_;
};

#endif  // SENSORS_H_

// Test