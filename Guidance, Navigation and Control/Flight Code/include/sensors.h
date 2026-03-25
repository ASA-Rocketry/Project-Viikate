#ifndef FLIGHT_CODE_INCLUDE_SENSORS_H_
#define FLIGHT_CODE_INCLUDE_SENSORS_H_

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
        FlightData flight_data;
        float readAltitude();
        float computeVerticalVelocity();
        float readAccelZ();
        float readRotZ();
        float readAccelMagnitude();
        DataLogger& data_logger_;
};

#endif  // FLIGHT_CODE_INCLUDE_SENSORS_H_