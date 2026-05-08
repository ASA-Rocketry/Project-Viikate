#ifndef FLIGHT_CODE_INCLUDE_SENSORS_H_
#define FLIGHT_CODE_INCLUDE_SENSORS_H_

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <BME280I2C.h>  //barometer
#include <SPI.h>  //spi
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Wire.h>

#include "ISM330DHCXSensor.h"
#include "data_logger.h"
//#include <SD.h> //for microSD card
//#include <SPI.h>

#define OUTPUT_DATA_RATE 1000

/**
 * @class Sensors
 * @brief Handles sensor interfacing and data acquisition for flight telemetry.
 */
class Sensors {
  public:
    explicit Sensors(DataLogger &data_logger);

    bool Initialize();
    FlightData ReadFlightData();

  private:
    // flight data
    FlightData flight_data;

    // IMU data
    struct IMU_Data_ {
        int32_t ax;
        int32_t ay;
        int32_t az;
        int32_t gx;
        int32_t gy;
        int32_t gz;
    };

    struct Mag_Data_ {
        double mx;
        double my;
        double mz;
        double heading;
    };

    struct InitialState {
        Eigen::Matrix<float, 9, 1> linear;
        Eigen::Matrix<float, 6, 1> angular;
        double linear_offsets[3];
        double angular_offsets[3];
        double angular_rate_offsets[3];
    };  // computed at the start of the flight, used as reference for vertical velocity and rotation rates

    IMU_Data_ imu_data_;
    Mag_Data_ mag_data_;
    InitialState initial_state_;

    float readAccelMagnitude(IMU_Data_ imu_data);
    float readAltitude();
    float computeVerticalVelocity(float az);
    // float readAccelZ();
    // float readRotZ();

    float _getAltitude(float pressure, float temperature);
    float readBarometer();
    IMU_Data_ readIMU(uint16_t num_samples);
    Mag_Data_ readMagnetometer();
    InitialState _computeInitialState(const IMU_Data_ &data);
    void calibrateIMU();
    void initialize_IMU();
    void initializeMagnetometer();
    void initaliseBarometer();
    void initialiseFilters();

    ISM330DHCXSensor imu;  // class member
    SFE_MMC5983MA mag;

    DataLogger &data_logger_;
    Eigen::VectorXd AccKalmanUpdate(int32_t ax, int32_t ay, int32_t az);
    Eigen::VectorXd GyroKalmanUpdate(int32_t gx, int32_t gy, int32_t gz);
};

#endif  // FLIGHT_CODE_INCLUDE_SENSORS_H_
