#include "sensors.h"
#include "Arduino.h"
#include "constants.h"
#include "Wire.h"
#include <SPI.h>
#include "ISM330DHCXSensor.h"
#include <BME280I2C.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

ISM330DHCXSensor imu(&SPI, constants::kCsPin, 1000000);
BME280I2C bme;
BME280::TempUnit tempUnit = BME280::TempUnit_Celsius;
BME280::PresUnit presUnit = BME280::PresUnit_Pa;
SFE_MMC5983MA mag;

float verticalVelocity_ = 0.0f;
unsigned long lastTimeMs_ = 0;

 // magnetometer values
uint32_t rawValueX_ = 0;
uint32_t rawValueY_ = 0;
uint32_t rawValueZ_ = 0; 

double scaledX = 0;
double scaledY = 0;
double scaledZ = 0;
double heading = 0;


/**
 * @brief Constructs a Sensors object.
 * @param data_logger Reference to the DataLogger for event logging.
 */
Sensors::Sensors(DataLogger& data_logger)
    : imu(&SPI, constants::kCsPin, 1000), data_logger_(data_logger) {
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

  if (readIMU().ax != 0 || readIMU().ay != 0 || readIMU().az != 0) { // Basic check to ensure IMU is providing data
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
  
  SPI.begin();
  if (!imu.begin()) {
    data_logger_.LogEvent(LogType::kCritical, "IMU INIT FAILURE");
  } else {
    data_logger_.LogEvent(LogType::kInfo, "IMU INITIALIZED");
  }
  initialize_IMU();
  initializeMagnetometer();
  initaliseBarometer();
}

void Sensors::initialize_IMU() {

        imu.begin();
        imu.ACC_SetFullScale(16);
        imu.GYRO_SetFullScale(2000);
        imu.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);   // continuous overwrite
        imu.FIFO_ACC_Set_BDR(104);
        imu.FIFO_GYRO_Set_BDR(104);
        imu.ACC_SetOutputDataRate(104);
        imu.GYRO_SetOutputDataRate(104);
        imu.ACC_Enable();
        imu.GYRO_Enable();

        delay(100);

        Serial.println("ISM330DHCX IMU initialized successfully.");
}

void Sensors::initializeMagnetometer() {
        if (!mag.begin(Wire)) {
                Serial.println("Failed to initialize MMC5983MA magnetometer!");
                while (1);
        }
        mag.softReset();
        mag.setFilterBandwidth(100); //set filter bandwidth to 100Hz
        mag.setContinuousModeFrequency(100); //set reading frquency to 100Hz
        mag.enableAutomaticSetReset(); //set automatic set/reset
        mag.enableContinuousMode(); //enable continuous mode

        Serial.println("MMC5983MA magnetometer initialized successfully.");
}
void Sensors::initaliseBarometer() {
        if (!bme.begin()) {
                Serial.println("Failed to initialize BME280 sensor!");
                while (1);
        }
        bme.begin();

        Serial.println("BME280 barometer initialized successfully.");
        
}
/** @brief Reads the current altitude (stub implementation). */
float Sensors::readAltitude() {
  return readBarometer();
  
}

/** @brief Computes vertical velocity (stub implementation). */
// Compute vertical velocity from IMU Z-axis acceleration
float Sensors::computeVerticalVelocity() {
    verticalVelocity_ = 0.0f; // Reset velocity for this example
    unsigned long now = flight_data.timeMs;
    float dt = (lastTimeMs_ > 0) ? (now - lastTimeMs_) / 1000.0f : 0.0f; // seconds
    lastTimeMs_ = now;

    float az = readAccelZ(); // acceleration in m/s^2, gravity-compensated

    // Integrate acceleration to get velocity
    verticalVelocity_ += az * dt;

    return verticalVelocity_;
}

/** @brief Reads linear acceleration along Z-axis, gravity-compensated. */
float Sensors::readAccelZ() {
  float accel_z_mps2 = readIMU().az;
  return accel_z_mps2 + constants::kGravity;
}

/** @brief Reads angular velocity around Z-axis. */
float Sensors::readRotZ() {
  return readIMU().gz;
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::readAccelMagnitude() {
  return 4.44f;
}

// reads raw sensor data
Sensors::IMU_Data_ Sensors::readIMU() {

        int32_t accelerometer[3];
        int32_t gyroscope[3];
        imu.FIFO_ACC_Get_Axes(accelerometer);
        imu.FIFO_GYRO_Get_Axes(gyroscope);

        Sensors::IMU_Data_ imu_data;

        imu_data.ax = accelerometer[0];
        imu_data.ay = accelerometer[1];
        imu_data.az = accelerometer[2];

        imu_data.gx = gyroscope[0];
        imu_data.gy = gyroscope[1];
        imu_data.gz = gyroscope[2];
        return imu_data;       
}

Sensors::Mag_Data_ Sensors::readMagnetometer(){
        
        mag.readFieldsXYZ(&rawValueX_, &rawValueY_, &rawValueZ_);

        scaledX = (double)rawValueX_ - 131072.0;
        scaledX /= 131072.0;

        scaledY = (double)rawValueY_ - 131072.0;
        scaledY /= 131072.0;

        scaledZ = (double)rawValueZ_ - 131072.0;
        scaledZ /= 131072.0;

        heading = atan2(scaledX, 0 - scaledY);

        // atan2 returns a value between +PI and -PI
        // Convert to degrees
        heading /= PI;
        heading *= 180;
        heading += 180;

        Sensors::Mag_Data_ mag_data;

        mag_data.mx = scaledX;
        mag_data.my = scaledY;
        mag_data.mz = scaledZ;

        return mag_data;
};

float Sensors::_getAltitude(float pressure, float temperature) {
        const float SEA_LEVEL_PRESSURE = 101300.0; // Standard sea level pressure in Pa
        const float TEMP_LAPSE_RATE = 0.0065; // Temperature lapse rate in K/m
        const float GAS_CONSTANT = 8.31432; // Universal gas constant in J/mol/K
        const float MOLAR_MASS_AIR = 0.0289644; // Molar mass of Earth's air in kg/mol
        const float GRAVITY = 9.80665; // Gravitational acceleration in m/s^2

        const float SIMPLIFIED_CONSTANTS = GAS_CONSTANT / (MOLAR_MASS_AIR * GRAVITY); // Precompute constant part of the formula
        
        return SIMPLIFIED_CONSTANTS * temperature * logf(SEA_LEVEL_PRESSURE / pressure);
};

float Sensors::readBarometer() {

        float pres, temp, hum;

        bme.read(pres, temp, hum, tempUnit, presUnit);

        return _getAltitude(pres, temp);
}