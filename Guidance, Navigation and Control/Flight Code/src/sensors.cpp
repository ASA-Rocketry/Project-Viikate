#include "sensors.h"
#include "Arduino.h"
#include "constants.h"
#include "Wire.h"
#include <SPI.h>
#include "ISM330DHCXSensor.h"
#include <BME280I2C.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <cstdint>

#define SAMPLE_THRESHOLD 2
#define SEA_LEVEL_PRESSURE 101300.0 // Standard sea level pressure in Pa
#define TEMP_LAPSE_RATE 0.0065 // Temperature lapse rate in K/m
#define GAS_CONSTANT 8.31432 // Universal gas constant in J/mol/K
#define MOLAR_MASS_AIR 0.0289644 // Molar mass of Earth's air in kg/mol
#define MG_TO_MPS2 0.00981f // Conversion factor from milli-g to m/s^2

ISM330DHCXSensor imu(&SPI, constants::kCsPin, 1000000);
BME280I2C bme;
BME280::TempUnit tempUnit = BME280::TempUnit_Celsius;
BME280::PresUnit presUnit = BME280::PresUnit_Pa;
SFE_MMC5983MA mag;

bool imu_initialized = false;
bool mag_initialized = false;
bool bme_initialized = false;

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
    : imu (&SPI, constants::kCsPin, 1000000), data_logger_(data_logger) {
  flight_data.altitude = 0.0f;
  flight_data.verticalVelocity = 0.0f;
  flight_data.accX = 0.0f;
  flight_data.accY = 0.0f;
  flight_data.accZ = 0.0f;
  flight_data.rotX = 0.0f;
  flight_data.rotY = 0.0f;
  flight_data.rotZ = 0.0f;
  flight_data.magX = 0.0f;
  flight_data.magY = 0.0f;
  flight_data.magZ = 0.0f;
  flight_data.accelMagnitude = 0.0f;
  flight_data.timeMs = 0;
  flight_data.rbfRemoved = false;
}

/**
 * @brief Reads current flight telemetry and populates FlightData.
 * @return FlightData structure with the latest sensor measurements.
 */
FlightData Sensors::ReadFlightData() {
  flight_data.timeMs = millis();

  uint16_t samples;

  IMU_Data_ imu_data;

  // Read each sensor ONCE
  imu.FIFO_Get_Num_Samples(&samples);
  if (samples > SAMPLE_THRESHOLD) {
    imu_data = readIMU(samples);
    flight_data.altitude         = readAltitude();
    flight_data.verticalVelocity = computeVerticalVelocity(imu_data.az);

    flight_data.accX = imu_data.ax * MG_TO_MPS2; // Convert from milli-g to m/s^2
    flight_data.accY = imu_data.ay * MG_TO_MPS2;
    flight_data.accZ = imu_data.az * MG_TO_MPS2;

    flight_data.rotX = imu_data.gx / 1000.0f; // Convert from mdps to dps
    flight_data.rotY = imu_data.gy / 1000.0f;
    flight_data.rotZ = imu_data.gz / 1000.0f;

    flight_data.accelMagnitude = sqrt(
      pow(imu_data.ax, 2) +
      pow(imu_data.ay, 2) +
      pow(imu_data.az, 2)
    );
  }
  Mag_Data_ mag_data = readMagnetometer();

  flight_data.magX    = mag_data.mx;
  flight_data.magY    = mag_data.my;
  flight_data.magZ    = mag_data.mz;
  flight_data.heading = mag_data.heading;

  flight_data.rbfRemoved = digitalRead(constants::kRbfPin);

  return flight_data;
}
/**
 * @brief Initializes the sensors, including I2C and IMU.
 */
void Sensors::Initialize() {
  Wire.begin();  // Begin I2C transmission
  
  SPI.begin();
  initialize_IMU();
  initializeMagnetometer();
  initaliseBarometer();
}

void Sensors::initialize_IMU() {

        if (!imu.begin()) {
          data_logger_.LogEvent(LogType::kCritical, "IMU INIT FAILURE");
          imu_initialized = false;
          Serial.println("ISM330DHCX IMU initialization unsuccessful.");
        } else {
          data_logger_.LogEvent(LogType::kInfo, "IMU INITIALIZED");
          imu_initialized = true;
          Serial.println("ISM330DHCX IMU initialized successfully.");
        } 
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
}

void Sensors::initializeMagnetometer() {
        if (!mag.begin(Wire)) {
          data_logger_.LogEvent(LogType::kCritical, "MAGNETOMETER INIT FAILURE");
          mag_initialized = false;
          Serial.println("MMC5983MA magnetometer initialization unsuccessful.");
        } else {
          data_logger_.LogEvent(LogType::kInfo, "MAGNETOMETER INITIALIZED");
          mag_initialized = true;
          Serial.println("MMC5983MA magnetometer initialized successfully.");
        } 
        mag.softReset();
        mag.setFilterBandwidth(100); //set filter bandwidth to 100Hz
        mag.enableAutomaticSetReset(); //set automatic set/reset
        mag.enableContinuousMode(); //enable continuous mode
        mag.setContinuousModeFrequency(100); //set reading frquency to 100Hz
}

void Sensors::initaliseBarometer() {
        if (!bme.begin()) {
          data_logger_.LogEvent(LogType::kCritical, "BAROMETER INIT FAILURE");
          bme_initialized = false;
          Serial.println("BME280 barometer initialization unsuccessful.");
        } else {
          data_logger_.LogEvent(LogType::kInfo, "BAROMETER INITIALIZED");
          bme_initialized = true;
          Serial.println("BME280 barometer initialized successfully.");
        }
        
}
/** @brief Reads the current altitude (stub implementation). */
float Sensors::readAltitude() {
  return readBarometer();
  
}

/** @brief Computes vertical velocity (stub implementation). */
// Compute vertical velocity from IMU Z-axis acceleration
float Sensors::computeVerticalVelocity(float az) {
    verticalVelocity_ = 0.0f; // Reset velocity for this example
    unsigned long now = flight_data.timeMs;
    float dt = (lastTimeMs_ > 0) ? (now - lastTimeMs_) / 1000.0f : 0.0f; // seconds
    lastTimeMs_ = now;
    // Integrate acceleration to get velocity
    verticalVelocity_ += az * dt;

    return verticalVelocity_;
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::readAccelMagnitude(IMU_Data_ imu_data) {
  return sqrt(imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay + imu_data.az * imu_data.az);
}

// reads raw sensor data
Sensors::IMU_Data_ Sensors::readIMU(uint16_t samples_to_read) {
  if (!imu_initialized) {
    return {0, 0, 0, 0, 0, 0};
  }

  int32_t accelerometer[3] = {0, 0, 0};
  int32_t gyroscope[3]     = {0, 0, 0};
  bool acc_available = false;
  bool gyr_available = false;

  IMU_Data_ imu_data = {0, 0, 0, 0, 0, 0};

  for (uint16_t i = 0; i < samples_to_read; i++) {
    uint8_t tag;
    imu.FIFO_Get_Tag(&tag);

    switch (tag) {
      case ISM330DHCX_GYRO_NC_TAG:
        imu.FIFO_GYRO_Get_Axes(gyroscope);
        gyr_available = true;
        break;
      case ISM330DHCX_XL_NC_TAG:
        imu.FIFO_ACC_Get_Axes(accelerometer);
        acc_available = true;
        break;
      default:
        break;
    }

    // Only update imu_data when we have a fresh pair of readings

    // axis rotation fixed to make accelerometer z-axis point upwards
    if (acc_available && gyr_available) {
      imu_data.az = accelerometer[0];
      imu_data.ay = accelerometer[1];
      imu_data.ax = -accelerometer[2];
      imu_data.gz = gyroscope[0];
      imu_data.gy = gyroscope[1];
      imu_data.gx = -gyroscope[2];
      acc_available = false;
      gyr_available = false;
    }
  }

  return imu_data;
}

Sensors::Mag_Data_ Sensors::readMagnetometer(){
        if (mag_initialized == true ) {
            mag.getMeasurementXYZ(&rawValueX_, &rawValueY_, &rawValueZ_);
            //mag.readFieldsXYZ(&rawValueX_, &rawValueY_, &rawValueZ_);
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

            // axis rotation fixed to make magenetometer z-axis point upwards

            mag_data.mx = scaledX;
            mag_data.mz = -scaledY;
            mag_data.my = scaledZ;
            mag_data.heading = heading;
            return mag_data;
            
        } else {
          
            Sensors::Mag_Data_ mag_data;
            mag_data.mx = 0;
            mag_data.my = 0;
            mag_data.mz = 0;
            mag_data.heading = 0;

            return mag_data;
        }
        
};

float Sensors::_getAltitude(float pressure, float temperature) {

        const float SIMPLIFIED_CONSTANTS = GAS_CONSTANT / (MOLAR_MASS_AIR * constants::kGravity); // Precompute constant part of the formula
        
        return SIMPLIFIED_CONSTANTS * temperature * logf(SEA_LEVEL_PRESSURE / pressure);
};

float Sensors::readBarometer() {
      if (bme_initialized == true) {
        float pres, temp, hum;

        bme.read(pres, temp, hum, tempUnit, presUnit);

        return _getAltitude(pres, temp);

      } else {
        return NAN;
      }

}