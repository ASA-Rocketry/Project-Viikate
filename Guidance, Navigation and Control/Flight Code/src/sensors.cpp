#include <BME280I2C.h>
#include <SPI.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <cstdint>
#include "Arduino.h"
#include "ISM330DHCXSensor.h"
#include "Wire.h"
#include "constants.h"
#include "pose.h"
#include "sensors.h"

#define SAMPLE_THRESHOLD 5
#define SEA_LEVEL_PRESSURE 101300.0  // Standard sea level pressure in Pa
#define TEMP_LAPSE_RATE 0.0065  // Temperature lapse rate in K/m
#define GAS_CONSTANT 8.31432  // Universal gas constant in J/mol/K
#define MOLAR_MASS_AIR 0.0289644  // Molar mass of Earth's air in kg/mol
#define RAD_TO_DEG 57.2957795131  // Conversion factor from radians to degrees
#define MG_TO_MPS2 0.00981f  // Conversion factor from milli-g to m/s^2

ISM330DHCXSensor imu(&SPI, constants::kImuCsPin, 1000000);
BME280I2C bme;
BME280::TempUnit tempUnit = BME280::TempUnit_Celsius;
BME280::PresUnit presUnit = BME280::PresUnit_Pa;
SFE_MMC5983MA mag;
KalmanFilter AccKalman;
KalmanFilter GyroKalman;

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

double dt = 1.0 / 1000.0;  // Time step between samples

bool use_filtered_data = true;  //whether to log and display filtered data

/**
 * @brief Constructs a Sensors object.
 * @param data_logger Reference to the DataLogger for event logging.
 */
Sensors::Sensors(DataLogger &data_logger) :
    imu(&SPI, constants::kImuCsPin, 1000000),
    data_logger_(data_logger) {
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

    double correctedAX, correctedAY, correctedAZ, correctedGX, correctedGY,
        correctedGZ;

    // Read each sensor ONCE
    imu.FIFO_Get_Num_Samples(&samples);
    if (samples > SAMPLE_THRESHOLD) {
        imu_data = readIMU(samples);
        flight_data.altitude = readAltitude();  // from barometer

        if (use_filtered_data
            == false) {  //if we choose not to use filtered data
            flight_data.accX = imu_data.ax;  // Convert from milli-g to m/s^2
            flight_data.accY = imu_data.ay;
            flight_data.accZ = imu_data.az;

            flight_data.rotX = imu_data.gx;
            flight_data.rotY = imu_data.gy;
            flight_data.rotZ = imu_data.gz;

            flight_data.accelMagnitude = sqrt(
                pow(imu_data.ax, 2) + pow(imu_data.ay, 2) + pow(imu_data.az, 2)
            );
        } else {  //if filtered data is to be used

            correctedAX = (imu_data.ax) + initial_state_.linear_offsets[0];
            correctedAY = (imu_data.ay) + initial_state_.linear_offsets[1];
            correctedAZ = (imu_data.az) + initial_state_.linear_offsets[2];
            correctedGX = imu_data.gx + initial_state_.angular_rate_offsets[0];
            correctedGY = imu_data.gy + initial_state_.angular_rate_offsets[1];
            correctedGZ = imu_data.gz + initial_state_.angular_rate_offsets[2];

            //get z axis pos/vel/acc estimate from kalman filter
            Eigen::VectorXd acc_estimate =
                AccKalmanUpdate(correctedAX, correctedAY, correctedAZ);

            //update accX, accY, accZ, altitude and velocity based on acc_estimates
            flight_data.accX = acc_estimate(2);
            flight_data.accY = acc_estimate(5);
            flight_data.accZ = acc_estimate(8);
            flight_data.altitude = acc_estimate(6);
            flight_data.verticalVelocity = acc_estimate(7);

            Eigen::VectorXd gyr_estimate =
                GyroKalmanUpdate(correctedGX, correctedGY, correctedGZ);

            flight_data.oriX = gyr_estimate(0) * RAD_TO_DEG;
            flight_data.oriY = gyr_estimate(2) * RAD_TO_DEG;
            flight_data.oriZ = gyr_estimate(4) * RAD_TO_DEG;

            flight_data.rotX = gyr_estimate(1);
            flight_data.rotY = gyr_estimate(3);
            flight_data.rotZ = gyr_estimate(5);

            flight_data.accelMagnitude = sqrt(
                pow(acc_estimate(2), 2) + pow(acc_estimate(5), 2)
                + pow(acc_estimate(8), 2)
            );
        };
    }
    Mag_Data_ mag_data = readMagnetometer();

    flight_data.magX = mag_data.mx;
    flight_data.magY = mag_data.my;
    flight_data.magZ = mag_data.mz;
    flight_data.heading = mag_data.heading;

    flight_data.rbfRemoved = digitalRead(constants::kRbfPin);

    return flight_data;
}

/**
 * @brief Initializes the sensors, including I2C and IMU.
 */
bool Sensors::Initialize() {
    Wire.begin();  // Begin I2C transmission

    SPI.begin();
    initialize_IMU();
    initializeMagnetometer();
    initaliseBarometer();

    Serial.println("Sensors initialized.");
    Serial.println("=== Initial State ===");

    Serial.println("Linear [position, velocity, acceleration]:");
    Serial.print("  position:     ");
    Serial.print(initial_state_.linear(0), 4);
    Serial.println(" m");
    Serial.print("  velocity:     ");
    Serial.print(initial_state_.linear(1), 4);
    Serial.println(" m/s");
    Serial.print("  acceleration (z): ");
    Serial.print(initial_state_.linear(2), 4);
    Serial.println(" m/s^2");

    Serial.println("Angular [orientation, angular_velocity]:");
    Serial.println("  orientation (X, Y, Z ):      ");
    Serial.println(initial_state_.angular(0), 4);
    Serial.println(initial_state_.angular(1), 4);
    Serial.println(initial_state_.angular(2), 4);
    Serial.println(" degree");
    Serial.println("  angular_velocity: ");
    Serial.println(initial_state_.angular(3), 4);
    Serial.println(initial_state_.angular(4), 4);
    Serial.println(initial_state_.angular(5), 4);
    Serial.println(" degree/s");

    calibrateIMU();

    Serial.println("====================");

    initialiseFilters();

    //delay(5000);

    Serial.println("Sensors initialized!");
    return true;
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
    imu.FIFO_Set_Mode(ISM330DHCX_STREAM_MODE);  // continuous overwrite
    imu.FIFO_ACC_Set_BDR(OUTPUT_DATA_RATE);
    imu.FIFO_GYRO_Set_BDR(OUTPUT_DATA_RATE);
    imu.ACC_SetOutputDataRate(OUTPUT_DATA_RATE);
    imu.GYRO_SetOutputDataRate(OUTPUT_DATA_RATE);
    imu.ACC_Enable();
    imu.GYRO_Enable();

    Serial.println("Computing initial state from IMU readings...");

    delay(100);  // wait for IMU buffer to fill up with initial readings

    Sensors::IMU_Data_ imu_data_initial;

    uint16_t init_samples;

    imu.FIFO_Get_Num_Samples(&init_samples);
    if (init_samples > SAMPLE_THRESHOLD) {
        imu_data_initial = readIMU(init_samples);
        // Use initial accelerometer z-axis reading as reference for vertical velocity
        Sensors::initial_state_ = _computeInitialState(imu_data_initial);
    }
}

Sensors::InitialState
Sensors::_computeInitialState(const Sensors::IMU_Data_ &data) {
    InitialState state;

    // ── linear  ────────────────────────────────────────────────
    float acc_x = data.ax;
    float acc_y = data.ay;
    float acc_z = data.az;

    state.linear = Eigen::Matrix<float, 9, 1>();

    state.linear << 0, 0, acc_x, 0, 0, acc_y, 0, 0, acc_z;
    // ── angular ────────────────────────────────────────────────

    float ori_z = (float)data.gz;
    float ori_y = (float)data.gy;
    float ori_x = (float)data.gx;

    state.angular = Eigen::Matrix<float, 6, 1>();

    state.angular << 0, ori_x, 0, ori_y, 0, ori_z;

    state.linear_offsets[0] = 0.0;
    state.linear_offsets[1] = 0.0;
    state.linear_offsets[2] = 0.0;

    state.angular_rate_offsets[0] = 0.0;
    state.angular_rate_offsets[1] = 0.0;
    state.angular_rate_offsets[2] = 0.0;

    return state;
}

void Sensors::initialiseFilters() {
    //creating kalman filter for accelerometer

    const int acc_n = 9;  // states:       [position, velocity, acceleration for x, y and z axis]
    const int acc_m = 3;  // measurements: [acceleration for x, y and z axis]

    Eigen::MatrixXd acc_A(acc_n, acc_n);  // state transition
    Eigen::MatrixXd acc_Q(acc_n, acc_n);  // process noise
    Eigen::MatrixXd acc_R(acc_m, acc_m);  // measurement noise
    Eigen::MatrixXd acc_H(acc_m, acc_n);  // observation matrix
    Eigen::MatrixXd acc_P(acc_n, acc_n);  // initial covariance

    // Noise variance from accelerometer noise density (60 ug/sqrt(Hz) at 104Hz)
    const double acc_var = 1.444e-5;

    //there is an easier way of doing this, however i alr typed A and H out lmao
    acc_A << 1, dt, pow(dt, 2) / 2, 0, 0, 0, 0, 0, 0,  //x axis pos
        0, 1, dt, 0, 0, 0, 0, 0, 0,  //x axis vel
        0, 0, 1, 0, 0, 0, 0, 0, 0,  //x axis acc
        0, 0, 0, 1, dt, pow(dt, 2) / 2, 0, 0, 0,  //y axis pos
        0, 0, 0, 0, 1, dt, 0, 0, 0,  //y axis vel
        0, 0, 0, 0, 0, 1, 0, 0, 0,  //y axis acc
        0, 0, 0, 0, 0, 0, 1, dt, pow(dt, 2) / 2,  //z axis pos
        0, 0, 0, 0, 0, 0, 0, 1, dt,  //z axis vel
        0, 0, 0, 0, 0, 0, 0, 0, 1;  //z axis acc

    acc_H.setZero();
    acc_H(0, 2) = 1;
    acc_H(1, 5) = 1;
    acc_H(2, 8) = 1;

    // R — accelerometer measurement noise in (ms^-2)²
    acc_R.setZero();
    acc_R(0, 0) = acc_var;
    acc_R(1, 1) = acc_var;
    acc_R(2, 2) = acc_var;

    // Q — process noise
    acc_Q.setZero();
    for (int axis = 0; axis < 3; axis++) {
        int i = axis * 3;
        acc_Q(i, i) = pow(dt, 4) / 4;
        acc_Q(i, i + 1) = pow(dt, 3) / 2;
        acc_Q(i + 1, i) = pow(dt, 3) / 2;
        acc_Q(i + 2, i) = pow(dt, 2) / 2;
        acc_Q(i, i + 2) = pow(dt, 2) / 2;
        acc_Q(i + 1, i + 1) = pow(dt, 2);
        acc_Q(i + 2, i + 1) = dt;
        acc_Q(i + 1, i + 2) = dt;
        acc_Q(i + 2, i + 2) = 1;
    }
    acc_Q *= acc_var;

    // P — initial covariance
    acc_P.setZero();
    for (int axis = 0; axis < 3; axis++) {
        int i = axis * 3;
        acc_P(i, i) = 500;  // pos - default value for now
        acc_P(i + 1, i + 1) = 10;  //vel - default value for now
        acc_P(i + 2, i + 2) = acc_var;  // rate  — matches sensor noise
    }

    Eigen::VectorXd acc_x0 = initial_state_.linear.cast<double>();
    AccKalman.init(acc_A, acc_Q, acc_R, acc_H, acc_P, 0.0, acc_x0);

    const int gyr_n = 6;  // states:       [roll(°), roll_rate(°/s), pitch(°), pitch_rate(°/s), yaw(°), yaw_rate(°/s)]
    const int gyr_m = 3;  // measurements: [roll_rate(°/s), pitch_rate(°/s), yaw_rate(°/s)]

    Eigen::MatrixXd gyr_A(gyr_n, gyr_n);
    Eigen::MatrixXd gyr_Q(gyr_n, gyr_n);
    Eigen::MatrixXd gyr_R(gyr_m, gyr_m);
    Eigen::MatrixXd gyr_H(gyr_m, gyr_n);
    Eigen::MatrixXd gyr_P(gyr_n, gyr_n);

    // sigma in degrees/s,  var in (degrees/s)²
    const double gyr_noise_density_mdps = 8.0;
    const double gyr_noise_density_dps = gyr_noise_density_mdps * 1e-3;
    const double gyr_sigma_dps = gyr_noise_density_dps * sqrt(OUTPUT_DATA_RATE);
    const double gyr_var_dps2 = gyr_sigma_dps * gyr_sigma_dps;

    gyr_A.setZero();
    gyr_A(0, 0) = 1;
    gyr_A(0, 1) = dt;  // roll    (°)  += roll_rate  (°/s) * dt
    gyr_A(1, 1) = 1;
    gyr_A(2, 2) = 1;
    gyr_A(2, 3) = dt;  // pitch   (°)  += pitch_rate (°/s) * dt
    gyr_A(3, 3) = 1;
    gyr_A(4, 4) = 1;
    gyr_A(4, 5) = dt;  // yaw     (°)  += yaw_rate   (°/s) * dt
    gyr_A(5, 5) = 1;

    // Observe angular rate directly on each axis
    gyr_H.setZero();
    gyr_H(0, 1) = 1;  // roll_rate  (°/s)
    gyr_H(1, 3) = 1;  // pitch_rate (°/s)
    gyr_H(2, 5) = 1;  // yaw_rate   (°/s)

    // R — gyro measurement noise in (°/s)²
    gyr_R.setZero();
    gyr_R(0, 0) = gyr_var_dps2 * 50;
    gyr_R(1, 1) = gyr_var_dps2 * 50;
    gyr_R(2, 2) = gyr_var_dps2 * 50;

    // Q — process noise in (°)² and (°/s)², block diagonal
    gyr_Q.setZero();
    for (int axis = 0; axis < 3; axis++) {
        int i = axis * 2;
        gyr_Q(i, i) = pow(dt, 2);  // angle    (°)²
        gyr_Q(i, i + 1) = dt;  // cross    (°)(°/s)
        gyr_Q(i + 1, i) = dt;  // cross    (°/s)(°)
        gyr_Q(i + 1, i + 1) = 1.0;  // rate     (°/s)²
    }
    gyr_Q *= gyr_var_dps2;

    const double gyr_Q_scale = 1;  // tune this value
    gyr_Q *= gyr_var_dps2 * gyr_Q_scale;

    // P — initial covariance in (°)² and (°/s)²
    gyr_P.setZero();
    for (int axis = 0; axis < 3; axis++) {
        int i = axis * 2;
        gyr_P(i, i) =
            180.0 * 180.0;  // angle — full 180° uncertainty at startup
        gyr_P(i + 1, i + 1) = gyr_var_dps2;  // rate  — matches sensor noise
    }

    Eigen::VectorXd gyr_x0 = initial_state_.angular.cast<double>();
    GyroKalman.init(gyr_A, gyr_Q, gyr_R, gyr_H, gyr_P, 0.0, gyr_x0);
}

void Sensors::calibrateIMU() {
    Serial.println("IMU calibration starting — hold still for 5 seconds...");

    // Accumulate samples over 5 seconds
    unsigned long calibration_start = millis();

    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr;
    Eigen::Vector3d acc;
    int sample_count = 0;

    while (millis() - calibration_start < constants::CALIBRATION_DURATION_MS) {
        uint16_t samples;
        imu.FIFO_Get_Num_Samples(&samples);

        if (samples > SAMPLE_THRESHOLD) {
            IMU_Data_ cal_data = readIMU(samples);

            acc << cal_data.ax, cal_data.ay, cal_data.az;
            gyr << cal_data.gx, cal_data.gy, cal_data.gz;

            acc_sum += acc;
            gyr_sum += gyr;
            sample_count++;
        }

        delay(10);  // ~100Hz polling rate, matches IMU ODR
    }

    if (sample_count == 0) {
        Serial.println("Calibration failed — no samples received.");
        return;
    }

    // Average over all samples
    Eigen::VectorXd acc_mean = acc_sum / sample_count;
    Eigen::VectorXd gyr_mean = gyr_sum / sample_count;

    // Linear offsets — position and velocity should be 0, gravity subtracted from acc
    Sensors::initial_state_.linear_offsets[0] =
        -acc_mean(0);  // position  offset
    Sensors::initial_state_.linear_offsets[1] =
        -acc_mean(1);  // velocity  offset
    Sensors::initial_state_.linear_offsets[2] =
        -acc_mean(2);  // acc offset (residual after gravity removal)

    // Angular rate offsets — gyro bias at rest should be 0
    Sensors::initial_state_.angular_rate_offsets[0] =
        -gyr_mean(0);  // roll_rate  offset
    Sensors::initial_state_.angular_rate_offsets[1] =
        -gyr_mean(1);  // pitch_rate offset
    Sensors::initial_state_.angular_rate_offsets[2] =
        -gyr_mean(2);  // yaw_rate   offset

    Serial.print("IMU calibration complete (");
    Serial.print(sample_count);
    Serial.println(" samples).");
    Serial.println("Offsets:");
    Serial.print("  linear (pos, vel, acc): ");
    Serial.print(Sensors::initial_state_.linear_offsets[0], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.linear_offsets[1], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.linear_offsets[2], 4);
    Serial.println(" m/s^2");
    Serial.print("  angular (roll, pitch, yaw): ");
    Serial.print(Sensors::initial_state_.angular_offsets[0], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.angular_offsets[1], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.angular_offsets[2], 4);
    Serial.println(" deg");
    Serial.print("  angular rate (roll, pitch, yaw): ");
    Serial.print(Sensors::initial_state_.angular_rate_offsets[0], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.angular_rate_offsets[1], 4);
    Serial.print(", ");
    Serial.print(Sensors::initial_state_.angular_rate_offsets[2], 4);
    Serial.println(" deg/s");

    AccKalman.reset();
    // GyroKalman.reset();
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
    mag.setFilterBandwidth(OUTPUT_DATA_RATE);  //set filter bandwidth to 100Hz
    mag.enableAutomaticSetReset();  //set automatic set/reset
    mag.enableContinuousMode();  //enable continuous mode
    mag.setContinuousModeFrequency(
        OUTPUT_DATA_RATE
    );  //set reading frquency to 100Hz
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
    verticalVelocity_ = 0.0f;  // Reset velocity for this example
    unsigned long now = flight_data.timeMs;
    float dt =
        (lastTimeMs_ > 0) ? (now - lastTimeMs_) / 1000.0f : 0.0f;  // seconds
    lastTimeMs_ = now;
    // Integrate acceleration to get velocity
    verticalVelocity_ += az * dt;

    return verticalVelocity_;
}

/** @brief Computes total acceleration magnitude (stub implementation). */
float Sensors::readAccelMagnitude(IMU_Data_ imu_data) {
    return sqrt(
        imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay
        + imu_data.az * imu_data.az
    );
}

// reads raw sensor data
Sensors::IMU_Data_ Sensors::readIMU(uint16_t samples_to_read) {
    if (!imu_initialized) {
        return {0, 0, 0, 0, 0, 0};
    }

    int32_t accelerometer[3] = {0, 0, 0};
    int32_t gyroscope[3] = {0, 0, 0};
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
            imu_data.az = -accelerometer[0] * MG_TO_MPS2;
            imu_data.ay = accelerometer[1] * MG_TO_MPS2;
            imu_data.ax = accelerometer[2] * MG_TO_MPS2;
            imu_data.gz = gyroscope[0] / 1000.0f;  // Convert from mdps to dps
            imu_data.gy = gyroscope[1] / 1000.0f;  // Convert from mdps to dps
            imu_data.gx = gyroscope[2] / 1000.0f;  // Convert from mdps to dps
            acc_available = false;
            gyr_available = false;
        }
    }

    return imu_data;
}

Sensors::Mag_Data_ Sensors::readMagnetometer() {
    if (mag_initialized == true) {
        mag.getMeasurementXYZ(&rawValueX_, &rawValueY_, &rawValueZ_);
        //mag.readFieldsXYZ(&rawValueX_, &rawValueY_, &rawValueZ_);
        scaledX = (double)rawValueX_ - 131072.0;
        scaledX /= 131072.0;

        scaledZ = (double)+rawValueY_ - 131072.0;
        scaledZ /= 131072.0;

        scaledY = (double)-rawValueZ_ - 131072.0;
        scaledY /= 131072.0;

        heading = atan2(scaledX, 0 - scaledY);

        // atan2 returns a value between +PI and -PI
        // Convert to degrees
        heading /= PI;
        heading *= 180;
        heading += 180;

        Sensors::Mag_Data_ mag_data;

        // axis rotation fixed to make magenetometer z-axis point upwards
        // TODO: fix magnetometer axis rotation
        mag_data.mx = scaledX;
        mag_data.mz = scaledZ;
        mag_data.my = scaledY;
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
    const float SIMPLIFIED_CONSTANTS = GAS_CONSTANT
        / (MOLAR_MASS_AIR
           * constants::kGravity);  // Precompute constant part of the formula

    return SIMPLIFIED_CONSTANTS * temperature
        * logf(SEA_LEVEL_PRESSURE / pressure);
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

VectorXd Sensors::AccKalmanUpdate(int32_t ax, int32_t ay, int32_t az) {
    Eigen::VectorXd z_vector(3);
    z_vector << (float)ax, (float)ay, (float)az;
    AccKalman.update(z_vector);
    return AccKalman.state();
}

VectorXd Sensors::GyroKalmanUpdate(int32_t gx, int32_t gy, int32_t gz) {
    Eigen::VectorXd z_vector(3);
    z_vector << static_cast<double>(gx), static_cast<double>(gy),
        static_cast<double>(gz);
    GyroKalman.update(z_vector);
    return GyroKalman.state();
}
