#include <ISM330DHCXSensor.h> //IMU
#include <SPI.h> //spi
//#include <BME280> //barometer
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
//#include <SD.h> //for microSD card
//#include <SPI.h>
#include <LittleFS.h> //flash, may use later
#include <algorithm> // for min/max_element
#include <iterator>  // for begin/end
#include <bits/stdc++.h>
using namespace std;

File flightFile;

LittleFS_Program myFS; // Use Program Flash

//logging time for imu
unsigned long lastLogTime = 0;
const int logInterval = 1; // Log at 100Hz (every 10ms)

// imu pins
const int myMOSI = 11;  
const int myMISO = 12;   
const int mySCK  = 13;  
const int myCS   = 10;  

// imu sensor object
ISM330DHCXSensor IMU(&SPI, myCS);

// How many sensor samples we want to store
const int HISTORY_SIZE = 2500;
const int maxlen=HISTORY_SIZE/10;

//final cal values
int32_t acc_calibration[3];
int32_t gyro_calibration[3];

void setup() {
  Serial.begin(112500);

  //initialize imu
  SPI.begin();
  Serial.println("SPI");
  IMU.begin();
  Serial.println("IMU");

  File flightFile = myFS.open("flight.bin", FILE_WRITE);

  // SET FULL SCALE FOR IMU
  IMU.ACC_SetFullScale(16);
  IMU.GYRO_SetFullScale(2000);
  //DANGIT WHY IS THIS ALL CAPS
  //SET DATA RATE
  IMU.ACC_SetOutputDataRate(100);
  IMU.GYRO_SetOutputDataRate(100);
  //ENABLE
  IMU.ACC_Enable();
  IMU.GYRO_Enable();

  Serial.println("HELLOOO");
  int32_t accelerometer[3];
  int32_t gyroscope[3];

  //arrays to store cal values
  int32_t acc_x[maxlen];
  int32_t acc_y[maxlen];
  int32_t acc_z[maxlen];
  int32_t gyro_x[maxlen];
  int32_t gyro_y[maxlen];
  int32_t gyro_z[maxlen];

  IMU.ACC_GetAxes(accelerometer);

  IMU.GYRO_GetAxes(gyroscope);

  delay(100);

  for (int i=0; i<maxlen; i++){
    //get values and print for ref
    IMU.ACC_GetAxes(accelerometer);
    IMU.GYRO_GetAxes(gyroscope);
    //store in array for calibration later
    acc_x[i] = accelerometer[0];
    acc_y[i] = accelerometer[1];
    acc_z[i] = accelerometer[2];
    gyro_x[i] = gyroscope[0];
    gyro_y[i] = gyroscope[1];
    gyro_z[i] = gyroscope[2];
    delay(10);
  }
  //this is supposed to be to find max/min values of each but.
  //this is. a lot of definition statements. im not sure if this si the best way of doing this ngl
  //i will adjust if i am being a stupid
  int min_x_acc = *std::min_element(std::begin(acc_x), std::end(acc_x));
  int max_x_acc = *std::max_element(std::begin(acc_x), std::end(acc_x));
  int min_y_acc = *std::min_element(std::begin(acc_y), std::end(acc_y));
  int max_y_acc = *std::max_element(std::begin(acc_y), std::end(acc_y));
  int min_z_acc = *std::min_element(std::begin(acc_z), std::end(acc_z));
  int max_z_acc = *std::max_element(std::begin(acc_z), std::end(acc_z));

  int min_x_gyro = *std::min_element(std::begin(gyro_x), std::end(gyro_x));
  int max_x_gyro = *std::max_element(std::begin(gyro_x), std::end(gyro_x));
  int min_y_gyro = *std::min_element(std::begin(gyro_y), std::end(gyro_y));
  int max_y_gyro = *std::max_element(std::begin(gyro_y), std::end(gyro_y));
  int min_z_gyro = *std::min_element(std::begin(gyro_z), std::end(gyro_z));
  int max_z_gyro = *std::max_element(std::begin(gyro_z), std::end(gyro_z));

  //print range for ref
  Serial.print("Acc X range: ");
  Serial.print(min_x_acc);
  Serial.print(", ");
  Serial.println(max_x_acc);
  Serial.print("Acc Y range: ");
  Serial.print(min_y_acc);
  Serial.print(", ");
  Serial.println(max_y_acc);
  Serial.print("Acc Z range: ");
  Serial.print(min_z_acc);
  Serial.print(", ");
  Serial.println(max_z_acc);
  Serial.print("Gyro X range: ");
  Serial.print(min_x_gyro);
  Serial.print(", ");
  Serial.println(max_x_gyro);
  Serial.print("Gyro Y range: ");
  Serial.print(min_y_gyro);
  Serial.print(", ");
  Serial.println(max_y_gyro);
  Serial.print("Gyro Z range: ");
  Serial.print(min_z_gyro);
  Serial.print(", ");
  Serial.println(max_z_gyro);
  delay(100);

  //calculate and print calibration values (take avg of max and min)
  //is this the best way of calibrating????????????
  acc_calibration[0] = (max_x_acc+min_x_acc)/2;
  acc_calibration[1] = (max_y_acc+min_y_acc)/2;
  acc_calibration[2] = (max_z_acc+min_z_acc)/2;
  gyro_calibration[0] = (max_x_gyro+min_x_gyro)/2;
  gyro_calibration[1] = (max_y_gyro+min_y_gyro)/2;
  gyro_calibration[2] = (max_z_gyro+min_z_gyro)/2;

  Serial.print("Final acc calibration in :");
  Serial.print(acc_calibration[0]);
  Serial.print(", ");
  Serial.print(acc_calibration[1]);
  Serial.print(", ");
  Serial.println(acc_calibration[2]);

  Serial.print("Final gyro calibration in rad/s:");
  Serial.print(gyro_calibration[0]);
  Serial.print(", ");
  Serial.print(gyro_calibration[1]);
  Serial.print(", ");
  Serial.println(gyro_calibration[2]);

  Serial.println("CAL END");
}


void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  // 1. Check if it's time to take a sample
  if (currentTime - lastLogTime >= logInterval) {
    lastLogTime = currentTime;
    // Using the STMduino library methods
    IMU.ACC_GetAxes(accelerometer);
    Serial.print("Acc[mg]:");
    Serial.print(accelerometer[0]-acc_calibration[0]);
    Serial.print(", ");
    Serial.print(accelerometer[1]-acc_calibration[1]);
    Serial.print(", ");
    Serial.print(accelerometer[2]-acc_calibration[2]);
    IMU.GYRO_GetAxes(gyroscope);
    Serial.print("Gyro[mdps]:");
    Serial.print(gyroscope[0]-gyro_calibration[0]);
    Serial.print(", ");
    Serial.print(gyroscope[1]-gyro_calibration[1]);
    Serial.print(", ");
    Serial.println(gyroscope[2]-gyro_calibration[2]);
  }

}
