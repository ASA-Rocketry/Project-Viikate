#include <ISM330DHCXSensor.h> //IMU
#include <SPI.h> //spi
#include <BME280I2C.h> //barometer
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
//#include <SD.h> //for microSD card
//#include <SPI.h>
#include <LittleFS.h> //flash

File flightFile;

LittleFS_Program myFS; // Use Program Flash

//logging time for imu
unsigned long lastLogTime = 0;
const int logInterval = 100; // Log at 100Hz (every 10ms)

// imu pins
const int myMOSI = 11;  
const int myMISO = 12;   
const int mySCK  = 13;  
const int myCS   = 10;  

//imu initial offsets
uint32_t imuinitX = 0;
uint32_t imuinitY = 0;
uint32_t imuinitZ = 0;

// imu sensor object
ISM330DHCXSensor IMU(&SPI, myCS);

//magnetometer pins and values
SFE_MMC5983MA myMag;

int interruptPin = 17;

volatile bool newDataAvailable = true;
uint32_t rawValueX = 0;
uint32_t rawValueY = 0;
uint32_t rawValueZ = 0;
double scaledX = 0;
double scaledY = 0;
double scaledZ = 0;
double heading = 0;

//barometer object
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

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
  //SET DATA RATE TO 100HZ
  IMU.ACC_SetOutputDataRate(100);
  IMU.GYRO_SetOutputDataRate(100);
  //ENABLE
  IMU.ACC_Enable();
  IMU.GYRO_Enable();

  //overly complicated magnetometer setup
  Wire.begin();

  // Configure the interrupt pin for the "Measurement Done" interrupt
  pinMode(interruptPin, INPUT);

  if (myMag.begin() == false)
  {
      Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
      while (true)
          ;
  }
  myMag.softReset();
  myMag.setFilterBandwidth(100); //set filter bandwidth to 100Hz
  myMag.setContinuousModeFrequency(100); //set reading frquency to 100Hz
  myMag.enableAutomaticSetReset(); //set automatic set/reset
  myMag.enableContinuousMode(); //enable continuous mode
  // Set our interrupt flag, just in case we missed the rising edge
  newDataAvailable = true;

  bme.begin();

  int32_t accelerometer[3];
  IMU.ACC_GetAxes(accelerometer);
  //IMU.GYRO_GetAxes(gyroscope);
  //int32_t gyroscope[3];
  imuinitX = accelerometer[0];
  imuinitY = accelerometer[1];
  imuinitZ = accelerometer[2];

  Serial.println("HELLOOO");
  delay(1000);
}


void loop() {
  unsigned long currentTime = millis();

  //Check if it's time to take a sample
  if (currentTime - lastLogTime >= logInterval) {
    lastLogTime = currentTime;
    int32_t accelerometer[3];
    int32_t gyroscope[3];
    // Using the STMduino library methods
    IMU.ACC_GetAxes(accelerometer);
    IMU.GYRO_GetAxes(gyroscope);
    Serial.print("IMU data: ");
    Serial.print("Acc[mg]:");
    Serial.print(accelerometer[0]-imuinitX);
    Serial.print(", ");
    Serial.print(accelerometer[1]-imuinitY);
    Serial.print(", ");
    Serial.print(accelerometer[2]-imuinitZ);
    Serial.print(", Gyro[mdps]:");
    Serial.print(gyroscope[0]);
    Serial.print(", ");
    Serial.print(gyroscope[1]);
    Serial.print(", ");
    Serial.println(gyroscope[2]);
    //Serial.println(Acceleration);
    //Serial.println(AngularRate);
    myMag.readFieldsXYZ(&rawValueX, &rawValueY, &rawValueZ);

    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
    // Here we scale each field to +/- 1.0 to make it easier to calculate an approximate heading.
    //
    // Please note: to properly correct and calibrate the X, Y and Z channels, you need to determine true
    // offsets (zero points) and scale factors (gains) for all three channels. Futher details can be found at:
    // https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
    scaledX = (double)rawValueX - 131072.0;
    scaledX /= 131072.0;

    scaledY = (double)rawValueY - 131072.0;
    scaledY /= 131072.0;

    scaledZ = (double)rawValueZ - 131072.0;
    scaledZ /= 131072.0;

    // Magnetic north is oriented with the Y axis
    // Convert the X and Y fields into heading using atan2 (Arc Tangent 2)
    heading = atan2(scaledX, 0 - scaledY);

    // atan2 returns a value between +PI and -PI
    // Convert to degrees
    heading /= PI;
    heading *= 180;
    heading += 180;
    
    Serial.print("Magnetometer data:");
    Serial.print("Heading: ");
    Serial.println(heading, 1);
    //print barometer data
    Serial.print("Barometer data:");
    printBME280Data(&Serial);
  }
}

void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");
}
