#include "Arduino.h"
#include "constants.h"
#include "control.h"
#include "control_hardware.h"
#include "data_logger.h"
#include "remote_config.h"
#include "sensors.h"
#include "state_machine.h"

DataLogger data_logger;
StateMachine state_machine(data_logger);
Sensors sensors(data_logger);
ControlHardware control_hardware;
Control control;

float error;

struct TelemetryData {
    float altitude;
    float verticalVelocity;
    float accZ;
    float rotZ;
    float error;
    uint32_t timestamp;
} __attribute__((packed));

void sendToSerial(Print &serial, FlightData data, Control control) {
    TelemetryData telem;
    telem.altitude = data.altitude;
    telem.verticalVelocity = data.verticalVelocity;
    telem.accZ = data.accZ;
    telem.rotZ = data.oriZ;
    telem.error = control.get_error();
    telem.timestamp = micros();

    serial.write((uint8_t *)&telem, sizeof(telem));
}

#define RESET_BUTTON 30

void setup() {
    Serial.begin(9600);
    Serial8.begin(9600);

    data_logger
        .Initialize();  // Initialize this before anything else to log failures
    sensors.Initialize();
    //control_hardware.Initialize();
    control.Initialize();
    data_logger.LogEvent(LogType::kInfo, "SETUP COMPLETE");

    pinMode(constants::kLEDPin, OUTPUT);  // Set LED pin as output
    //
    pinMode(RESET_BUTTON, INPUT_PULLUP);
}

void softwareReset() {
    // Trigger a system reset using the ARM System Control Block
    SCB_AIRCR = 0x05FA0004;
    while (1) {
        // wait for reset to occur
    }
}

/**
 * @brief Arduino main loop.
 *
 * Reads sensor data, logs flight telemetry, and performs control.
 */
void loop() {
    if (digitalRead(RESET_BUTTON) == LOW) {
        delay(20);  // debounce
        if (digitalRead(RESET_BUTTON) == LOW) {
            softwareReset();
        }
    }

    FlightData data = sensors.ReadFlightData();
    data_logger.LogFlightData(data);

    control.PID(
        90.0f,
        -data.oriZ
    );  // Example: control to maintain 90 degrees orientation around Z-axis
    char serializedFlightData[512] = "";
    data.SerializeJson(serializedFlightData, sizeof(serializedFlightData));

    size_t jsonLen =
        strnlen(serializedFlightData, sizeof(serializedFlightData));

    // Rust expects PACKET_MAGIC as little-endian bytes for:
    // 0x68d7ede87b264e66b6091a22a3325b10
    static const uint8_t magic[16] = {
        0x10,
        0x5b,
        0x32,
        0xa3,
        0x22,
        0x1a,
        0x09,
        0xb6,
        0x66,
        0x4e,
        0x26,
        0x7b,
        0xe8,
        0xed,
        0xd7,
        0x68
    };

    uint32_t len = (uint32_t)jsonLen;
    uint8_t lenBytes[4] = {
        (uint8_t)(len & 0xff),
        (uint8_t)((len >> 8) & 0xff),
        (uint8_t)((len >> 16) & 0xff),
        (uint8_t)((len >> 24) & 0xff)
    };

    Serial.write(magic, sizeof(magic));
    Serial.write(lenBytes, sizeof(lenBytes));
    Serial.write((const uint8_t *)serializedFlightData, jsonLen);

    readAndApplyPacket(Serial);

    // Print values to Serial
    //Serial.print("Altitude: ");
    //Serial.println(data.altitude);
    //Serial.print("Vertical Velocity: ");
    //Serial.println(data.verticalVelocity);
    //Serial.print("AccelZ: ");
    //Serial.println(data.accZ);
    //Serial.print("RotZ: ");
    //Serial.println(data.rotZ);
    //Serial.print("AccelMagnitude: ");
    //Serial.println(data.accelMagnitude);
    //Serial.print("RBF Removed: ");
    //Serial.println(data.rbfRemoved);
    //Serial.print("Acc (x, y, z): ");
    //Serial.print(data.accX);
    //Serial.print(", ");
    //Serial.print(data.accY);
    //Serial.print(", ");
    //Serial.println(data.accZ);
    //Serial.print("Gyro Angular Rate (x, y, z): ");
    //Serial.print(data.rotX);
    //Serial.print(", ");
    //Serial.print(data.rotY);
    //Serial.print(", ");
    //Serial.println(data.rotZ);
    //Serial.print("Orientation (x, y, z): ");
    //Serial.print(data.oriX);
    //Serial.print(", ");
    //Serial.print(data.oriY);
    //Serial.print(", ");
    //Serial.println(data.oriZ);
    //Serial.print("Mag: ");
    //Serial.print(data.magX);
    //Serial.print(", ");
    //Serial.print(data.magY);
    //Serial.print(", ");
    //Serial.println(data.magZ), Serial.print("Heading: ");
    //Serial.println(data.heading);
    //Serial.println("--------------------");

    //Serial.println("Control error:");
    //error = control.get_error();
    //Serial.println(error);
    if (abs(error) < 5.0f) {  // Example threshold for critical error
        digitalWrite(
            constants::kLEDPin,
            HIGH
        );  // Turn on LED if error is small (indicating good control)
    } else {
        digitalWrite(constants::kLEDPin, LOW);  // Turn off LED if error is
        delay(50);
    }

    //sendToSerial(Serial8, data, control);
}
