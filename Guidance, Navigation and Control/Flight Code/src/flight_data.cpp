#include <ArduinoJson.h>

#include <cstddef>

#include "constants.h"

void FlightData::SerializeJson(
    char *outputBuffer,
    std::size_t outputBufferSize
) {
    JsonDocument doc;
    doc["timeMs"] = this->timeMs;
    doc["altitude"] = this->altitude;
    doc["verticalVelocity"] = this->verticalVelocity;
    doc["accX"] = this->accX;
    doc["accY"] = this->accY;
    doc["accZ"] = this->accZ;
    doc["rotX"] = this->rotX;
    doc["rotY"] = this->rotY;
    doc["rotZ"] = this->rotZ;
    doc["oriX"] = this->oriX;
    doc["oriY"] = this->oriY;
    doc["oriZ"] = this->oriZ;
    doc["magX"] = this->magX;
    doc["magY"] = this->magY;
    doc["magZ"] = this->magZ;
    doc["heading"] = this->heading;
    doc["accelMagnitude"] = this->accelMagnitude;
    doc["rbfRemoved"] = this->rbfRemoved;

    doc["kZProportional"] = constants::kZProportional;
    doc["kZIntegrator"] = constants::kZIntegrator;
    doc["kZDerivative"] = constants::kZDerivative;
    doc["kServoTrim1"] = constants::kServoTrim1;
    doc["kServoTrim2"] = constants::kServoTrim2;
    doc["kServoTrim3"] = constants::kServoTrim3;
    doc["kServoTrim4"] = constants::kServoTrim4;

    serializeJson(doc, outputBuffer, outputBufferSize);
}
