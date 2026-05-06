#include <ArduinoJson.h>

#include <cstddef>
#include <vector>

#include "constants.h"

#include <SD.h>

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

static bool parseFlightDataCsvLine(const String &line, FlightData &output) {
    String trimmed = line;
    trimmed.trim();

    if (trimmed.length() == 0 || trimmed.startsWith("#")) {
        return false;
    }

    float values[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int start = 0;

    for (int i = 0; i < 5; ++i) {
        int commaIndex = trimmed.indexOf(',', start);
        String token;

        if (commaIndex == -1) {
            if (i == 4) {
                token = trimmed.substring(start);
            } else {
                return false;
            }
        } else {
            token = trimmed.substring(start, commaIndex);
            start = commaIndex + 1;
        }

        token.trim();
        values[i] = token.toFloat();
    }

    output.timeMs = static_cast<unsigned long>(values[0] * 1000.0f + 0.5f);
    output.altitude = values[1];
    output.verticalVelocity = values[2];
    output.accZ = values[3];
    output.accelMagnitude = values[4];

    output.accX = 0.0f;
    output.accY = 0.0f;
    output.rotX = 0.0f;
    output.rotY = 0.0f;
    output.rotZ = 0.0f;
    output.oriX = 0.0f;
    output.oriY = 0.0f;
    output.oriZ = 0.0f;
    output.magX = 0.0;
    output.magY = 0.0;
    output.magZ = 0.0;
    output.heading = 0.0;
    output.rbfRemoved = false;

    return true;
}

static std::vector<FlightData> simulatedData;
void readSimulatedData() {
    // Implementation for reading simulated data
    pinMode(constants::kCsPin, OUTPUT);
    if (!SD.begin(constants::kCsPin)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    File file = SD.open("simulated_flight_data_1.csv");

    if (!file) {
        Serial.println("Failed to open CSV file: simulated_flight_data_1.csv");
        return;
    }

    while (file.available()) {
        String line = file.readStringUntil('\n');
        FlightData data;
        if (parseFlightDataCsvLine(line, data)) {
            simulatedData.push_back(data);
        }
    }
    file.close();
    Serial.println("Loaded " + String(simulatedData.size()) + " data points from CSV");
}

// This goes to the main loop when we're running in simulation mode. 
// It returns the next data point from the simulated data set on each call.
FlightData getSimulatedFlightData() {
    static size_t index = 0;

    if (index < simulatedData.size()) {
        FlightData data = simulatedData[index++];
        data.rbfRemoved = true; // Simulated launch prep: allow the state machine to transition.
        return data;
    } else {
        return simulatedData.empty() ? FlightData() : simulatedData.back();
    }
}
