#include <ArduinoJson.h>
#include <cstddef>
#include <vector>
#include <SD.h>
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

static bool parseFlightDataCsvLine(const String &line, FlightData &output) {
    // Strip whitespace and ignore blank lines or commented lines.
    String trimmed = line;
    trimmed.trim();

    if (trimmed.length() == 0 || trimmed.startsWith("#")) {
        return false;
    }

    // Parse the five comma-separated numeric columns used by the state machine.
    float values[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int start = 0;

    for (int i = 0; i < 5; ++i) {
        int commaIndex = trimmed.indexOf(',', start);
        String token;

        if (commaIndex == -1) {
            // Last token should consume the remainder of the line.
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

    // Convert CSV columns into FlightData fields.
    output.timeMs = static_cast<unsigned long>(values[0] * 1000.0f + 0.5f);
    output.altitude = values[1];
    output.verticalVelocity = values[2];
    output.accZ = values[3];
    output.accelMagnitude = values[4];

    // Populate unused fields with defaults so the struct remains valid.
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
static size_t simulated_index = 0;
static bool simulated_end_printed = false;

void readSimulatedData() {
    // Reset the simulation buffer and playback state before every load.
    simulatedData.clear();
    simulated_index = 0;
    simulated_end_printed = false;

    // Initialize the Teensy SD library for built-in SD card access.
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialization successful!");

    File file = SD.open("simulated_flight_data_1.csv");
    if (!file) {
        Serial.println("Failed to open CSV file: simulated_flight_data_1.csv");
        return;
    }
    Serial.println("CSV file opened successfully: simulated_flight_data_1.csv");

    // Read the CSV file line-by-line and store only valid flight data rows.
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

// Return the next simulated row on each call; preserve final state once the
// dataset is exhausted and print "End of data!" exactly once.
FlightData getSimulatedFlightData() {
    if (simulated_index < simulatedData.size()) {
        FlightData data = simulatedData[simulated_index++];
        return data;
    }

    if (simulatedData.empty()) {
        return FlightData();
    }

    if (!simulated_end_printed) {
        Serial.println("End of data!");
        simulated_end_printed = true;
    }

    FlightData data = simulatedData.back();
    data.rbfRemoved = true; // Keep simulated launch prep after EOF.
    return data;
}
