#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "constants.h"

// Packet framing (16-byte magic + 4-byte length)
static const size_t HEADER_SIZE = 16 + 4;
static const size_t MAX_PACKET_SIZE = 64 * 1024;

// Magic value (u128 = 0x68d7ede87b264e66b6091a22a3325b10), little-endian
static const uint8_t MAGIC_LE[16] = {
    0x10,
    0x5B,
    0x32,
    0xA3,
    0x22,
    0x1A,
    0x09,
    0xB6,
    0x66,
    0x4E,
    0x26,
    0x7B,
    0xE8,
    0xED,
    0xD7,
    0x68
};

static bool bytesEqual16(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 16; ++i) {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

static bool readExactly(Stream &s, uint8_t *dst, size_t n, uint32_t timeoutMs) {
    const uint32_t deadline = millis() + timeoutMs;
    size_t got = 0;
    while (got < n) {
        if (s.available()) {
            int c = s.read();
            if (c < 0)
                continue;
            dst[got++] = (uint8_t)c;
        } else {
            if ((int32_t)(deadline - millis()) <= 0)
                return false;
            yield();
        }
    }
    return true;
}

static bool readU32LenLE(Stream &s, uint32_t &len, uint32_t timeoutMs) {
    uint8_t b[4];
    if (!readExactly(s, b, 4, timeoutMs))
        return false;

    uint32_t v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16)
        | ((uint32_t)b[3] << 24);

    if (v == 0 || v > (uint32_t)MAX_PACKET_SIZE)
        return false;
    len = v;
    return true;
}

static bool findMagicLE(Stream &s, uint32_t timeoutMs) {
    uint8_t win[16];
    size_t fill = 0;
    const uint32_t deadline = millis() + timeoutMs;

    while (true) {
        if ((int32_t)(deadline - millis()) <= 0)
            return false;

        if (!s.available()) {
            yield();
            continue;
        }

        int c = s.read();
        if (c < 0)
            continue;

        if (fill < 16) {
            win[fill++] = (uint8_t)c;
            if (fill < 16)
                continue;
        } else {
            memmove(win, win + 1, 15);
            win[15] = (uint8_t)c;
        }

        if (bytesEqual16(win, MAGIC_LE))
            return true;
    }
}

// Apply JSON keys if present. Keys are optional and update on presence.
static void applyJsonToGlobals(const JsonVariantConst &root) {
    if (root.containsKey("kProportional")) {
        constants::kProportional = root["kProportional"].as<float>();
    }
    if (root.containsKey("kIntegrator")) {
        constants::kIntegrator = root["kIntegrator"].as<float>();
    }
    if (root.containsKey("kDerivative")) {
        constants::kDerivative = root["kDerivative"].as<float>();
    }
    if (root.containsKey("kServoTrim1")) {
        constants::kServoTrim1 = root["kServoTrim1"].as<float>();
    }
    if (root.containsKey("kServoTrim2")) {
        constants::kServoTrim2 = root["kServoTrim2"].as<float>();
    }
    if (root.containsKey("kServoTrim3")) {
        constants::kServoTrim3 = root["kServoTrim3"].as<float>();
    }
    if (root.containsKey("kServoTrim4")) {
        constants::kServoTrim4 = root["kServoTrim4"].as<float>();
    }
}

// Reads one packet from 'serial', parses JSON body, and updates globals.
// Returns true on success (packet fully parsed and applied).
bool readAndApplyPacket(Stream &serial, uint32_t timeoutMs = 100) {
    if (!findMagicLE(serial, timeoutMs))
        return false;

    uint32_t bodyLen = 0;
    if (!readU32LenLE(serial, bodyLen, timeoutMs))
        return false;

    // Read JSON body
    uint8_t *buf = (uint8_t *)malloc(bodyLen);
    if (!buf)
        return false;

    bool ok = readExactly(serial, buf, bodyLen, timeoutMs);
    if (!ok) {
        free(buf);
        return false;
    }

    // Parse JSON
    DynamicJsonDocument doc(bodyLen + 512);
    DeserializationError err = deserializeJson(doc, buf, bodyLen);
    free(buf);
    if (err)
        return false;

    JsonVariantConst root = doc.as<JsonVariantConst>();
    if (!root.is<JsonObjectConst>() && !root.is<JsonVariantConst>()) {
        return false;
    }

    applyJsonToGlobals(root);
    return true;
}

/*
Example usage:

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (readAndApplyPacket(Serial, 200)) {
    // Values updated
  }
}
*/
