/**
 * @file telemetry.hpp
 * @brief Bluetooth SPP telemetry and command handler with wireless OTA utility.
 * Cross-platform compatible declaration.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <Arduino.h>

#ifdef ESP32
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

/**
 * @class Telemetry
 * @brief Handles Bluetooth SPP telemetry transmission, commands, and OTA updates on ESP32.
 */
class Telemetry {
public:
    Telemetry();
    ~Telemetry();

    bool begin(const char* deviceName = "RTR_Glider_Telemetry");
    void sendText(const char* data);
    void process();
    bool isOTAModeActive() const { return _otaActive; }
    void handleOTA();
    static float calculateCalibZeroIQR(float* samples, size_t count);

    void registerCalibCallback(void (*callback)());
    void registerOTACallback(void (*callback)());

private:
    BluetoothSerial _serialBT;
    bool _bluetoothActive;
    bool _otaActive;
    uint32_t _otaTimeoutStart;
    char _rxBuffer[64];
    int _rxIdx;

    void (*_calibCallback)();
    void (*_otaCallback)();

    void parseCommand(const char* cmd);
    void startOTAServer();
};

#else // Mock class for non-ESP32 platforms (like STM32)

class Telemetry {
public:
    Telemetry() {}
    ~Telemetry() {}

    bool begin(const char* deviceName = "") { return false; }
    void sendText(const char* data) {}
    void process() {}
    bool isOTAModeActive() const { return false; }
    void handleOTA() {}
    static float calculateCalibZeroIQR(float* samples, size_t count) {
        // Simple sorting and averaging for stubs (basic compatibility)
        if (count == 0) return 0.0f;
        float sum = 0.0f;
        for(size_t i=0; i<count; ++i) sum += samples[i];
        return sum / count;
    }

    void registerCalibCallback(void (*callback)()) {}
    void registerOTACallback(void (*callback)()) {}
};

#endif // ESP32

#endif // TELEMETRY_HPP
