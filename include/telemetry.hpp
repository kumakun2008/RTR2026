/**
 * @file telemetry.hpp
 * @brief Bluetooth SPP telemetry and command handler with wireless OTA utility.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

/**
 * @class Telemetry
 * @brief Handles Bluetooth SPP telemetry transmission, commands, and OTA updates.
 */
class Telemetry {
public:
    Telemetry();
    ~Telemetry();

    /**
     * @brief Initialize Bluetooth SPP interface.
     * @param deviceName Bluetooth broadcast name.
     * @return true on success, false otherwise.
     */
    bool begin(const char* deviceName = "RTR_Glider_Telemetry");

    /**
     * @brief Send a text log or structured telemetry data line via Bluetooth.
     * @param data String to transmit.
     */
    void sendText(const char* data);

    /**
     * @brief Process incoming serial characters from Bluetooth.
     * Checks for standard command patterns.
     */
    void process();

    /**
     * @brief Check if OTA mode is requested and active.
     */
    bool isOTAModeActive() const { return _otaActive; }

    /**
     * @brief Run the Arduino OTA handles. Called in main loop if OTA active.
     */
    void handleOTA();

    /**
     * @brief Compute the calibration zero-offset using IQR outlier removal.
     * Section 5: Buffers 3 seconds of data, computes IQR, averages normal readings.
     * @param samples Array of raw sensor values.
     * @param count Number of samples.
     * @return Computed zero-point offset (average of filtered samples).
     */
    static float calculateCalibZeroIQR(float* samples, size_t count);

    // Callbacks to notify main task
    void registerCalibCallback(void (*callback)()) { _calibCallback = callback; }
    void registerOTACallback(void (*callback)()) { _otaCallback = callback; }

private:
    BluetoothSerial _serialBT;
    bool _bluetoothActive;
    bool _otaActive;
    uint32_t _otaTimeoutStart;
    char _rxBuffer[64];
    int _rxIdx;

    // Callbacks
    void (*_calibCallback)();
    void (*_otaCallback)();

    /**
     * @brief Parse a fully received command string.
     */
    void parseCommand(const char* cmd);

    /**
     * @brief Start the OTA Wi-Fi AP and OTA Server.
     */
    void startOTAServer();
};

#endif // TELEMETRY_HPP
