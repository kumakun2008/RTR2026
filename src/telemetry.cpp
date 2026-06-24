/**
 * @file telemetry.cpp
 * @brief Bluetooth SPP telemetry and command handler implementation.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "telemetry.hpp"
#include <algorithm>

Telemetry::Telemetry()
    : _bluetoothActive(false), _otaActive(false), _otaTimeoutStart(0), _rxIdx(0),
      _calibCallback(NULL), _otaCallback(NULL) {
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
}

Telemetry::~Telemetry() {
    if (_bluetoothActive) {
        _serialBT.end();
    }
}

bool Telemetry::begin(const char* deviceName) {
    if (_serialBT.begin(deviceName)) {
        _bluetoothActive = true;
        return true;
    }
    return false;
}

void Telemetry::sendText(const char* data) {
    if (_bluetoothActive) {
        _serialBT.println(data);
    }
}

void Telemetry::process() {
    if (!_bluetoothActive || _otaActive) return;
    
    while (_serialBT.available() > 0) {
        char c = _serialBT.read();
        if (c == '\n' || c == '\r') {
            if (_rxIdx > 0) {
                _rxBuffer[_rxIdx] = '\0';
                parseCommand(_rxBuffer);
                _rxIdx = 0;
            }
        } else {
            if (_rxIdx < (int)sizeof(_rxBuffer) - 1) {
                _rxBuffer[_rxIdx++] = c;
            } else {
                _rxIdx = 0; // Buffer overflow, reset
            }
        }
    }
}

void Telemetry::parseCommand(const char* cmd) {
    if (strcmp(cmd, ">CMD:CALIB_ZERO") == 0) {
        sendText("ACK:CALIB_ZERO_START");
        if (_calibCallback != NULL) {
            _calibCallback();
        }
    } else if (strcmp(cmd, ">CMD:OTA_START") == 0) {
        sendText("ACK:OTA_MODE_ACTIVE");
        if (_otaCallback != NULL) {
            _otaCallback();
        }
        startOTAServer();
    } else {
        char response[80];
        snprintf(response, sizeof(response), "ERR:UNKNOWN_COMMAND: %s", cmd);
        sendText(response);
    }
}

void Telemetry::startOTAServer() {
    // 1. Terminate Bluetooth SPP to free radio peripheral resources
    if (_bluetoothActive) {
        _serialBT.end();
        _bluetoothActive = false;
    }
    
    // 2. Setup WiFi in Access Point mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP("RTR_Glider_OTA_AP", "rtr2026glider");
    
    // 3. Setup OTA endpoints
    ArduinoOTA.onStart([]() {
        // Shutdown critical subsystems if needed
    });
    
    ArduinoOTA.onEnd([]() {
        ESP.restart(); // Reboot on successful flash
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        ESP.restart(); // Reboot on flash failure to return to normal firmware
    });
    
    ArduinoOTA.begin();
    
    _otaActive = true;
    _otaTimeoutStart = millis();
}

void Telemetry::handleOTA() {
    if (!_otaActive) return;
    
    ArduinoOTA.handle();
    
    // Auto-timeout after 5 minutes of inactivity (no update started)
    if (millis() - _otaTimeoutStart > 300000) {
        ESP.restart(); // Reboot back to normal telemetry mode
    }
}

float Telemetry::calculateCalibZeroIQR(float* samples, size_t count) {
    if (count == 0) return 0.0f;
    if (count == 1) return samples[0];
    
    // 1. Sort samples in ascending order
    std::sort(samples, samples + count);
    
    // 2. Compute Q1 (25th percentile) and Q3 (75th percentile)
    size_t q1Idx = count / 4;
    size_t q3Idx = (3 * count) / 4;
    float q1 = samples[q1Idx];
    float q3 = samples[q3Idx];
    
    // 3. Calculate Interquartile Range (IQR)
    float iqr = q3 - q1;
    
    // 4. Determine boundaries for normal distribution range (IQR factor = 1.5)
    float lowerBound = q1 - 1.5f * iqr;
    float upperBound = q3 + 1.5f * iqr;
    
    // 5. Average values within the normal boundaries (excluding outliers)
    double sum = 0.0;
    size_t validCount = 0;
    
    for (size_t i = 0; i < count; ++i) {
        if (samples[i] >= lowerBound && samples[i] <= upperBound) {
            sum += samples[i];
            validCount++;
        }
    }
    
    if (validCount == 0) return 0.0f;
    return (float)(sum / validCount);
}
