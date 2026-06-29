/**
 * @file can_manager.cpp
 * @brief TWAI (CAN) bus driver manager implementation with cross-platform stubs.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "can_manager.hpp"

#ifdef ESP32

CANManager::CANManager() : _initialized(false) {}

CANManager::~CANManager() {
    end();
}

bool CANManager::begin(int txPin, int rxPin) {
    if (_initialized) return true;
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)txPin, 
        (gpio_num_t)rxPin, 
        TWAI_MODE_NORMAL
    );
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    
    _initialized = true;
    return true;
}

void CANManager::end() {
    if (_initialized) {
        twai_stop();
        twai_driver_uninstall();
        _initialized = false;
    }
}

bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) {
    if (!_initialized) return false;
    
    twai_message_t msg;
    msg.identifier = id;
    msg.extd = 0; 
    msg.rtr = 0;  
    msg.data_length_code = dlc > 8 ? 8 : dlc;
    
    if (dlc > 0 && data != NULL) {
        for (int i = 0; i < msg.data_length_code; i++) {
            msg.data[i] = data[i];
        }
    }
    
    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) {
    if (!_initialized) return false;
    
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK) {
        id = msg.identifier;
        dlc = msg.data_length_code;
        for (int i = 0; i < dlc; i++) {
            data[i] = msg.data[i];
        }
        return true;
    }
    return false;
}

// --- Telemetry Transmissions (ESP32) ---

bool CANManager::transmitAttitude(float pitch, float roll) {
    uint8_t data[8];
    memcpy(data, &pitch, 4);
    memcpy(data + 4, &roll, 4);
    return transmitRaw(CAN_ID_ATTITUDE, data, 8);
}

bool CANManager::transmitAirspeed(float pressPa, float airspeed) {
    uint8_t data[8];
    memcpy(data, &pressPa, 4);
    memcpy(data + 4, &airspeed, 4);
    return transmitRaw(CAN_ID_AIRSPEED, data, 8);
}

bool CANManager::transmitRudderAngle(float angle) {
    uint8_t data[4];
    memcpy(data, &angle, 4);
    return transmitRaw(CAN_ID_RUDDER_ANGLE, data, 4);
}

bool CANManager::transmitAltitude(float staticPressOrLidar, float ultrasonic, bool isAltimeterNode) {
    uint8_t data[8];
    if (isAltimeterNode) {
        memcpy(data, &staticPressOrLidar, 4);
        memcpy(data + 4, &ultrasonic, 4);
    } else {
        memcpy(data, &staticPressOrLidar, 4);
        float zero = 0.0f;
        memcpy(data + 4, &zero, 4);
    }
    return transmitRaw(CAN_ID_ALTITUDE, data, 8);
}

bool CANManager::transmitGPSPos(double lat, double lon) {
    int32_t latVal = (int32_t)(lat * CAN_Scale::GPS_DEG);
    int32_t lonVal = (int32_t)(lon * CAN_Scale::GPS_DEG);
    uint8_t data[8];
    memcpy(data, &latVal, 4);
    memcpy(data + 4, &lonVal, 4);
    return transmitRaw(CAN_ID_GPS_POS, data, 8);
}

bool CANManager::transmitAoaAos(float press1, float press2) {
    uint8_t data[8];
    memcpy(data, &press1, 4);
    memcpy(data + 4, &press2, 4);
    return transmitRaw(CAN_ID_AOA_AOS, data, 8);
}

bool CANManager::transmitBattery(float busVolt) {
    uint8_t data[4];
    memcpy(data, &busVolt, 4);
    return transmitRaw(CAN_ID_BATTERY_VOLT, data, 4);
}

bool CANManager::transmitVoiceCmd(uint8_t alertCode) {
    return transmitRaw(CAN_ID_VOICE_CMD, &alertCode, 1);
}

bool CANManager::transmitCalibZero() {
    return transmitRaw(CAN_ID_CALIB_ZERO, NULL, 0);
}

bool CANManager::transmitOtaStart() {
    return transmitRaw(CAN_ID_OTA_START, NULL, 0);
}

#else // Non-ESP32 Mock implementations (STM32 stubs)

CANManager::CANManager() : _initialized(false) {}
CANManager::~CANManager() {}
bool CANManager::begin(int txPin, int rxPin) { _initialized = true; return true; }
void CANManager::end() { _initialized = false; }
bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) { return true; }
bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) { return false; }
bool CANManager::transmitAttitude(float pitch, float roll) { return true; }
bool CANManager::transmitAirspeed(float pressPa, float airspeed) { return true; }
bool CANManager::transmitRudderAngle(float angle) { return true; }
bool CANManager::transmitAltitude(float staticPressOrLidar, float ultrasonic, bool isAltimeterNode) { return true; }
bool CANManager::transmitGPSPos(double lat, double lon) { return true; }
bool CANManager::transmitAoaAos(float press1, float press2) { return true; }
bool CANManager::transmitBattery(float busVolt) { return true; }
bool CANManager::transmitVoiceCmd(uint8_t alertCode) { return true; }
bool CANManager::transmitCalibZero() { return true; }
bool CANManager::transmitOtaStart() { return true; }

#endif // ESP32
