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
    
    twai_message_t msg = {};
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
    
    twai_message_t msg = {};
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

bool CANManager::transmitScaled(uint32_t id, float value, float scale) {
    int32_t val = (int32_t)(value * scale);
    uint8_t data[4];
    memcpy(data, &val, 4);
    return transmitRaw(id, data, 4);
}

bool CANManager::transmitInt32(uint32_t id, int32_t value) {
    uint8_t data[4];
    memcpy(data, &value, 4);
    return transmitRaw(id, data, 4);
}

bool CANManager::transmitDoubleSplit(uint32_t idUpper, uint32_t idLower, double value) {
    uint64_t binVal;
    memcpy(&binVal, &value, 8);
    uint32_t upper = (uint32_t)(binVal >> 32);
    uint32_t lower = (uint32_t)(binVal & 0xFFFFFFFF);
    
    uint8_t dataUpper[4];
    uint8_t dataLower[4];
    memcpy(dataUpper, &upper, 4);
    memcpy(dataLower, &lower, 4);
    
    bool okUpper = transmitRaw(idUpper, dataUpper, 4);
    bool okLower = transmitRaw(idLower, dataLower, 4);
    return okUpper && okLower;
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

bool CANManager::transmitScaled(uint32_t id, float value, float scale) { return true; }
bool CANManager::transmitInt32(uint32_t id, int32_t value) { return true; }
bool CANManager::transmitDoubleSplit(uint32_t idUpper, uint32_t idLower, double value) { return true; }
bool CANManager::transmitVoiceCmd(uint8_t alertCode) { return true; }
bool CANManager::transmitCalibZero() { return true; }
bool CANManager::transmitOtaStart() { return true; }

#endif // ESP32
