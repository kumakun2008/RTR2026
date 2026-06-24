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

// --- Serialized Float Transmissions ---

bool CANManager::transmitAccel(float ax, float ay, float az) {
    int16_t rawX = (int16_t)(ax * CAN_Scale::ACCEL);
    int16_t rawY = (int16_t)(ay * CAN_Scale::ACCEL);
    int16_t rawZ = (int16_t)(az * CAN_Scale::ACCEL);
    
    uint8_t data[6];
    data[0] = (uint8_t)(rawX >> 8);
    data[1] = (uint8_t)(rawX & 0xFF);
    data[2] = (uint8_t)(rawY >> 8);
    data[3] = (uint8_t)(rawY & 0xFF);
    data[4] = (uint8_t)(rawZ >> 8);
    data[5] = (uint8_t)(rawZ & 0xFF);
    
    return transmitRaw(CAN_MSG_MAIN_IMU_ACCEL, data, 6);
}

bool CANManager::transmitGyro(float gx, float gy, float gz) {
    int16_t rawX = (int16_t)(gx * CAN_Scale::GYRO);
    int16_t rawY = (int16_t)(gy * CAN_Scale::GYRO);
    int16_t rawZ = (int16_t)(gz * CAN_Scale::GYRO);
    
    uint8_t data[6];
    data[0] = (uint8_t)(rawX >> 8);
    data[1] = (uint8_t)(rawX & 0xFF);
    data[2] = (uint8_t)(rawY >> 8);
    data[3] = (uint8_t)(rawY & 0xFF);
    data[4] = (uint8_t)(rawZ >> 8);
    data[5] = (uint8_t)(rawZ & 0xFF);
    
    return transmitRaw(CAN_MSG_MAIN_IMU_GYRO, data, 6);
}

bool CANManager::transmitMag(float mx, float my, float mz) {
    int16_t rawX = (int16_t)(mx * CAN_Scale::MAG);
    int16_t rawY = (int16_t)(my * CAN_Scale::MAG);
    int16_t rawZ = (int16_t)(mz * CAN_Scale::MAG);
    
    uint8_t data[6];
    data[0] = (uint8_t)(rawX >> 8);
    data[1] = (uint8_t)(rawX & 0xFF);
    data[2] = (uint8_t)(rawY >> 8);
    data[3] = (uint8_t)(rawY & 0xFF);
    data[4] = (uint8_t)(rawZ >> 8);
    data[5] = (uint8_t)(rawZ & 0xFF);
    
    return transmitRaw(CAN_MSG_MAIN_MAG, data, 6);
}

bool CANManager::transmitBaro(float pressPa, float tempC) {
    int32_t rawPress = (int32_t)(pressPa * CAN_Scale::PRESS);
    int16_t rawTemp  = (int16_t)(tempC * CAN_Scale::TEMP);
    
    uint8_t data[6];
    data[0] = (uint8_t)(rawPress >> 24);
    data[1] = (uint8_t)(rawPress >> 16);
    data[2] = (uint8_t)(rawPress >> 8);
    data[3] = (uint8_t)(rawPress & 0xFF);
    data[4] = (uint8_t)(rawTemp >> 8);
    data[5] = (uint8_t)(rawTemp & 0xFF);
    
    return transmitRaw(CAN_MSG_MAIN_ENV, data, 6);
}

bool CANManager::transmitBattery(float busVolt, float currentAmp) {
    uint16_t rawVolt = (uint16_t)(busVolt * CAN_Scale::VOLT);
    int16_t rawCurr  = (int16_t)(currentAmp * CAN_Scale::CURR);
    
    uint8_t data[4];
    data[0] = (uint8_t)(rawVolt >> 8);
    data[1] = (uint8_t)(rawVolt & 0xFF);
    data[2] = (uint8_t)(rawCurr >> 8);
    data[3] = (uint8_t)(rawCurr & 0xFF);
    
    return transmitRaw(CAN_MSG_MAIN_BATTERY, data, 4);
}

bool CANManager::transmitCalibZero() {
    return transmitRaw(CAN_MSG_CMD_CALIB_ZERO, NULL, 0);
}

#else // Non-ESP32 Mock implementations (STM32 stubs)

CANManager::CANManager() : _initialized(false) {}
CANManager::~CANManager() {}
bool CANManager::begin(int txPin, int rxPin) { _initialized = true; return true; }
void CANManager::end() { _initialized = false; }
bool CANManager::transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc) { return true; }
bool CANManager::receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs) { return false; }
bool CANManager::transmitAccel(float ax, float ay, float az) { return true; }
bool CANManager::transmitGyro(float gx, float gy, float gz) { return true; }
bool CANManager::transmitMag(float mx, float my, float mz) { return true; }
bool CANManager::transmitBaro(float pressPa, float tempC) { return true; }
bool CANManager::transmitBattery(float busVolt, float currentAmp) { return true; }
bool CANManager::transmitCalibZero() { return true; }

#endif // ESP32
