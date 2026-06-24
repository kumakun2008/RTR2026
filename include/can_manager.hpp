/**
 * @file can_manager.hpp
 * @brief ESP32 native TWAI (CAN) bus driver manager.
 * Cross-platform compatible declaration.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include <Arduino.h>
#include "can_protocol.hpp"

#ifdef ESP32
#include "driver/twai.h"
#endif

/**
 * @class CANManager
 * @brief Manages the CAN communication layer. Wraps TWAI on ESP32, and stubs on non-ESP32 platforms.
 */
class CANManager {
public:
    CANManager();
    ~CANManager();

    /**
     * @brief Initialize and start the CAN driver.
     */
    bool begin(int txPin, int rxPin);

    /**
     * @brief Stop and uninstall the CAN driver.
     */
    void end();

    /**
     * @brief Transmit a raw CAN frame.
     */
    bool transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc);

    /**
     * @brief Receive a CAN frame.
     */
    bool receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs = 10);

    // --- Scaled Float Serialized Transmission Helper Methods ---
    bool transmitAccel(float ax, float ay, float az);
    bool transmitGyro(float gx, float gy, float gz);
    bool transmitMag(float mx, float my, float mz);
    bool transmitBaro(float pressPa, float tempC);
    bool transmitBattery(float busVolt, float currentAmp);
    bool transmitCalibZero();

private:
    bool _initialized;
};

#endif // CAN_MANAGER_HPP
