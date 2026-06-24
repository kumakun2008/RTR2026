/**
 * @file can_manager.hpp
 * @brief ESP32 native TWAI (CAN) bus driver manager.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include <Arduino.h>
#include "driver/twai.h"
#include "can_protocol.hpp"

/**
 * @class CANManager
 * @brief Manages the ESP32 native TWAI peripheral for CAN communication at 1 Mbps.
 */
class CANManager {
public:
    CANManager();
    ~CANManager();

    /**
     * @brief Initialize and start the TWAI driver.
     * @param txPin TWAI TX GPIO pin.
     * @param rxPin TWAI RX GPIO pin.
     * @return true on success, false otherwise.
     */
    bool begin(int txPin, int rxPin);

    /**
     * @brief Stop and uninstall the TWAI driver.
     */
    void end();

    /**
     * @brief Transmit a raw CAN frame.
     * @param id Standard CAN message ID (11-bit).
     * @param data Pointer to data bytes (max 8).
     * @param dlc Data Length Code (0-8).
     * @return true if transmitted successfully, false on error or timeout.
     */
    bool transmitRaw(uint32_t id, const uint8_t* data, uint8_t dlc);

    /**
     * @brief Receive a CAN frame.
     * @param id Output message ID.
     * @param data Output data buffer.
     * @param dlc Output Data Length Code.
     * @param timeoutMs Maximum block time in milliseconds.
     * @return true if a message was received, false on timeout or error.
     */
    bool receiveRaw(uint32_t& id, uint8_t* data, uint8_t& dlc, uint32_t timeoutMs = 10);

    // --- Scaled Float Serialized Transmission Helper Methods (Section 7-1) ---

    /**
     * @brief Transmit 3-axis accelerometer data (milli-g).
     * ID: CAN_MSG_MAIN_IMU_ACCEL
     */
    bool transmitAccel(float ax, float ay, float az);

    /**
     * @brief Transmit 3-axis gyroscope data (centi-dps).
     * ID: CAN_MSG_MAIN_IMU_GYRO
     */
    bool transmitGyro(float gx, float gy, float gz);

    /**
     * @brief Transmit 3-axis magnetometer data (deci-uT).
     * ID: CAN_MSG_MAIN_MAG
     */
    bool transmitMag(float mx, float my, float mz);

    /**
     * @brief Transmit barometric pressure and temperature (centi-Pascal / centi-Celsius).
     * ID: CAN_MSG_MAIN_ENV
     */
    bool transmitBaro(float pressPa, float tempC);

    /**
     * @brief Transmit battery parameters (mV / mA).
     * ID: CAN_MSG_MAIN_BATTERY
     */
    bool transmitBattery(float busVolt, float currentAmp);

    /**
     * @brief Transmit calibration zero command.
     */
    bool transmitCalibZero();

private:
    bool _initialized; ///< Track driver setup state
};

#endif // CAN_MANAGER_HPP
