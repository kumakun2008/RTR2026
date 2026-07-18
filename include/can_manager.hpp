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

#ifdef ARDUINO_ARCH_STM32
#include <stm32f3xx_hal.h>
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
     * @param txPin  GPIO pin connected to transceiver TXD
     * @param rxPin  GPIO pin connected to transceiver RXD
     * @param stbPin GPIO pin connected to transceiver STB (MCP2561 / BD41041).
     *               Pass -1 (default) if STB is hard-wired to GND on the PCB.
     *               When provided, the pin is driven LOW to enable Normal mode.
     */
    bool begin(int txPin, int rxPin, int stbPin = -1);

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

    // --- Telemetry Transmissions (Aligned with new CAN ID specification) ---
    bool transmitScaled(uint32_t id, float value, float scale);
    bool transmitInt32(uint32_t id, int32_t value);
    bool transmitDoubleSplit(uint32_t idUpper, uint32_t idLower, double value);
    
    // Legacy mapping helpers / Commands
    bool transmitVoiceCmd(uint8_t alertCode);
    bool transmitCalibZero();
    bool transmitOtaStart();
    void printStatus();
    bool isInitialized() const { return _initialized; }

private:
    bool _initialized;
    int  _stbPin = -1; ///< GPIO for MCP2561/BD41041 STB pin (-1 = not used)
    uint32_t _lastRecoveryAttemptMs = 0;
    void handleAutoRecovery();
};

#endif // CAN_MANAGER_HPP
