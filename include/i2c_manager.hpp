/**
 * @file i2c_manager.hpp
 * @brief Thread-safe I2C bus wrapper with automated hardware bus recovery.
 * Compatible with ESP32 and non-ESP32 (STM32) architectures.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef I2C_MANAGER_HPP
#define I2C_MANAGER_HPP

#include <Arduino.h>
#include <Wire.h>

#ifdef ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#else
// Type stub for non-FreeRTOS (STM32/other) environments
typedef void* SemaphoreHandle_t;
#define TickType_t uint32_t
#define pdMS_TO_TICKS(ms) (ms)
#endif

/**
 * @class I2CManager
 * @brief Thread-safe I2C wrapper that guards transactions with mutexes and offers recovery procedures.
 */
class I2CManager {
public:
    /**
     * @brief Construct a new I2CManager instance.
     * @param wire Reference to the Arduino TwoWire instance.
     * @param sda Pin number for I2C Serial Data line.
     * @param scl Pin number for I2C Serial Clock line.
     * @param speed Bus frequency in Hz (default 400kHz).
     */
    I2CManager(TwoWire& wire, int sda, int scl, uint32_t speed = 400000);

    /**
     * @brief Destroy the I2CManager.
     */
    ~I2CManager();

    /**
     * @brief Initialize the I2C peripheral and setup the mutex.
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin();

    /**
     * @brief Write a single byte to a register.
     * @param devAddr Device address on the I2C bus.
     * @param regAddr Register address.
     * @param data Byte to write.
     * @return true if successful, false otherwise.
     */
    bool writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data);

    /**
     * @brief Read a block of bytes from a register.
     * @param devAddr Device address on the I2C bus.
     * @param regAddr Start register address.
     * @param data Buffer to store read bytes.
     * @param size Number of bytes to read.
     * @return true if successful, false otherwise.
     */
    bool readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t* data, size_t size);

    /**
     * @brief Write raw bytes directly to the device (without register address).
     * @param devAddr Device address on the I2C bus.
     * @param data Buffer containing data to write.
     * @param size Number of bytes to write.
     * @return true if successful, false otherwise.
     */
    bool writeRaw(uint8_t devAddr, const uint8_t* data, size_t size);

    /**
     * @brief Read raw bytes directly from the device.
     * @param devAddr Device address on the I2C bus.
     * @param data Buffer to store read bytes.
     * @param size Number of bytes to read.
     * @return true if successful, false otherwise.
     */
    bool readRaw(uint8_t devAddr, uint8_t* data, size_t size);

    /**
     * @brief Transmit a 16-bit command and then read responses after an optional delay.
     * @param devAddr Device address.
     * @param command 16-bit command.
     * @param data Buffer to store read bytes.
     * @param size Number of bytes to read.
     * @param delayMs Optional delay in milliseconds before reading.
     * @return true if successful, false otherwise.
     */
    bool readAfterCommand16(uint8_t devAddr, uint16_t command, uint8_t* data, size_t size, uint32_t delayMs = 0);

    /**
     * @brief Execute the hardware bus recovery routine.
     * Toggles SCL pin as a GPIO to release a locked SDA line (e.g., slave holding SDA low).
     */
    void recoverBus();

private:
    TwoWire& _wire;           ///< Reference to Wire peripheral
    int _sdaPin;             ///< SDA pin number
    int _sclPin;             ///< SCL pin number
    uint32_t _speed;         ///< Bus frequency (Hz)
    SemaphoreHandle_t _mutex; ///< FreeRTOS mutex to enforce exclusive access

    /**
     * @brief Internal lock helper.
     */
    bool lock(TickType_t waitTicks = 100);

    /**
     * @brief Internal unlock helper.
     */
    void unlock();
};

#endif // I2C_MANAGER_HPP
