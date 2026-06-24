/**
 * @file sd_logger.hpp
 * @brief Thread-safe SD card binary logger with ping-pong double buffering.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef SD_LOGGER_HPP
#define SD_LOGGER_HPP

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "common_types.hpp"

/**
 * @class SDLogger
 * @brief Handles background binary logging to SD card using double buffering.
 */
class SDLogger {
public:
    /**
     * @brief Size of each ping-pong buffer in bytes.
     * Large enough to hold 200ms worth of data at 100Hz (~8KB).
     */
    static constexpr size_t BUFFER_SIZE = 16384;

    SDLogger();
    ~SDLogger();

    /**
     * @brief Initialize SD card, SPI, buffers, and background write task.
     * @param csPin SPI Chip Select pin.
     * @param spi Reference to SPIClass instance.
     * @param spiSpeed SPI clock speed (Hz).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(int csPin, SPIClass& spi, uint32_t spiSpeed = 20000000);

    /**
     * @brief Log a data packet. Serializes the header, payload, and CRC footer.
     * Thread-safe; writes to the active ping-pong buffer.
     * @param sensorId Identifier of the logging sensor.
     * @param payload Pointer to raw payload data.
     * @param payloadLen Length of the payload in bytes.
     * @param timestamp Microsecond absolute timestamp.
     * @return true if successfully queued, false if buffer overflow (SD card write lag).
     */
    bool logPacket(uint8_t sensorId, const void* payload, size_t payloadLen, uint64_t timestamp);

    /**
     * @brief Force swap active and inactive buffers, triggering write.
     */
    void triggerSync();

    /**
     * @brief Get the filename currently being written to.
     */
    const char* getActiveFilename() const { return _fileName; }

    /**
     * @brief Check if logging is active and functional.
     */
    bool isActive() const { return _cardMounted && _fileOpen; }

private:
    // Ping-pong buffers
    uint8_t* _bufferA;            ///< Pointer to buffer A allocation
    uint8_t* _bufferB;            ///< Pointer to buffer B allocation
    uint8_t* _activeBuffer;       ///< Pointer to current writing buffer
    uint8_t* _inactiveBuffer;     ///< Pointer to buffer waiting for write
    size_t _activeSize;           ///< Number of bytes in active buffer
    size_t _inactiveSize;         ///< Number of bytes in inactive buffer

    // Mutex for buffer swaps and appends
    SemaphoreHandle_t _bufferMutex;
    // Task handle for background SD writing
    TaskHandle_t _writeTaskHandle;

    int _csPin;
    SPIClass* _spi;
    bool _cardMounted;
    bool _fileOpen;
    char _fileName[32];
    File _logFile;

    /**
     * @brief Scan SD card directory to find the next log file index.
     * e.g., if log_0001.bin exists, it creates log_0002.bin.
     */
    void rotateLogFile();

    /**
     * @brief FreeRTOS background task function for writing inactive buffer to SD.
     */
    static void writeTask(void* pvParameters);

    /**
     * @brief Internal implementation of the write process.
     */
    void handleWrite();

    /**
     * @brief Compute standard CRC16 CCITT (0x1021).
     */
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
};

#endif // SD_LOGGER_HPP
