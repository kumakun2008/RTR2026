/**
 * @file sd_logger.hpp
 * @brief Thread-safe SD card binary logger with ping-pong double buffering.
 * Compatible with ESP32 and provides stubs for non-ESP32 platforms.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef SD_LOGGER_HPP
#define SD_LOGGER_HPP

#include <Arduino.h>

#ifdef ESP32
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
    static constexpr size_t BUFFER_SIZE = 16384;

    SDLogger();
    ~SDLogger();

    bool begin(int csPin, SPIClass& spi, uint32_t spiSpeed = 20000000);
    bool logPacket(uint8_t sensorId, const void* payload, size_t payloadLen, uint64_t timestamp);
    void triggerSync();
    const char* getActiveFilename() const { return _fileName; }
    bool isActive() const { return _cardMounted && _fileOpen; }

private:
    uint8_t* _bufferA;
    uint8_t* _bufferB;
    uint8_t* _activeBuffer;
    uint8_t* _inactiveBuffer;
    size_t _activeSize;
    size_t _inactiveSize;

    SemaphoreHandle_t _bufferMutex;
    TaskHandle_t _writeTaskHandle;

    int _csPin;
    SPIClass* _spi;
    bool _cardMounted;
    bool _fileOpen;
    char _fileName[32];
    File _logFile;

    void rotateLogFile();
    static void writeTask(void* pvParameters);
    void handleWrite();
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
};

#else // Mock class for non-ESP32 platforms (like STM32) to compile cleanly

class SDLogger {
public:
    SDLogger() {}
    ~SDLogger() {}
    template<typename T>
    bool begin(int csPin, T& spi, uint32_t spiSpeed = 0) { return false; }
    bool logPacket(uint8_t sensorId, const void* payload, size_t payloadLen, uint64_t timestamp) { return false; }
    void triggerSync() {}
    const char* getActiveFilename() const { return ""; }
    bool isActive() const { return false; }
};

#endif // ESP32

#endif // SD_LOGGER_HPP
