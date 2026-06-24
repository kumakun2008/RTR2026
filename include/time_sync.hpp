/**
 * @file time_sync.hpp
 * @brief High-precision UTC clock synchronization using GPS NMEA data and PPS hardware interrupt.
 * Cross-platform compatible declaration.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef TIME_SYNC_HPP
#define TIME_SYNC_HPP

#include <Arduino.h>

#ifdef ESP32
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @class TimeSync
 * @brief Handles PPS interrupt and GPS UART parsing to maintain a microsecond-accurate UTC clock on ESP32.
 */
class TimeSync {
public:
    TimeSync();
    ~TimeSync();

    bool begin(HardwareSerial& gpsSerial, int ppsPin, int rxPin, int txPin);
    uint64_t getAbsoluteTimeUs();
    bool isSynced() const { return _synced; }
    static void IRAM_ATTR ppsISR();
    static void gpsParseTask(void* pvParameters);

private:
    HardwareSerial* _gpsSerial;
    int _ppsPin;
    int _rxPin;
    int _txPin;
    bool _synced;
    TaskHandle_t _taskHandle;

    static volatile uint64_t _lastPPSUs;
    static volatile uint64_t _utcOffsetUs;
    static volatile bool _ppsTriggered;
    static SemaphoreHandle_t _timeMutex;

    void parseNMEA(const char* sentence);
    uint64_t convertNMEAToEpochUs(const char* timeStr, const char* dateStr);
};

#else // Mock class for non-ESP32 platforms to allow clean compiles

class TimeSync {
public:
    TimeSync() {}
    ~TimeSync() {}
    template<typename T>
    bool begin(T& gpsSerial, int ppsPin, int rxPin, int txPin) { return false; }
    uint64_t getAbsoluteTimeUs() { return (uint64_t)micros(); }
    bool isSynced() const { return false; }
};

#endif // ESP32

#endif // TIME_SYNC_HPP
