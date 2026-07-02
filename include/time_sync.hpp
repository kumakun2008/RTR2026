/**
 * @file time_sync.hpp
 * @brief High-precision UTC clock synchronization and GPS NMEA parsing.
 * Cross-platform compatible declaration.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef TIME_SYNC_HPP
#define TIME_SYNC_HPP

#include <Arduino.h>
#include "common_types.hpp"

#ifdef ESP32
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @class TimeSync
 * @brief Handles PPS interrupt and GPS NMEA stream parsing to extract high-precision UTC time,
 * 3D coordinates, speed, heading, and satellite statistics.
 */
class TimeSync {
public:
    TimeSync();
    ~TimeSync();

    /**
     * @brief Initialize GPS UART and attach PPS interrupt.
     */
    bool begin(HardwareSerial& gpsSerial, int ppsPin, int rxPin, int txPin);

    /**
     * @brief Get the current microsecond UTC timestamp.
     */
    uint64_t getAbsoluteTimeUs();

    /**
     * @brief Copy the latest parsed GPS payload.
     * @param payload Target structure to copy data into.
     * @return true if fresh data has been parsed, false otherwise.
     */
    bool getGPSData(GPSPayload& payload);

    /**
     * @brief Check if the clock has been synchronized with GPS.
     */
    bool isSynced() const { return _synced; }

    /**
     * @brief Hardware interrupt service routine for PPS signal.
     */
    static void IRAM_ATTR ppsISR();

    /**
     * @brief Task function to continuously read and parse GPS serial stream.
     */
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

    // Shared GPS data cache
    static GPSPayload _latestGPSData;
    static volatile bool _hasFreshGPS;

    /**
     * @brief Parse a single NMEA line sentence.
     */
    void parseNMEA(const char* sentence);

    /**
     * @brief Parse Latitude/Longitude NMEA degree-minute format to float degrees.
     */
    double parseDegrees(const char* val, const char* dir);

    /**
     * @brief Convert NMEA Date and Time strings into Unix Epoch microseconds.
     */
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
    bool getGPSData(GPSPayload& payload) {
        payload.latitude = 35.6812;
        payload.longitude = 139.7671;
        payload.altitude = 45.0f;
        payload.speed = 0.0f;
        payload.sat_count = 12;
        payload.fix_status = 4;
        payload.heading = 0;
        payload.utc = 123456.0f;
        return true;
    }
    bool isSynced() const { return false; }
};

#endif // ESP32

#endif // TIME_SYNC_HPP
