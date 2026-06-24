/**
 * @file time_sync.hpp
 * @brief High-precision UTC clock synchronization using GPS NMEA data and PPS hardware interrupt.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef TIME_SYNC_HPP
#define TIME_SYNC_HPP

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @class TimeSync
 * @brief Handles PPS interrupt and GPS UART parsing to maintain a microsecond-accurate UTC clock.
 */
class TimeSync {
public:
    TimeSync();
    ~TimeSync();

    /**
     * @brief Initialize GPS UART and attach PPS interrupt.
     * @param gpsSerial Reference to HardwareSerial instance (e.g. Serial2).
     * @param ppsPin GPIO pin connected to the GPS PPS line.
     * @param rxPin UART RX pin.
     * @param txPin UART TX pin.
     * @return true on success, false otherwise.
     */
    bool begin(HardwareSerial& gpsSerial, int ppsPin, int rxPin, int txPin);

    /**
     * @brief Get the current microsecond-accurate UTC timestamp.
     * Fallbacks to system time if GPS sync is not yet available.
     * @return UTC Epoch timestamp in microseconds.
     */
    uint64_t getAbsoluteTimeUs();

    /**
     * @brief Check if the clock has been synchronized with GPS.
     */
    bool isSynced() const { return _synced; }

    /**
     * @brief Hardware interrupt service routine (ISR) for PPS signal rising edge.
     * Must be in IRAM.
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

    static volatile uint64_t _lastPPSUs;   ///< Internal ESP32 microsecond clock at last PPS pulse
    static volatile uint64_t _utcOffsetUs; ///< Offset to add to esp_timer_get_time() to get UTC Epoch Us
    static volatile bool _ppsTriggered;    ///< Flag indicating a new PPS pulse occurred
    
    // Mutex to protect access to offset configurations
    static SemaphoreHandle_t _timeMutex;

    /**
     * @brief Internal helper to parse a single NMEA sentence line.
     * Extracts date and time to establish UTC Epoch offset.
     */
    void parseNMEA(const char* sentence);

    /**
     * @brief Helper to convert NMEA Date and Time into Unix Epoch microseconds.
     */
    uint64_t convertNMEAToEpochUs(const char* timeStr, const char* dateStr);
};

#endif // TIME_SYNC_HPP
