/**
 * @file time_sync.cpp
 * @brief High-precision UTC clock synchronization implementation.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "time_sync.hpp"
#include <time.h>

// Static members initialization
volatile uint64_t TimeSync::_lastPPSUs = 0;
volatile uint64_t TimeSync::_utcOffsetUs = 0;
volatile bool TimeSync::_ppsTriggered = false;
SemaphoreHandle_t TimeSync::_timeMutex = NULL;

TimeSync::TimeSync()
    : _gpsSerial(NULL), _ppsPin(-1), _rxPin(-1), _txPin(-1),
      _synced(false), _taskHandle(NULL) {}

TimeSync::~TimeSync() {
    if (_taskHandle != NULL) {
        vTaskDelete(_taskHandle);
    }
}

void IRAM_ATTR TimeSync::ppsISR() {
    _lastPPSUs = esp_timer_get_time();
    _ppsTriggered = true;
}

bool TimeSync::begin(HardwareSerial& gpsSerial, int ppsPin, int rxPin, int txPin) {
    _gpsSerial = &gpsSerial;
    _ppsPin = ppsPin;
    _rxPin = rxPin;
    _txPin = txPin;
    
    // Create static time mutex if it hasn't been created
    if (_timeMutex == NULL) {
        _timeMutex = xSemaphoreCreateMutex();
        if (_timeMutex == NULL) {
            return false;
        }
    }
    
    pinMode(_ppsPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppsPin), ppsISR, RISING);
    
    _gpsSerial->begin(115200, SERIAL_8N1, _rxPin, _txPin);
    
    // Spawn GPS parsing task on Core 1 (APP_CPU) with Medium priority (2)
    BaseType_t result = xTaskCreatePinnedToCore(
        gpsParseTask,
        "GPSParseTask",
        4096,
        this,
        2,
        &_taskHandle,
        1 // Core 1
    );
    
    return (result == pdPASS);
}

uint64_t TimeSync::getAbsoluteTimeUs() {
    uint64_t offset = 0;
    if (_timeMutex != NULL && xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        offset = _utcOffsetUs;
        xSemaphoreGive(_timeMutex);
    } else {
        // Fallback lock-free read if mutex is busy
        offset = _utcOffsetUs;
    }
    return esp_timer_get_time() + offset;
}

void TimeSync::gpsParseTask(void* pvParameters) {
    TimeSync* sync = (TimeSync*)pvParameters;
    char buffer[128];
    int idx = 0;
    
    while (true) {
        while (sync->_gpsSerial->available() > 0) {
            char c = sync->_gpsSerial->read();
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    buffer[idx] = '\0';
                    sync->parseNMEA(buffer);
                    idx = 0;
                }
            } else {
                if (idx < (int)sizeof(buffer) - 1) {
                    buffer[idx++] = c;
                } else {
                    idx = 0; // Overflow, discard
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks
    }
}

void TimeSync::parseNMEA(const char* sentence) {
    // We parse GPRMC or GNRMC sentences for time and date synchronization
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return;
    }
    
    char temp[128];
    strncpy(temp, sentence, sizeof(temp));
    temp[sizeof(temp) - 1] = '\0';
    
    char* fields[20];
    int fieldCount = 0;
    char* token = strtok(temp, ",");
    
    while (token != NULL && fieldCount < 20) {
        fields[fieldCount++] = token;
        token = strtok(NULL, ",");
    }
    
    // Check if sentence has enough fields (at least 10: 0 to 9)
    if (fieldCount < 10) return;
    
    // Check status: 'A' means active / valid GPS data
    const char* status = fields[2];
    if (status[0] != 'A') return;
    
    const char* timeStr = fields[1]; // hhmmss.ss
    const char* dateStr = fields[9]; // ddmmyy
    
    // If a PPS edge was recently captured, synchronize the clock
    if (_ppsTriggered) {
        uint64_t utcEpochUs = convertNMEAToEpochUs(timeStr, dateStr);
        if (utcEpochUs > 0) {
            if (xSemaphoreTake(_timeMutex, portMAX_DELAY) == pdTRUE) {
                // The time parsed represents the start of the current second (aligned with PPS)
                // Offset = absolute UTC time of PPS - ESP32 internal time at PPS
                _utcOffsetUs = utcEpochUs - _lastPPSUs;
                _synced = true;
                xSemaphoreGive(_timeMutex);
            }
        }
        _ppsTriggered = false;
    }
}

uint64_t TimeSync::convertNMEAToEpochUs(const char* timeStr, const char* dateStr) {
    if (strlen(timeStr) < 6 || strlen(dateStr) < 6) return 0;
    
    // Parse time components
    char hh[3] = { timeStr[0], timeStr[1], '\0' };
    char mm[3] = { timeStr[2], timeStr[3], '\0' };
    char ss[3] = { timeStr[4], timeStr[5], '\0' };
    
    int hour = atoi(hh);
    int min  = atoi(mm);
    int sec  = atoi(ss);
    
    // Parse date components
    char dd[3] = { dateStr[0], dateStr[1], '\0' };
    char mth[3] = { dateStr[2], dateStr[3], '\0' };
    char yy[3] = { dateStr[4], dateStr[5], '\0' };
    
    int day = atoi(dd);
    int month = atoi(mth);
    int year = atoi(yy);
    
    // Convert 2-digit year to 4-digit
    if (year < 80) {
        year += 2000;
    } else {
        year += 1900;
    }
    
    // Calculate fractional seconds if available
    float fracSeconds = 0.0f;
    if (timeStr[6] == '.') {
        fracSeconds = atof(&timeStr[6]);
    }
    
    struct tm t;
    t.tm_sec = sec;
    t.tm_min = min;
    t.tm_hour = hour;
    t.tm_mday = day;
    t.tm_mon = month - 1;     // tm_mon is 0-indexed (0 = January)
    t.tm_year = year - 1900;  // years since 1900
    t.tm_isdst = 0;           // UTC time has no DST
    
    time_t epochSeconds = mktime(&t);
    if (epochSeconds == -1) return 0;
    
    uint64_t epochUs = ((uint64_t)epochSeconds * 1000000ULL) + (uint64_t)(fracSeconds * 1000000.0f);
    return epochUs;
}
