/**
 * @file time_sync.cpp
 * @brief High-precision UTC clock synchronization and GPS NMEA parsing implementation.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifdef ESP32

#include "time_sync.hpp"
#include <time.h>

// Static members initialization
volatile uint64_t TimeSync::_lastPPSUs = 0;
volatile uint64_t TimeSync::_utcOffsetUs = 0;
volatile bool TimeSync::_ppsTriggered = false;
SemaphoreHandle_t TimeSync::_timeMutex = NULL;

GPSPayload TimeSync::_latestGPSData = { 0.0, 0.0, 0.0f, 0.0f, 0, 0, 0, 0.0f };
volatile bool TimeSync::_hasFreshGPS = false;

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
    
    if (_timeMutex == NULL) {
        _timeMutex = xSemaphoreCreateMutex();
        if (_timeMutex == NULL) {
            return false;
        }
    }
    
    pinMode(_ppsPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppsPin), ppsISR, RISING);
    
    _gpsSerial->begin(115200, SERIAL_8N1, _rxPin, _txPin);
    
    // Configure UM982C commands for the program
    delay(500); // Wait for module serial to stabilize
    _gpsSerial->println("unlog");
    delay(100);
    _gpsSerial->println("GPRMC 0.1"); // Recommended Minimum Navigation (10Hz)
    delay(100);
    _gpsSerial->println("GPGGA 0.1"); // GPS Fix Data (10Hz)
    delay(100);
    _gpsSerial->println("GPTHS 0.1"); // Dual-antenna True Heading (10Hz)
    delay(100);
    _gpsSerial->println("saveconfig");
    delay(200);
    
    BaseType_t result = xTaskCreatePinnedToCore(
        gpsParseTask,
        "GPSParseTask",
        4096,
        this,
        2,
        &_taskHandle,
        1
    );
    
    return (result == pdPASS);
}

uint64_t TimeSync::getAbsoluteTimeUs() {
    uint64_t offset = 0;
    if (_timeMutex != NULL && xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        offset = _utcOffsetUs;
        xSemaphoreGive(_timeMutex);
    } else {
        offset = _utcOffsetUs;
    }
    return esp_timer_get_time() + offset;
}

bool TimeSync::getGPSData(GPSPayload& payload) {
    bool fresh = false;
    if (_timeMutex != NULL && xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        payload = _latestGPSData;
        fresh = _hasFreshGPS;
        _hasFreshGPS = false; // Reset read flag
        xSemaphoreGive(_timeMutex);
    }
    return fresh;
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
                    idx = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TimeSync::parseNMEA(const char* sentence) {
    // 1. Parse GPRMC / GNRMC (Recommended Minimum Navigation Information)
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char temp[128];
        strncpy(temp, sentence, sizeof(temp));
        temp[sizeof(temp)-1] = '\0';
        
        char* fields[20];
        int fieldCount = 0;
        char* token = strtok(temp, ",");
        while (token != NULL && fieldCount < 20) {
            fields[fieldCount++] = token;
            token = strtok(NULL, ",");
        }
        
        if (fieldCount < 10) return;
        if (fields[2][0] != 'A') return; // Status: A=active, V=void
        
        const char* timeStr = fields[1];
        const char* latStr = fields[3];
        const char* latDir = fields[4];
        const char* lonStr = fields[5];
        const char* lonDir = fields[6];
        const char* speedKnotsStr = fields[7];
        const char* courseStr = fields[8];
        const char* dateStr = fields[9];
        
        double latVal = parseDegrees(latStr, latDir);
        double lonVal = parseDegrees(lonStr, lonDir);
        float speedMs = atof(speedKnotsStr) * 0.514444f; // knots to m/s
        float headingDeg = atof(courseStr);
        
        // Update UTC clock offset using PPS interrupt sync
        if (_ppsTriggered) {
            uint64_t utcEpochUs = convertNMEAToEpochUs(timeStr, dateStr);
            if (utcEpochUs > 0) {
                if (xSemaphoreTake(_timeMutex, portMAX_DELAY) == pdTRUE) {
                    _utcOffsetUs = utcEpochUs - _lastPPSUs;
                    _synced = true;
                    xSemaphoreGive(_timeMutex);
                }
            }
            _ppsTriggered = false;
        }
        
        // Update cache
        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.latitude = latVal;
            _latestGPSData.longitude = lonVal;
            _latestGPSData.speed = speedMs;
            _latestGPSData.heading = (uint16_t)(headingDeg * 100.0f);
            _latestGPSData.utc = atof(timeStr);
            _hasFreshGPS = true;
            xSemaphoreGive(_timeMutex);
        }
    }
    // 2. Parse GPGGA / GNGGA (Global Positioning System Fix Data)
    else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char temp[128];
        strncpy(temp, sentence, sizeof(temp));
        temp[sizeof(temp)-1] = '\0';
        
        char* fields[20];
        int fieldCount = 0;
        char* token = strtok(temp, ",");
        while (token != NULL && fieldCount < 20) {
            fields[fieldCount++] = token;
            token = strtok(NULL, ",");
        }
        
        if (fieldCount < 10) return;
        
        const char* fixQualStr = fields[6];
        const char* satCountStr = fields[7];
        const char* altStr = fields[9];
        
        uint8_t fixQual = atoi(fixQualStr);
        uint8_t satCount = atoi(satCountStr);
        float altMeters = atof(altStr);
        
        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.fix_status = fixQual;
            _latestGPSData.sat_count = satCount;
            _latestGPSData.altitude = altMeters;
            _hasFreshGPS = true;
            xSemaphoreGive(_timeMutex);
        }
    }
    // 3. Parse GPTHS / GNTHS (True Heading and Status) for dual-antenna heading
    else if (strncmp(sentence, "$GPTHS", 6) == 0 || strncmp(sentence, "$GNTHS", 6) == 0) {
        char temp[128];
        strncpy(temp, sentence, sizeof(temp));
        temp[sizeof(temp)-1] = '\0';
        
        char* fields[10];
        int fieldCount = 0;
        char* token = strtok(temp, ",");
        while (token != NULL && fieldCount < 10) {
            fields[fieldCount++] = token;
            token = strtok(NULL, ",");
        }
        
        if (fieldCount < 3) return;
        if (fields[2][0] != 'A') return; // Status: A=active, V=void
        
        float headingDeg = atof(fields[1]);
        
        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.heading = (uint16_t)(headingDeg * 100.0f);
            _hasFreshGPS = true;
            xSemaphoreGive(_timeMutex);
        }
    }
}

double TimeSync::parseDegrees(const char* val, const char* dir) {
    if (strlen(val) < 5) return 0.0;
    
    // Find decimal point
    const char* dot = strchr(val, '.');
    if (dot == NULL) return 0.0;
    
    // Minute starts 2 chars before dot
    int minStartIdx = (dot - val) - 2;
    if (minStartIdx < 0) return 0.0;
    
    char degBuf[8];
    char minBuf[10];
    
    strncpy(degBuf, val, minStartIdx);
    degBuf[minStartIdx] = '\0';
    strcpy(minBuf, val + minStartIdx);
    
    double deg = atof(degBuf);
    double min = atof(minBuf);
    
    double decDeg = deg + (min / 60.0);
    
    if (dir[0] == 'S' || dir[0] == 'W') {
        decDeg = -decDeg;
    }
    
    return decDeg;
}

uint64_t TimeSync::convertNMEAToEpochUs(const char* timeStr, const char* dateStr) {
    if (strlen(timeStr) < 6 || strlen(dateStr) < 6) return 0;
    
    char hh[3] = { timeStr[0], timeStr[1], '\0' };
    char mm[3] = { timeStr[2], timeStr[3], '\0' };
    char ss[3] = { timeStr[4], timeStr[5], '\0' };
    
    int hour = atoi(hh);
    int min  = atoi(mm);
    int sec  = atoi(ss);
    
    char dd[3] = { dateStr[0], dateStr[1], '\0' };
    char mth[3] = { dateStr[2], dateStr[3], '\0' };
    char yy[3] = { dateStr[4], dateStr[5], '\0' };
    
    int day = atoi(dd);
    int month = atoi(mth);
    int year = atoi(yy);
    
    if (year < 80) {
        year += 2000;
    } else {
        year += 1900;
    }
    
    float fracSeconds = 0.0f;
    if (timeStr[6] == '.') {
        fracSeconds = atof(&timeStr[6]);
    }
    
    struct tm t;
    t.tm_sec = sec;
    t.tm_min = min;
    t.tm_hour = hour;
    t.tm_mday = day;
    t.tm_mon = month - 1;     
    t.tm_year = year - 1900;  
    t.tm_isdst = 0;           
    
    time_t epochSeconds = mktime(&t);
    if (epochSeconds == -1) return 0;
    
    uint64_t epochUs = ((uint64_t)epochSeconds * 1000000ULL) + (uint64_t)(fracSeconds * 1000000.0f);
    return epochUs;
}

#endif // ESP32
