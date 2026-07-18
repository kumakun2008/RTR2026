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
volatile bool TimeSync::_hasValidCoord = false;  // RMC A=Active で座標が有効かどうか
static volatile uint32_t s_lastDualAntennaHeadingMs = 0;

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
    char buffer[256];  // 256B: UM982Cの長文NMEA対応
    int idx = 0;
    
    while (true) {
        while (sync->_gpsSerial->available() > 0) {
            char c = sync->_gpsSerial->read();
#ifdef NODE_GPS
            Serial.write(c);
#endif
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

        const char* timeStr = fields[1];
        const char* statusStr = fields[2];  // A=Active, V=Void
        const char* latStr  = fields[3];
        const char* latDir  = fields[4];
        const char* lonStr  = fields[5];
        const char* lonDir  = fields[6];
        const char* speedKnotsStr = fields[7];
        const char* courseStr     = fields[8];
        const char* dateStr       = fields[9];

        // --- 時刻は Active / Void 問わず更新 ---
        float utcFloat = atof(timeStr);
        int hh  = (int)(utcFloat / 10000.0);
        int mm  = ((int)(utcFloat / 100.0)) % 100;
        float ss = fmod(utcFloat, 100.0);
        int jstH = (hh + 9) % 24;
        float jstTime = jstH * 10000.0f + mm * 100.0f + ss;

        // PPS 同期 (Active でなくても時刻は信頼できる)
        if (_ppsTriggered && strlen(timeStr) >= 6 && strlen(dateStr) >= 6) {
            uint64_t utcEpochUs = convertNMEAToEpochUs(timeStr, dateStr);
            if (utcEpochUs > 0) {
                uint64_t jstEpochUs = utcEpochUs + (9ULL * 3600ULL * 1000000ULL);
                if (xSemaphoreTake(_timeMutex, portMAX_DELAY) == pdTRUE) {
                    _utcOffsetUs = jstEpochUs - _lastPPSUs;
                    _synced = true;
                    xSemaphoreGive(_timeMutex);
                }
            }
            _ppsTriggered = false;
        }

        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.utc = jstTime;

            if (statusStr[0] == 'A') {
                // Active: 座標・速度・進行方向を更新
                double latVal  = parseDegrees(latStr, latDir);
                double lonVal  = parseDegrees(lonStr, lonDir);
                float speedMs  = atof(speedKnotsStr) * 0.514444f;
                float headingDeg = atof(courseStr);

                _latestGPSData.latitude  = latVal;
                _latestGPSData.longitude = lonVal;
                _latestGPSData.speed     = speedMs;
                _hasValidCoord = true;

                if (millis() - s_lastDualAntennaHeadingMs > 3000) {
                    _latestGPSData.heading = (uint16_t)(headingDeg * 100.0f);
                }
            } else {
                // Void: 座標無効フラグのみ落とす（古い座標はそのまま保持）
                _hasValidCoord = false;
            }
            _hasFreshGPS = true;  // 時刻・RMC受信自体は常にフレッシュ
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
        const char* hdopStr = fields[8];
        const char* altStr = fields[9];
        
        uint8_t fixQual = atoi(fixQualStr);
        uint8_t satCount = atoi(satCountStr);
        float hdopVal = atof(hdopStr);
        float altMeters = atof(altStr);
        
        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.fix_status = fixQual;
            _latestGPSData.sat_count = satCount;
            _latestGPSData.hdop = hdopVal;
            _latestGPSData.altitude = altMeters;
            _hasFreshGPS = true;
            xSemaphoreGive(_timeMutex);
        }
    }
    // 3. Parse GPTHS / GNTHS or GPHDT / GNHDT (True Heading) for dual-antenna heading
    else if (strncmp(sentence, "$GPTHS", 6) == 0 || strncmp(sentence, "$GNTHS", 6) == 0 ||
             strncmp(sentence, "$GPHDT", 6) == 0 || strncmp(sentence, "$GNHDT", 6) == 0) {
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
        // GPTHS uses 'A' for active status. GPHDT uses 'T' for True.
        if (fields[2][0] != 'A' && fields[2][0] != 'T') return;
        
        float headingDeg = atof(fields[1]);
        
        if (xSemaphoreTake(_timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _latestGPSData.heading = (uint16_t)(headingDeg * 100.0f);
            _hasFreshGPS = true;
            s_lastDualAntennaHeadingMs = millis();
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
    
    char degBuf[16];
    char minBuf[32];
    
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
