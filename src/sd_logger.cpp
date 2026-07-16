/**
 * @file sd_logger.cpp
 * @brief Thread-safe SD card binary logger with ping-pong double buffering.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifdef ESP32

#include "sd_logger.hpp"
#include <esp_heap_caps.h>

SDLogger::SDLogger()
    : _bufferA(NULL), _bufferB(NULL), _activeBuffer(NULL), _inactiveBuffer(NULL),
      _activeSize(0), _inactiveSize(0), _bufferMutex(NULL), _writeTaskHandle(NULL),
      _csPin(-1), _spi(NULL), _cardMounted(false), _fileOpen(false) {
    memset(_fileName, 0, sizeof(_fileName));
}

SDLogger::~SDLogger() {
    if (_writeTaskHandle != NULL) {
        vTaskDelete(_writeTaskHandle);
    }
    
    if (_fileOpen) {
        _logFile.close();
    }
    
    if (_bufferA != NULL) {
        heap_caps_free(_bufferA);
    }
    if (_bufferB != NULL) {
        heap_caps_free(_bufferB);
    }
    if (_bufferMutex != NULL) {
        vSemaphoreDelete(_bufferMutex);
    }
}

bool SDLogger::begin(int csPin, SPIClass& spi, uint32_t spiSpeed) {
    _csPin = csPin;
    _spi = &spi;
    
    _bufferMutex = xSemaphoreCreateMutex();
    if (_bufferMutex == NULL) {
        return false;
    }
    
    _bufferA = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    _bufferB = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    
    if (_bufferA == NULL) _bufferA = (uint8_t*)malloc(BUFFER_SIZE);
    if (_bufferB == NULL) _bufferB = (uint8_t*)malloc(BUFFER_SIZE);
    
    if (_bufferA == NULL || _bufferB == NULL) {
        return false;
    }
    
    _activeBuffer = _bufferA;
    _inactiveBuffer = _bufferB;
    _activeSize = 0;
    _inactiveSize = 0;
    
    if (SD.begin(_csPin, *_spi, spiSpeed)) {
        _cardMounted = true;
        rotateLogFile();
    } else {
        _cardMounted = false;
        _fileOpen = false;
        _fileName[0] = '\0';
        // Explicitly set CS pin to HIGH to release SPI bus
        pinMode(_csPin, OUTPUT);
        digitalWrite(_csPin, HIGH);
    }
    
    if (!_cardMounted || !_fileOpen) {
        return false;
    }
    
    BaseType_t result = xTaskCreatePinnedToCore(
        writeTask,
        "SDWriteTask",
        4096,
        this,
        2,
        &_writeTaskHandle,
        0
    );
    
    return (result == pdPASS);
}

bool SDLogger::logPacket(uint8_t sensorId, const void* payload, size_t payloadLen, uint64_t timestamp) {
    size_t packetSize = sizeof(LogHeader) + payloadLen + sizeof(LogFooter);
    if (packetSize > BUFFER_SIZE) return false;
    
    if (xSemaphoreTake(_bufferMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    
    if (_activeSize + packetSize > BUFFER_SIZE) {
        if (_inactiveSize == 0) {
            uint8_t* temp = _activeBuffer;
            _activeBuffer = _inactiveBuffer;
            _inactiveBuffer = temp;
            
            _inactiveSize = _activeSize;
            _activeSize = 0;
            
            if (_writeTaskHandle != NULL) {
                xTaskNotifyGive(_writeTaskHandle);
            }
        } else {
            xSemaphoreGive(_bufferMutex);
            return false;
        }
    }
    
    LogHeader header;
    header.sync = LOG_SYNC_WORD;
    header.timestamp = timestamp;
    header.sensor_id = sensorId;
    header.length = (uint8_t)payloadLen;
    
    uint8_t* dst = _activeBuffer + _activeSize;
    memcpy(dst, &header, sizeof(LogHeader));
    
    if (payloadLen > 0 && payload != NULL) {
        memcpy(dst + sizeof(LogHeader), payload, payloadLen);
    }
    
    uint16_t crcVal = calculateCRC16(dst, sizeof(LogHeader) + payloadLen);
    LogFooter footer;
    footer.crc16 = crcVal;
    
    memcpy(dst + sizeof(LogHeader) + payloadLen, &footer, sizeof(LogFooter));
    _activeSize += packetSize;
    
    xSemaphoreGive(_bufferMutex);
    return true;
}

void SDLogger::triggerSync() {
    if (xSemaphoreTake(_bufferMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (_activeSize > 0 && _inactiveSize == 0) {
            uint8_t* temp = _activeBuffer;
            _activeBuffer = _inactiveBuffer;
            _inactiveBuffer = temp;
            
            _inactiveSize = _activeSize;
            _activeSize = 0;
            
            if (_writeTaskHandle != NULL) {
                xTaskNotifyGive(_writeTaskHandle);
            }
        }
        xSemaphoreGive(_bufferMutex);
    }
}

void SDLogger::rotateLogFile() {
    if (!_cardMounted) {
        _fileName[0] = '\0';
        return;
    }
    
    int fileIdx = 1;
    for (; fileIdx <= 9999; fileIdx++) {
        snprintf(_fileName, sizeof(_fileName), "/log_%04d.log", fileIdx);
        if (!SD.exists(_fileName)) {
            break;
        }
    }
    
    int retry = 0;
    while (retry < 3) {
        _logFile = SD.open(_fileName, FILE_WRITE);
        if (_logFile) {
            _fileOpen = true;
            return;
        }
        delay(50);
        retry++;
    }
    
    _fileOpen = false;
    _fileName[0] = '\0';
    Serial.println("[ERROR] Failed to open log file on SD card!");
}

void SDLogger::writeTask(void* pvParameters) {
    SDLogger* logger = (SDLogger*)pvParameters;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        logger->handleWrite();
    }
}

void SDLogger::handleWrite() {
    if (_inactiveSize == 0) return;
    
    if (_cardMounted && _fileOpen) {
        size_t written = _logFile.write(_inactiveBuffer, _inactiveSize);
        if (written == _inactiveSize) {
            _logFile.flush();
        } else {
            _fileOpen = false;
            Serial.printf("[ERROR] SD write mismatch! Written %u of %u bytes. Log file closed.\n", (unsigned int)written, (unsigned int)_inactiveSize);
            _logFile.close();
        }
    } else {
        static uint32_t lastWarn = 0;
        if (millis() - lastWarn > 5000) {
            Serial.println("[WARNING] SD Card not mounted or file not open. Data is being discarded.");
            lastWarn = millis();
        }
    }
    
    if (xSemaphoreTake(_bufferMutex, portMAX_DELAY) == pdTRUE) {
        _inactiveSize = 0;
        xSemaphoreGive(_bufferMutex);
    }
}

uint16_t SDLogger::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

#endif // ESP32
