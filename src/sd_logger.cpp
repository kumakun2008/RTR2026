/**
 * @file sd_logger.cpp
 * @brief Thread-safe SD card binary logger with ping-pong double buffering.
 * @author Team ЯTR
 * @date 2026-06-24
 */

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
    
    // 1. Create buffer mutex
    _bufferMutex = xSemaphoreCreateMutex();
    if (_bufferMutex == NULL) {
        return false;
    }
    
    // 2. Allocate ping-pong buffers in DMA-capable RAM (SRAM)
    _bufferA = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    _bufferB = (uint8_t*)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    
    // Fallback to standard heap if DMA caps allocation fails
    if (_bufferA == NULL) _bufferA = (uint8_t*)malloc(BUFFER_SIZE);
    if (_bufferB == NULL) _bufferB = (uint8_t*)malloc(BUFFER_SIZE);
    
    if (_bufferA == NULL || _bufferB == NULL) {
        return false;
    }
    
    _activeBuffer = _bufferA;
    _inactiveBuffer = _bufferB;
    _activeSize = 0;
    _inactiveSize = 0;
    
    // 3. Initialize SD card interface
    if (SD.begin(_csPin, *_spi, spiSpeed)) {
        _cardMounted = true;
        rotateLogFile();
    } else {
        // Warning: Card not inserted or SPI fault. We will operate in buffered/discard mode.
        _cardMounted = false;
    }
    
    // 4. Create background SD write task pinned to Core 0 (PRO_CPU)
    // Priority set to 2 (Medium priority, above idle and bluetooth SPP telemetry)
    BaseType_t result = xTaskCreatePinnedToCore(
        writeTask,
        "SDWriteTask",
        4096,
        this,
        2,
        &_writeTaskHandle,
        0 // Core 0
    );
    
    return (result == pdPASS);
}

bool SDLogger::logPacket(uint8_t sensorId, const void* payload, size_t payloadLen, uint64_t timestamp) {
    size_t packetSize = sizeof(LogHeader) + payloadLen + sizeof(LogFooter);
    if (packetSize > BUFFER_SIZE) return false; // Packet too large for buffer
    
    if (xSemaphoreTake(_bufferMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false; // Mutex acquisition timeout
    }
    
    // Check if packet fits in the active buffer
    if (_activeSize + packetSize > BUFFER_SIZE) {
        // Swap buffers if inactive buffer is empty (written to SD)
        if (_inactiveSize == 0) {
            uint8_t* temp = _activeBuffer;
            _activeBuffer = _inactiveBuffer;
            _inactiveBuffer = temp;
            
            _inactiveSize = _activeSize;
            _activeSize = 0;
            
            // Notify background task to write to SD
            if (_writeTaskHandle != NULL) {
                xTaskNotifyGive(_writeTaskHandle);
            }
        } else {
            // Buffer overflow - SD write task is lagging (e.g. SD card busy write latency)
            xSemaphoreGive(_bufferMutex);
            return false;
        }
    }
    
    // Assemble packet in the active buffer
    LogHeader header;
    header.sync = LOG_SYNC_WORD;
    header.timestamp = timestamp;
    header.sensor_id = sensorId;
    header.length = (uint8_t)payloadLen;
    
    uint8_t* dst = _activeBuffer + _activeSize;
    
    // 1. Copy Header
    memcpy(dst, &header, sizeof(LogHeader));
    
    // 2. Copy Payload
    if (payloadLen > 0 && payload != NULL) {
        memcpy(dst + sizeof(LogHeader), payload, payloadLen);
    }
    
    // 3. Compute CRC over Header + Payload
    uint16_t crcVal = calculateCRC16(dst, sizeof(LogHeader) + payloadLen);
    LogFooter footer;
    footer.crc16 = crcVal;
    
    // 4. Copy Footer
    memcpy(dst + sizeof(LogHeader) + payloadLen, &footer, sizeof(LogFooter));
    
    // 5. Update write pointer size
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
    if (!_cardMounted) return;
    
    // Scan root directory for next log_XXXX.bin file index
    int fileIdx = 1;
    for (; fileIdx <= 9999; fileIdx++) {
        snprintf(_fileName, sizeof(_fileName), "/log_%04d.bin", fileIdx);
        if (!SD.exists(_fileName)) {
            break;
        }
    }
    
    _logFile = SD.open(_fileName, FILE_WRITE);
    if (_logFile) {
        _fileOpen = true;
    } else {
        _fileOpen = false;
    }
}

void SDLogger::writeTask(void* pvParameters) {
    SDLogger* logger = (SDLogger*)pvParameters;
    while (true) {
        // Wait for buffer swap signal
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        logger->handleWrite();
    }
}

void SDLogger::handleWrite() {
    // Check if we have data to write
    if (_inactiveSize == 0) return;
    
    // Write data to SD Card (we do not hold the mutex here to allow logPacket to write to _activeBuffer)
    if (_cardMounted && _fileOpen) {
        size_t written = _logFile.write(_inactiveBuffer, _inactiveSize);
        if (written == _inactiveSize) {
            _logFile.flush(); // Commit write to flash memory
        } else {
            // Write error occurred, try to re-mount card or handle write error
            _fileOpen = false;
        }
    }
    
    // Clear the inactive size under mutex lock
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
