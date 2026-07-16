/**
 * @file i2c_manager.cpp
 * @brief Thread-safe I2C bus wrapper implementation.
 * Cross-platform compatible for ESP32 and STM32.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "i2c_manager.hpp"

I2CManager::I2CManager(TwoWire& wire, int sda, int scl, uint32_t speed)
    : _wire(wire), _sdaPin(sda), _sclPin(scl), _speed(speed), _mutex(NULL) {}

I2CManager::~I2CManager() {
#ifdef ESP32
    if (_mutex != NULL) {
        vSemaphoreDelete(_mutex);
    }
#endif
}

bool I2CManager::begin() {
#ifdef ESP32
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == NULL) {
        return false;
    }
    _wire.begin(_sdaPin, _sclPin, _speed);
    _wire.setTimeOut(50); // Set timeout to 50ms to prevent infinite blocking on bus error
#else
    // For STM32 or other standard Arduino architectures
    _wire.begin();
    _wire.setClock(_speed);
#endif
    return true;
}

bool I2CManager::lock(TickType_t waitTicks) {
#ifdef ESP32
    if (_mutex == NULL) return false;
    return xSemaphoreTake(_mutex, waitTicks) == pdTRUE;
#else
    return true; // No-op lock on single threaded environments
#endif
}

void I2CManager::unlock() {
#ifdef ESP32
    if (_mutex != NULL) {
        xSemaphoreGive(_mutex);
    }
#endif
}

bool I2CManager::writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    if (!lock()) return false;
    
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    _wire.write(data);
    uint8_t error = _wire.endTransmission();
    
    unlock();
    return (error == 0);
}

bool I2CManager::readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t* data, size_t size) {
    if (!lock()) return false;
    
    _wire.beginTransmission(devAddr);
    _wire.write(regAddr);
    uint8_t error = _wire.endTransmission(false); // Send restart
    
    if (error != 0) {
        unlock();
        return false;
    }
    
    size_t requested = _wire.requestFrom(devAddr, (uint8_t)size);
    if (requested != size) {
        unlock();
        return false;
    }
    
    for (size_t i = 0; i < size; i++) {
        if (_wire.available()) {
            data[i] = _wire.read();
        } else {
            unlock();
            return false;
        }
    }
    
    unlock();
    return true;
}

bool I2CManager::writeRaw(uint8_t devAddr, const uint8_t* data, size_t size) {
    if (!lock()) return false;
    
    _wire.beginTransmission(devAddr);
    _wire.write(data, size);
    uint8_t error = _wire.endTransmission();
    
    unlock();
    return (error == 0);
}

bool I2CManager::readRaw(uint8_t devAddr, uint8_t* data, size_t size) {
    if (!lock()) return false;
    
    size_t requested = _wire.requestFrom(devAddr, (uint8_t)size);
    if (requested != size) {
        unlock();
        return false;
    }
    
    for (size_t i = 0; i < size; i++) {
        if (_wire.available()) {
            data[i] = _wire.read();
        } else {
            unlock();
            return false;
        }
    }
    
    unlock();
    return true;
}

bool I2CManager::readAfterCommand16(uint8_t devAddr, uint16_t command, uint8_t* data, size_t size, uint32_t delayMs) {
    if (!lock()) return false;
    
    _wire.beginTransmission(devAddr);
    _wire.write((uint8_t)(command >> 8));   // High byte
    _wire.write((uint8_t)(command & 0xFF)); // Low byte
    uint8_t error = _wire.endTransmission();
    
    if (error != 0) {
        unlock();
        return false;
    }
    
    if (delayMs > 0) {
#ifdef ESP32
        vTaskDelay(pdMS_TO_TICKS(delayMs));
#else
        delay(delayMs);
#endif
    }
    
    size_t requested = _wire.requestFrom(devAddr, (uint8_t)size);
    if (requested != size) {
        unlock();
        return false;
    }
    
    for (size_t i = 0; i < size; i++) {
        if (_wire.available()) {
            data[i] = _wire.read();
        } else {
            unlock();
            return false;
        }
    }
    
    unlock();
    return true;
}

bool I2CManager::readAfterCommand8(uint8_t devAddr, uint8_t command, uint8_t* data, size_t size, uint32_t delayMs) {
    if (!lock()) return false;
    
    _wire.beginTransmission(devAddr);
    _wire.write(command);
    uint8_t error = _wire.endTransmission();
    
    if (error != 0) {
        unlock();
        return false;
    }
    
    if (delayMs > 0) {
#ifdef ESP32
        vTaskDelay(pdMS_TO_TICKS(delayMs));
#else
        delay(delayMs);
#endif
    }
    
    size_t requested = _wire.requestFrom(devAddr, (uint8_t)size);
    if (requested != size) {
        unlock();
        return false;
    }
    
    for (size_t i = 0; i < size; i++) {
        if (_wire.available()) {
            data[i] = _wire.read();
        } else {
            unlock();
            return false;
        }
    }
    
    unlock();
    return true;
}

void I2CManager::recoverBus() {
    bool locked = lock(10);
    
    // 1. Terminate current wire peripheral
    _wire.end();
    
    // 2. Configure SCL and SDA as GPIOs with internal pullups
#ifdef INPUT_PULLUP
    pinMode(_sdaPin, INPUT_PULLUP);
#else
    pinMode(_sdaPin, INPUT);
#endif
    pinMode(_sclPin, OUTPUT);
    
    digitalWrite(_sclPin, HIGH);
    delayMicroseconds(5);
    
    // 3. Cycle SCL if SDA is held low
    if (digitalRead(_sdaPin) == LOW) {
        for (int i = 0; i < 9; i++) {
            digitalWrite(_sclPin, LOW);
            delayMicroseconds(5);
            digitalWrite(_sclPin, HIGH);
            delayMicroseconds(5);
            
            if (digitalRead(_sdaPin) == HIGH) {
                break;
            }
        }
    }
    
    // 4. Force a STOP condition
    pinMode(_sdaPin, OUTPUT);
    digitalWrite(_sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(_sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_sdaPin, HIGH);
    delayMicroseconds(5);
    
    // 5. Reinitialize I2C wire peripheral
#ifdef ESP32
    _wire.begin(_sdaPin, _sclPin, _speed);
#else
    _wire.begin();
    _wire.setClock(_speed);
#endif
    
    if (locked) {
        unlock();
    }
}
