/**
 * @file i2c_manager.cpp
 * @brief Thread-safe I2C bus wrapper implementation.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "i2c_manager.hpp"

I2CManager::I2CManager(TwoWire& wire, int sda, int scl, uint32_t speed)
    : _wire(wire), _sdaPin(sda), _sclPin(scl), _speed(speed), _mutex(NULL) {}

I2CManager::~I2CManager() {
    if (_mutex != NULL) {
        vSemaphoreDelete(_mutex);
    }
}

bool I2CManager::begin() {
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == NULL) {
        return false;
    }
    _wire.begin(_sdaPin, _sclPin, _speed);
    return true;
}

bool I2CManager::lock(TickType_t waitTicks) {
    if (_mutex == NULL) return false;
    return xSemaphoreTake(_mutex, waitTicks) == pdTRUE;
}

void I2CManager::unlock() {
    if (_mutex != NULL) {
        xSemaphoreGive(_mutex);
    }
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
        vTaskDelay(pdMS_TO_TICKS(delayMs));
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
    // Attempt to lock first, but force recovery even if locked to prevent permanent hang
    bool locked = lock(pdMS_TO_TICKS(10));
    
    // 1. Terminate current wire peripheral
    _wire.end();
    
    // 2. Configure SCL and SDA as GPIOs with internal pullups
    pinMode(_sclPin, OUTPUT_OPEN_DRAIN);
    pinMode(_sdaPin, INPUT_PULLUP);
    
    digitalWrite(_sclPin, HIGH);
    delayMicroseconds(5);
    
    // 3. If SDA is held low by a slave, cycle SCL to release the bus
    if (digitalRead(_sdaPin) == LOW) {
        for (int i = 0; i < 9; i++) {
            digitalWrite(_sclPin, LOW);
            delayMicroseconds(5);
            digitalWrite(_sclPin, HIGH);
            delayMicroseconds(5);
            
            // Check if slave released SDA
            if (digitalRead(_sdaPin) == HIGH) {
                break;
            }
        }
    }
    
    // 4. Force a STOP condition (SDA transitioning low-to-high while SCL is high)
    pinMode(_sdaPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(_sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(_sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_sdaPin, HIGH);
    delayMicroseconds(5);
    
    // 5. Reinitialize I2C wire peripheral
    _wire.begin(_sdaPin, _sclPin, _speed);
    
    if (locked) {
        unlock();
    }
}
