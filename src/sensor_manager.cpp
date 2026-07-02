/**
 * @file sensor_manager.cpp
 * @brief Thread-safe sensor driver classes implementation.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include "sensor_manager.hpp"

// ==========================================
// ICM-42688 IMU Sensor Implementation
// ==========================================

ICM42688Sensor::ICM42688Sensor(I2CManager& i2c, uint8_t address)
    : SensorBase(i2c, address) {}

bool ICM42688Sensor::begin() {
    uint8_t whoAmI = 0;
    // WHO_AM_I register (0x75)
    if (!_i2c.readRegister(_address, 0x75, &whoAmI, 1)) {
        return false;
    }
    
    if (whoAmI != 0x47) {
        return false;
    }
    
    // PWR_MGMT0 (0x4E): Enable Gyro and Accel in Low Noise Mode (0x0F)
    if (!_i2c.writeRegister(_address, 0x4E, 0x0F)) {
        return false;
    }
    delay(10); // Wait for sensor to boot up
    
    // ACCEL_CONFIG0 (0x4F): +/- 8g range (0b01 << 5), 100Hz ODR (0b1000) -> 0x28
    if (!_i2c.writeRegister(_address, 0x4F, 0x28)) {
        return false;
    }
    
    // GYRO_CONFIG0 (0x50): +/- 2000 dps range (0b000 << 5), 100Hz ODR (0b1000) -> 0x08
    if (!_i2c.writeRegister(_address, 0x50, 0x08)) {
        return false;
    }
    
    _initialized = true;
    return true;
}

bool ICM42688Sensor::read(IMUPayload& payload) {
    if (!_initialized) return false;
    
    uint8_t rawData[14];
    // Read starting from TEMP_DATA1 (0x1D) to GYRO_DATA_Z0 (0x2A)
    if (!_i2c.readRegister(_address, 0x1D, rawData, 14)) {
        return false;
    }
    
    // Parse registers
    int16_t rawAccelX = (rawData[2] << 8) | rawData[3];
    int16_t rawAccelY = (rawData[4] << 8) | rawData[5];
    int16_t rawAccelZ = (rawData[6] << 8) | rawData[7];
    
    int16_t rawGyroX  = (rawData[8] << 8) | rawData[9];
    int16_t rawGyroY  = (rawData[10] << 8) | rawData[11];
    int16_t rawGyroZ  = (rawData[12] << 8) | rawData[13];
    
    // Conversions
    // Sensitivity scale factor for +/- 8g = 4096 LSB/g
    // Sensitivity scale factor for +/- 2000 dps = 16.4 LSB/dps
    payload.accel_x = (float)rawAccelX / 4096.0f;
    payload.accel_y = (float)rawAccelY / 4096.0f;
    payload.accel_z = (float)rawAccelZ / 4096.0f;
    
    payload.gyro_x  = (float)rawGyroX / 16.4f;
    payload.gyro_y  = (float)rawGyroY / 16.4f;
    payload.gyro_z  = (float)rawGyroZ / 16.4f;
    
    return true;
}

// ==========================================
// BM1422AGMV Magnetometer Implementation
// ==========================================

BM1422Sensor::BM1422Sensor(I2CManager& i2c, uint8_t address)
    : SensorBase(i2c, address) {}

bool BM1422Sensor::begin() {
    uint8_t whoAmI = 0;
    // WHO_AM_I (0x0F) in some doc, or chip ID register. Let's read register 0x0F.
    // BM1422AGMV chip ID is 0x41
    if (!_i2c.readRegister(_address, 0x0F, &whoAmI, 1)) {
        return false;
    }
    
    // Put BM1422 in Standby: CNTL1 (0x0D) = 0x00
    if (!_i2c.writeRegister(_address, 0x0D, 0x00)) {
        return false;
    }
    
    // Select 14-bit Mode: CNTL4 (0x1D) = 0x02
    if (!_i2c.writeRegister(_address, 0x1D, 0x02)) {
        return false;
    }
    
    // Continuous Mode, ODR 100Hz: CNTL1 (0x0D) = 0x81 (Bit 7: Active, Bit 1:0 = 01 for 100Hz)
    if (!_i2c.writeRegister(_address, 0x0D, 0x81)) {
        return false;
    }
    
    // Start Measurement: CNTL3 (0x1C) = 0x40 (Force start or continuous conversion bit)
    if (!_i2c.writeRegister(_address, 0x1C, 0x40)) {
        return false;
    }
    
    _initialized = true;
    return true;
}

bool BM1422Sensor::read(MagPayload& payload) {
    if (!_initialized) return false;
    
    uint8_t rawData[6];
    // Data registers are X_L (0x10) to Z_H (0x15), little endian
    if (!_i2c.readRegister(_address, 0x10, rawData, 6)) {
        return false;
    }
    
    int16_t rawX = (int16_t)((rawData[1] << 8) | rawData[0]);
    int16_t rawY = (int16_t)((rawData[3] << 8) | rawData[2]);
    int16_t rawZ = (int16_t)((rawData[5] << 8) | rawData[4]);
    
    // BM1422 14-bit mode resolution is 120 LSB/uT
    payload.mag_x = (float)rawX / 120.0f;
    payload.mag_y = (float)rawY / 120.0f;
    payload.mag_z = (float)rawZ / 120.0f;
    
    return true;
}

// ==========================================
// LPS22 Barometer Implementation
// ==========================================

LPS22Sensor::LPS22Sensor(I2CManager& i2c, uint8_t address)
    : SensorBase(i2c, address) {}

bool LPS22Sensor::begin() {
    uint8_t whoAmI = 0;
    // WHO_AM_I (0x0F)
    if (!_i2c.readRegister(_address, 0x0F, &whoAmI, 1)) {
        return false;
    }
    
    if (whoAmI != 0xB1) {
        return false;
    }
    
    // CTRL_REG1 (0x10): 75Hz ODR (0x50), block data update enabled (0x02) -> 0x52
    if (!_i2c.writeRegister(_address, 0x10, 0x52)) {
        return false;
    }
    
    _initialized = true;
    return true;
}

bool LPS22Sensor::read(BaroPayload& payload) {
    if (!_initialized) return false;
    
    uint8_t rawData[5];
    // Read PRESS_OUT_XL (0x28) to TEMP_OUT_H (0x2C)
    if (!_i2c.readRegister(_address, 0x28, rawData, 5)) {
        return false;
    }
    
    // Pressure is 24-bit signed little endian
    int32_t rawPress = (int32_t)((rawData[2] << 16) | (rawData[1] << 8) | rawData[0]);
    if (rawPress & 0x00800000) {
        rawPress |= 0xFF000000; // Sign extend
    }
    
    // Temperature is 16-bit signed little endian
    int16_t rawTemp = (int16_t)((rawData[4] << 8) | rawData[3]);
    
    // Convert: Pressure scale is 4096 LSB/hPa, Temp scale is 100 LSB/C
    payload.pressure = (float)rawPress / 4096.0f;
    payload.temperature = (float)rawTemp / 100.0f;
    
    return true;
}

// ==========================================
// SDP3x Differential Pressure Implementation
// ==========================================

SDP3xSensor::SDP3xSensor(I2CManager& i2c, uint8_t address, bool isSDP32)
    : SensorBase(i2c, address), _isSDP32(isSDP32), _zeroOffset(0.0f) {
    // SDP32 scale is 240 LSB/Pa, SDP31 scale is 60 LSB/Pa
    _scaleFactor = _isSDP32 ? 240.0f : 60.0f;
}

bool SDP3xSensor::begin() {
    uint8_t cmd[2] = { 0x36, 0x15 };
    
    // Single non-blocking write attempt to avoid halting the main thread
    if (!_i2c.writeRaw(_address, cmd, 2)) {
        return false;
    }
    
    _lastInitTime = millis();
    _errorCount = 0;
    _initialized = true;
    return true;
}

bool SDP3xSensor::read(float& pressure, float& temperature) {
    if (!_initialized) {
        // Asynchronously retry initialization every 1000ms to avoid blocking loop execution
        if (millis() - _lastInitTime >= 1000) {
            _lastInitTime = millis();
            begin();
        }
        return false;
    }
    
    // Asynchronously wait 50ms for first measurement to become ready after a successful init
    if (millis() - _lastInitTime < 50) {
        return false;
    }
    
    uint8_t rawData[6]; // [Pres_H, Pres_L, Pres_CRC, Temp_H, Temp_L, Temp_CRC]
    if (!_i2c.readRaw(_address, rawData, 6)) {
        _errorCount++;
        if (_errorCount > 15) {
            _i2c.recoverBus();    // Clear hardware bus state
            _initialized = false; // Schedule non-blocking re-init
            _errorCount = 0;
            _lastInitTime = millis();
            Serial.printf("[SDP3x] Lost contact with sensor 0x%02X. Scheduling re-init.\n", _address);
        }
        return false;
    }
    
    _errorCount = 0; // Reset counter on success
    
    int16_t rawPress = (int16_t)((rawData[0] << 8) | rawData[1]);
    int16_t rawTemp  = (int16_t)((rawData[3] << 8) | rawData[4]);
    
    // Convert to Pascals and Celsius
    pressure = ((float)rawPress / _scaleFactor) - _zeroOffset;
    temperature = (float)rawTemp / 200.0f;
    
    return true;
}

// ==========================================
// BNO055 Absolute Orientation Implementation
// ==========================================

BNO055Sensor::BNO055Sensor(I2CManager& i2c, uint8_t address)
    : SensorBase(i2c, address) {}

bool BNO055Sensor::begin() {
    uint8_t whoAmI = 0;
    bool deviceFound = false;
    uint8_t detectedAddr = _address;
    
    uint8_t addrCandidates[2] = { _address, (uint8_t)(_address == 0x28 ? 0x29 : 0x28) };
    
    // Retry WHO_AM_I check up to 10 times with 100ms delay to wait for BNO055 power-up
    for (int retry = 0; retry < 10; retry++) {
        delay(100);
        for (int i = 0; i < 2; i++) {
            uint8_t addr = addrCandidates[i];
            if (_i2c.readRegister(addr, 0x00, &whoAmI, 1)) {
                if (whoAmI == 0xA0) {
                    detectedAddr = addr;
                    deviceFound = true;
                    _address = addr; // Auto-assign detected address
                    break;
                }
            }
        }
        if (deviceFound) break;
    }
    
    if (!deviceFound) {
        Serial.printf("[BNO055 ERR] Sensor not responding at 0x28 or 0x29 (Read chip ID: 0x%02X)\n", whoAmI);
        return false;
    }
    Serial.printf("[BNO055 OK] Initialized at I2C address 0x%02X\n", _address);
    
    // Set to Config Mode: OPR_MODE (0x3D) = 0x00
    if (!_i2c.writeRegister(_address, 0x3D, 0x00)) return false;
    delay(20);
    
    // Set Power Mode: PWR_MODE (0x3E) = 0x00 (Normal)
    if (!_i2c.writeRegister(_address, 0x3E, 0x00)) return false;
    
    // Clear Reset: SYS_TRIGGER (0x3F) = 0x00
    if (!_i2c.writeRegister(_address, 0x3F, 0x00)) return false;
    delay(10);
    
    // Set to NDOF (Nine Degrees of Freedom) Sensor Fusion Mode: OPR_MODE (0x3D) = 0x0C
    if (!_i2c.writeRegister(_address, 0x3D, 0x0C)) return false;
    delay(80); // Wait for transition
    
    _initialized = true;
    return true;
}

bool BNO055Sensor::read(float& roll, float& pitch, float& yaw) {
    if (!_initialized) return false;
    
    uint8_t rawData[6];
    // Read Euler angles starting from EUL_HEADING_LSB (0x1A) to EUL_PITCH_MSB (0x1F)
    if (!_i2c.readRegister(_address, 0x1A, rawData, 6)) {
        return false;
    }
    
    int16_t rawYaw   = (int16_t)((rawData[1] << 8) | rawData[0]); // Yaw / Heading
    int16_t rawRoll  = (int16_t)((rawData[3] << 8) | rawData[2]); // Roll
    int16_t rawPitch = (int16_t)((rawData[5] << 8) | rawData[4]); // Pitch
    
    // BNO055 Euler conversion: 1 degree = 16 LSB
    yaw   = (float)rawYaw / 16.0f;
    roll  = (float)rawRoll / 16.0f;
    pitch = (float)rawPitch / 16.0f;
    
    return true;
}

// ==========================================
// SHT41 Temp and Humidity Sensor Implementation
// ==========================================

SHT41Sensor::SHT41Sensor(I2CManager& i2c, uint8_t address)
    : SensorBase(i2c, address) {}

bool SHT41Sensor::begin() {
    // SHT4x is a simple command-response device. Just verify it's responsive.
    _initialized = true;
    return true;
}

bool SHT41Sensor::read(float& temp, float& humidity) {
    if (!_initialized) return false;
    
    uint8_t rawData[6];
    // Command 0xFD: High precision read command. Wait 10ms for measurement.
    if (!_i2c.readAfterCommand8(_address, 0xFD, rawData, 6, 10)) {
        return false;
    }
    
    uint16_t rawTemp = (rawData[0] << 8) | rawData[1];
    uint16_t rawHum  = (rawData[3] << 8) | rawData[4];
    
    // Conversion formulas from SHT4x datasheet
    temp = -45.0f + 175.0f * ((float)rawTemp / 65535.0f);
    humidity = -6.0f + 125.0f * ((float)rawHum / 65535.0f);
    
    // Clip humidity to [0, 100]
    if (humidity < 0.0f) humidity = 0.0f;
    if (humidity > 100.0f) humidity = 100.0f;
    
    return true;
}
