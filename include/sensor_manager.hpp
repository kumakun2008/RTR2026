/**
 * @file sensor_manager.hpp
 * @brief Thread-safe sensor driver classes for all system peripherals.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "i2c_manager.hpp"
#include "common_types.hpp"

/**
 * @class SensorBase
 * @brief Abstract base class for all I2C sensors.
 */
class SensorBase {
public:
    /**
     * @brief Construct a SensorBase object.
     * @param i2c Reference to the I2CManager.
     * @param address I2C address of the device.
     */
    SensorBase(I2CManager& i2c, uint8_t address) : _i2c(i2c), _address(address), _initialized(false) {}
    
    virtual ~SensorBase() {}

    /**
     * @brief Initialize the sensor.
     * @return true if initialized successfully, false otherwise.
     */
    virtual bool begin() = 0;

    /**
     * @brief Check if the sensor is initialized.
     */
    bool isInitialized() const { return _initialized; }

protected:
    I2CManager& _i2c;      ///< Reference to the shared I2C bus manager
    uint8_t _address;      ///< Sensor I2C address
    bool _initialized;     ///< Initialization state flag
};

/**
 * @class ICM42688Sensor
 * @brief Driver class for the ICM-42688-P 6-axis IMU.
 */
class ICM42688Sensor : public SensorBase {
public:
    ICM42688Sensor(I2CManager& i2c, uint8_t address = 0x68);
    bool begin() override;

    /**
     * @brief Read accelerometer and gyroscope measurements.
     * @param payload Struct to store the parsed float readings.
     * @return true on success, false on communication failure.
     */
    bool read(IMUPayload& payload);
};

/**
 * @class BM1422Sensor
 * @brief Driver class for the BM1422AGMV 3-axis magnetometer.
 */
class BM1422Sensor : public SensorBase {
public:
    BM1422Sensor(I2CManager& i2c, uint8_t address = 0x0F);
    bool begin() override;

    /**
     * @brief Read magnetic field strength measurements.
     * @param payload Struct to store the parsed float readings (uT).
     * @return true on success, false on communication failure.
     */
    bool read(MagPayload& payload);
};

/**
 * @class LPS22Sensor
 * @brief Driver class for the LPS22HB/DF pressure sensor (barometer).
 */
class LPS22Sensor : public SensorBase {
public:
    LPS22Sensor(I2CManager& i2c, uint8_t address = 0x5C);
    bool begin() override;

    /**
     * @brief Read barometric pressure and temperature.
     * @param payload Struct to store parsed float readings (hPa and C).
     * @return true on success, false on communication failure.
     */
    bool read(BaroPayload& payload);
};

/**
 * @class SDP3xSensor
 * @brief Driver class for SDP31 and SDP32 differential pressure sensors (Pitot tube).
 */
class SDP3xSensor : public SensorBase {
public:
    /**
     * @brief Construct an SDP3x sensor.
     * @param i2c I2C bus manager.
     * @param address I2C address (0x21 for SDP32, 0x22 or 0x23 for SDP31).
     * @param isSDP32 Set to true if SDP32, false for SDP31 (impacts scaling).
     */
    SDP3xSensor(I2CManager& i2c, uint8_t address, bool isSDP32);
    bool begin() override;

    /**
     * @brief Read raw measurements, scale them, and apply calibration offset.
     * @param pressure Output variable to receive dynamic pressure (Pa).
     * @param temperature Output variable to receive temperature (C).
     * @return true on success, false on communication failure.
     */
    bool read(float& pressure, float& temperature);

    /**
     * @brief Set the dynamic zero-point calibration offset.
     * @param offset Offset value in Pascal to subtract from raw readings.
     */
    void setCalibrationOffset(float offset) { _zeroOffset = offset; }

    /**
     * @brief Get the current calibration offset.
     */
    float getCalibrationOffset() const { return _zeroOffset; }

private:
    bool _isSDP32;
    float _scaleFactor; ///< LSB per Pascal
    float _zeroOffset;  ///< Dynamically calculated zero-offset (Pa)
    uint32_t _errorCount = 0;
    uint32_t _lastInitTime = 0;
};

/**
 * @class BNO055Sensor
 * @brief Driver class for the BNO055 absolute orientation sensor (Pitot board auxiliary).
 */
class BNO055Sensor : public SensorBase {
public:
    BNO055Sensor(I2CManager& i2c, uint8_t address = 0x28);
    bool begin() override;

    /**
     * @brief Read Euler angles (Roll, Pitch, Yaw).
     * @param roll Output Roll (degrees).
     * @param pitch Output Pitch (degrees).
     * @param yaw Output Yaw (degrees).
     * @return true on success, false on communication failure.
     */
    bool read(float& roll, float& pitch, float& yaw);

    /**
     * @brief Read Euler angles, Acceleration, Gyroscope, and Magnetometer data.
     * @param roll, pitch, yaw Euler angles (degrees).
     * @param accX, accY, accZ Acceleration (m/s^2).
     * @param gyroX, gyroY, gyroZ Angular velocity (dps).
     * @param magX, magY, magZ Magnetic field (uT).
     * @return true on success, false on communication failure.
     */
    bool readAll(float& roll, float& pitch, float& yaw,
                 float& accX, float& accY, float& accZ,
                 float& gyroX, float& gyroY, float& gyroZ,
                 float& magX, float& magY, float& magZ);
};

/**
 * @class SHT41Sensor
 * @brief Driver class for the SHT41 temperature and relative humidity sensor.
 */
class SHT41Sensor : public SensorBase {
public:
    SHT41Sensor(I2CManager& i2c, uint8_t address = 0x44);
    bool begin() override;

    /**
     * @brief Read temperature and relative humidity.
     * @param temp Output temperature (C).
     * @param humidity Output humidity (% RH).
     * @return true on success, false on communication failure.
     */
    bool read(float& temp, float& humidity);
};

#endif // SENSOR_MANAGER_HPP
