/**
 * @file can_protocol.hpp
 * @brief Common CAN communication protocol definitions, message IDs, and scaling factors.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef CAN_PROTOCOL_HPP
#define CAN_PROTOCOL_HPP

#include <stdint.h>

/**
 * @brief CAN Node Identifiers.
 */
enum CANNodeID : uint8_t {
    CAN_NODE_MAIN      = 0x01, ///< Main board
    CAN_NODE_PITOT     = 0x02, ///< Pitot tube board
    CAN_NODE_GPS       = 0x03, ///< GPS board
    CAN_NODE_SPEAKER   = 0x04, ///< Speaker board
    CAN_NODE_DISPLAY   = 0x05, ///< Pre-flight display board
    CAN_NODE_ALTIMETER = 0x06, ///< Altimeter board
    CAN_NODE_RUDDER    = 0x07  ///< Rudder angle board
};

/**
 * @brief CAN Message Identifiers.
 */
enum CANMessageID : uint32_t {
    // Command Messages (Broadcast / Unicast)
    CAN_MSG_CMD_CALIB_ZERO = 0x010, ///< Command: Trigger Zero Calibration (payload: empty)
    CAN_MSG_CMD_OTA_START  = 0x011, ///< Command: Start OTA (payload: empty)
    
    // Main Board Telemetry (0x100 - 0x11F)
    CAN_MSG_MAIN_IMU_ACCEL = 0x100, ///< IMU Acceleration: Accel X, Y, Z (int16_t x3, scale 1000)
    CAN_MSG_MAIN_IMU_GYRO  = 0x101, ///< IMU Gyroscope: Gyro X, Y, Z (int16_t x3, scale 100)
    CAN_MSG_MAIN_MAG       = 0x102, ///< Magnetometer: Mag X, Y, Z (int16_t x3, scale 10)
    CAN_MSG_MAIN_ENV       = 0x103, ///< Environment: Pressure (int32_t, scale 100), Temp (int16_t, scale 100)
    CAN_MSG_MAIN_BATTERY   = 0x104, ///< Battery voltage: Voltage (uint16_t, scale 1000), Current (int16_t, scale 1000)

    // Pitot Board Telemetry (0x120 - 0x13F)
    CAN_MSG_PITOT_PRESSURES = 0x120, ///< SDP32, SDP31_1, SDP31_2 Pressures (int32_t x3, scale 100)
    CAN_MSG_PITOT_IMU       = 0x121, ///< Pitot IMU (BNO055) Euler Angles: Roll, Pitch, Yaw (int16_t x3, scale 100)
    CAN_MSG_PITOT_ENV       = 0x122, ///< Pitot SHT41 Temp (int16_t, scale 100), Humidity (uint16_t, scale 100)

    // GPS Board Telemetry (0x140 - 0x15F)
    CAN_MSG_GPS_POS_LAT_LON = 0x140, ///< GPS Latitude (int32_t, scale 10,000,000), Longitude (int32_t, scale 10,000,000)
    CAN_MSG_GPS_POS_ALT     = 0x141, ///< GPS Altitude (int32_t, scale 100), Speed (uint16_t, scale 100)
    CAN_MSG_GPS_TIME        = 0x142, ///< GPS Epoch Time (uint64_t ms)

    // Altimeter Board Telemetry (0x160 - 0x16F)
    CAN_MSG_ALTIMETER_DATA = 0x160, ///< Ultrasonic Dist (uint16_t mm), LiDAR Dist (uint16_t mm)

    // Rudder Board Telemetry (0x170 - 0x17F)
    CAN_MSG_RUDDER_ANGLE   = 0x170  ///< Rudder Angle: Angle (int16_t, scale 100), ICM-42688 Z-axis Gyro (int16_t, scale 100)
};

/**
 * @brief Scaling Factors for CAN bus data serialization.
 */
namespace CAN_Scale {
    constexpr float ACCEL  = 1000.0f; ///< 1g = 1000 units (milli-g)
    constexpr float GYRO   = 100.0f;  ///< 1 dps = 100 units (centi-dps)
    constexpr float MAG    = 10.0f;   ///< 1 uT = 10 units (deci-uT)
    constexpr float PRESS  = 100.0f;  ///< 1 Pa = 100 units (centi-Pascal)
    constexpr float TEMP   = 100.0f;  ///< 1 C = 100 units (centi-Celsius)
    constexpr float VOLT   = 1000.0f; ///< 1 V = 1000 units (mV)
    constexpr float CURR   = 1000.0f; ///< 1 A = 1000 units (mA)
    constexpr float GPS_DEG = 10000000.0f; ///< 1 degree = 1e7 units
    constexpr float SPEED  = 100.0f;  ///< 1 m/s = 100 units
    constexpr float ANGLE  = 100.0f;  ///< 1 deg = 100 units (centi-degree)
}

#endif // CAN_PROTOCOL_HPP
