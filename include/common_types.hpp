/**
 * @file common_types.hpp
 * @brief Definition of data structures, binary logging packet layouts, and common types.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <stdint.h>

#pragma pack(push, 1)

/**
 * @brief Binary Log Packet Header (12 bytes)
 * Layout: [Sync 0xAA55 (2B)] + [Timestamp (8B)] + [Sensor ID (1B)] + [Length (1B)]
 */
struct LogHeader {
    uint16_t sync;        ///< Sync word (0xAA55)
    uint64_t timestamp;   ///< Absolute timestamp (microseconds)
    uint8_t  sensor_id;   ///< Identifier of the sensor node or source
    uint8_t  length;      ///< Length of the payload in bytes
};

/**
 * @brief Binary Log Packet Footer (2 bytes)
 * Layout: [CRC16 (2B)]
 */
struct LogFooter {
    uint16_t crc16;       ///< CRC16 checksum calculated over header + payload (excluding sync)
};

// --- Payload Structures (N-bytes) ---

/**
 * @brief IMU Sensor Payload (ICM-42688-P)
 * Size: 24 bytes (all float format)
 */
struct IMUPayload {
    float accel_x;        ///< Acceleration X (g)
    float accel_y;        ///< Acceleration Y (g)
    float accel_z;        ///< Acceleration Z (g)
    float gyro_x;         ///< Angular rate X (dps)
    float gyro_y;         ///< Angular rate Y (dps)
    float gyro_z;         ///< Angular rate Z (dps)
};

/**
 * @brief Magnetometer Sensor Payload (BM1422AGMV)
 * Size: 12 bytes
 */
struct MagPayload {
    float mag_x;          ///< Magnetic field X (uT)
    float mag_y;          ///< Magnetic field Y (uT)
    float mag_z;          ///< Magnetic field Z (uT)
};

/**
 * @brief Barometer Sensor Payload (LPS22)
 * Size: 8 bytes
 */
struct BaroPayload {
    float pressure;       ///< Barometric pressure (hPa)
    float temperature;    ///< Core temperature (C)
};

/**
 * @brief Differential Pressure / Pitot Payload (SDP31/SDP32)
 * Size: 16 bytes
 */
struct PitotPayload {
    float diff_press_sdp32;   ///< Primary pitot dynamic pressure (Pa)
    float diff_press_sdp31_1; ///< Backup 1 dynamic pressure (Pa)
    float diff_press_sdp31_2; ///< Backup 2 dynamic pressure (Pa)
    float temperature;        ///< Ambient temperature (C)
};

/**
 * @brief Battery Voltage Monitoring Payload
 * Size: 8 bytes
 */
struct BatteryPayload {
    float bus_voltage;    ///< Battery bus voltage (V)
    float shunt_current;  ///< Current draw (A)
};

/**
 * @brief GPS Data Payload (UM982C)
 * Size: 28 bytes
 */
struct GPSPayload {
    double latitude;      ///< Latitude (degrees, positive North)
    double longitude;     ///< Longitude (degrees, positive East)
    float  altitude;      ///< Altitude above ellipsoid (m)
    float  speed;         ///< Ground speed (m/s)
    uint8_t sat_count;    ///< Number of tracked satellites
    uint8_t fix_status;   ///< Fix quality (0=no fix, 1=single, 2=diff, 4=RTK fixed, etc.)
    uint16_t heading;     ///< Heading (degrees * 100, 0-35999)
    float  utc;           ///< UTC Time (HHMMSS.SS format)
};

/**
 * @brief Altimeter Data Payload
 * Size: 8 bytes
 */
struct AltimeterPayload {
    float ultrasonic_dist; ///< Distance from Ultrasonic sensor (m)
    float lidar_dist;      ///< Distance from single-point LiDAR (m)
};

/**
 * @brief Rudder Angle Data Payload
 * Size: 8 bytes
 */
struct RudderPayload {
    float rudder_angle;    ///< Control surface angle (degrees)
    float yaw_rate;        ///< Turn rate / Yaw axis gyro (dps)
};

#pragma pack(pop)

// Sync word definition
constexpr uint16_t LOG_SYNC_WORD = 0xAA55;

// Sensor IDs used in binary logging
enum LogSensorID : uint8_t {
    LOG_ID_MAIN_IMU      = 0x01,
    LOG_ID_MAIN_MAG      = 0x02,
    LOG_ID_MAIN_BARO     = 0x03,
    LOG_ID_PITOT_DATA    = 0x04,
    LOG_ID_BATTERY       = 0x05,
    LOG_ID_GPS           = 0x06,
    LOG_ID_ALTIMETER     = 0x07,
    LOG_ID_RUDDER        = 0x08,
    LOG_ID_EVENT_MARK    = 0xFF  ///< System events, calibration triggers, OTA markers, etc.
};

#endif // COMMON_TYPES_HPP
