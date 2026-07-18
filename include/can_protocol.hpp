/**
 * @file can_protocol.hpp
 * @brief Common CAN communication protocol definitions, message IDs, and scaling factors.
 * @author Team ЯTR
 * @date 2026-06-25
 */

#ifndef CAN_PROTOCOL_HPP
#define CAN_PROTOCOL_HPP

#include <stdint.h>

// CAN IDs (11-bit standard ID / 1Mbps - all nodes unified)
// [Node 1] Main Board
constexpr uint32_t CAN_ID_MAIN_ACC_X   = 0x010;
constexpr uint32_t CAN_ID_MAIN_ACC_Y   = 0x011;
constexpr uint32_t CAN_ID_MAIN_ACC_Z   = 0x012;
constexpr uint32_t CAN_ID_MAIN_GYRO_X  = 0x013;
constexpr uint32_t CAN_ID_MAIN_GYRO_Y  = 0x014;
constexpr uint32_t CAN_ID_MAIN_GYRO_Z  = 0x015;
constexpr uint32_t CAN_ID_MAIN_MAG_X   = 0x016;
constexpr uint32_t CAN_ID_MAIN_MAG_Y   = 0x017;
constexpr uint32_t CAN_ID_MAIN_MAG_Z   = 0x018;

// [Node 3] Rudder Board
constexpr uint32_t CAN_ID_RUDDER_ACC_X = 0x020;
constexpr uint32_t CAN_ID_RUDDER_ACC_Y = 0x021;
constexpr uint32_t CAN_ID_RUDDER_ACC_Z = 0x022;
constexpr uint32_t CAN_ID_RUDDER_GYRO_X = 0x023;
constexpr uint32_t CAN_ID_RUDDER_GYRO_Y = 0x024;
constexpr uint32_t CAN_ID_RUDDER_GYRO_Z = 0x025;
constexpr uint32_t CAN_ID_RUDDER_ANGLE = 0x026;

// [Node 2] Pitot Board
constexpr uint32_t CAN_ID_PITOT_AIRSPEED = 0x030;
constexpr uint32_t CAN_ID_PITOT_AOA     = 0x031;
constexpr uint32_t CAN_ID_PITOT_AOS     = 0x032;
constexpr uint32_t CAN_ID_PITOT_PITCH   = 0x033;
constexpr uint32_t CAN_ID_PITOT_ROLL    = 0x034;
constexpr uint32_t CAN_ID_PITOT_YAW     = 0x035;
constexpr uint32_t CAN_ID_PITOT_TEMP    = 0x036;
constexpr uint32_t CAN_ID_PITOT_HUMID   = 0x037;
constexpr uint32_t CAN_ID_PITOT_ACC_X   = 0x038;
constexpr uint32_t CAN_ID_PITOT_ACC_Y   = 0x039;
constexpr uint32_t CAN_ID_PITOT_ACC_Z   = 0x03A;
constexpr uint32_t CAN_ID_PITOT_GYRO_X  = 0x03B;
constexpr uint32_t CAN_ID_PITOT_GYRO_Y  = 0x03C;
constexpr uint32_t CAN_ID_PITOT_GYRO_Z  = 0x03D;
constexpr uint32_t CAN_ID_PITOT_MAG_X   = 0x03E;
constexpr uint32_t CAN_ID_PITOT_MAG_Y   = 0x03F;
// NOTE (Fix #2): 0x040-0x041 = Main LPS22, 0x050-0x05A = GPS, 0x042-0x04F / 0x05B-0x05F are reserved.
// CAN_ID_PITOT_MAG_Z skips to 0x060 to avoid collision with the Main/GPS ID blocks above.
constexpr uint32_t CAN_ID_PITOT_MAG_Z   = 0x060;
constexpr uint32_t CAN_ID_PITOT_PRESS_RAW_SDP32   = 0x061;
constexpr uint32_t CAN_ID_PITOT_PRESS_RAW_SDP31_1 = 0x062;
constexpr uint32_t CAN_ID_PITOT_PRESS_RAW_SDP31_2 = 0x063;
constexpr uint32_t CAN_ID_PITOT_TEMP_RAW_SDP      = 0x064;
constexpr uint32_t CAN_ID_PITOT_TEMP_RAW_SDP31_1  = 0x065;
constexpr uint32_t CAN_ID_PITOT_TEMP_RAW_SDP31_2  = 0x066;

// [Node 1] Main Board - LPS22
constexpr uint32_t CAN_ID_MAIN_PRESS    = 0x040;
constexpr uint32_t CAN_ID_MAIN_TEMP     = 0x041;

// [Node 5] Altimeter Board
constexpr uint32_t CAN_ID_ALT_LIDAR     = 0x100;
constexpr uint32_t CAN_ID_ALT_US        = 0x101;

// [Node 6] Ladder Board (ESP32-C3 + AS5600)
constexpr uint32_t CAN_ID_LADDER_ANGLE   = 0x080;  ///< AS5600 raw angle (degrees * 100)

// [Node 7] Elevator Board (ESP32-WROOM-32E + MPU6050)
constexpr uint32_t CAN_ID_ELEV_PITCH     = 0x090;  ///< Madgwick pitch (degrees * 100)
constexpr uint32_t CAN_ID_ELEV_ROLL      = 0x091;  ///< Madgwick roll  (degrees * 100)
constexpr uint32_t CAN_ID_ELEV_YAW       = 0x092;  ///< Madgwick yaw   (degrees * 100)
constexpr uint32_t CAN_ID_ELEV_ACC_X     = 0x093;
constexpr uint32_t CAN_ID_ELEV_ACC_Y     = 0x094;
constexpr uint32_t CAN_ID_ELEV_ACC_Z     = 0x095;
constexpr uint32_t CAN_ID_ELEV_GYRO_X    = 0x096;
constexpr uint32_t CAN_ID_ELEV_GYRO_Y    = 0x097;
constexpr uint32_t CAN_ID_ELEV_GYRO_Z    = 0x098;

// [Node 4] GPS Board
constexpr uint32_t CAN_ID_GPS_LAT_UPPER = 0x050;
constexpr uint32_t CAN_ID_GPS_LAT_LOWER = 0x051;
constexpr uint32_t CAN_ID_GPS_LON_UPPER = 0x052;
constexpr uint32_t CAN_ID_GPS_LON_LOWER = 0x053;
constexpr uint32_t CAN_ID_GPS_ALT       = 0x054;
constexpr uint32_t CAN_ID_GPS_SPEED     = 0x055;
constexpr uint32_t CAN_ID_GPS_AZIMUTH   = 0x056;
constexpr uint32_t CAN_ID_GPS_UTC       = 0x057;
constexpr uint32_t CAN_ID_GPS_SATS      = 0x058;
constexpr uint32_t CAN_ID_GPS_HDOP      = 0x059;
constexpr uint32_t CAN_ID_GPS_FIX       = 0x05A;


// Status & UI
constexpr uint32_t CAN_ID_BATTERY_VOLT  = 0x070;
constexpr uint32_t CAN_ID_SD_STATUS     = 0x071;
constexpr uint32_t CAN_ID_CALIB_ZERO    = 0x072;
constexpr uint32_t CAN_ID_VOICE_CMD     = 0x073; // Speaker CMD (Async)
constexpr uint32_t CAN_ID_OTA_START     = 0x074; // OTA Trigger (Async)

// Node Heartbeat IDs (1Hz, payload = 1 byte node ID)
// Receiving HB = node is powered and alive (CAN connected)
// Receiving sensor data IDs = node is actively measuring
constexpr uint32_t CAN_ID_HB_MAIN      = 0x0F0;  ///< Main Board heartbeat
constexpr uint32_t CAN_ID_HB_PITOT     = 0x0F1;  ///< Pitot Board heartbeat
constexpr uint32_t CAN_ID_HB_RUDDER    = 0x0F2;  ///< Rudder Board heartbeat
constexpr uint32_t CAN_ID_HB_GPS       = 0x0F3;  ///< GPS Board heartbeat
constexpr uint32_t CAN_ID_HB_ALT       = 0x0F4;  ///< Altimeter Board heartbeat
constexpr uint32_t CAN_ID_HB_LADDER    = 0x0F5;  ///< Ladder Board heartbeat
constexpr uint32_t CAN_ID_HB_ELEVATOR  = 0x0F6;  ///< Elevator Board heartbeat
constexpr uint32_t CAN_ID_HB_SPEAKER   = 0x0F7;  ///< Speaker Board heartbeat

// Node ID byte values used in heartbeat payload
constexpr uint8_t NODE_ID_MAIN     = 0x01;
constexpr uint8_t NODE_ID_PITOT    = 0x02;
constexpr uint8_t NODE_ID_RUDDER   = 0x03;
constexpr uint8_t NODE_ID_GPS      = 0x04;
constexpr uint8_t NODE_ID_ALT      = 0x05;
constexpr uint8_t NODE_ID_LADDER   = 0x06;
constexpr uint8_t NODE_ID_ELEVATOR = 0x07;
constexpr uint8_t NODE_ID_SPEAKER  = 0x08;

// Voice alert codes for CAN_ID_VOICE_CMD payload
enum VoiceAlertCode : uint8_t {
    ALERT_CALIB_START = 1,
    ALERT_CALIB_END   = 2,
    ALERT_LOW_BATTERY = 3,
    ALERT_LOW_ALTITUDE = 4
};

// Scaling Factors
namespace CAN_Scale {
    constexpr float ACCEL     = 1000.0f;
    constexpr float GYRO      = 1000.0f;
    constexpr float MAG       = 10.0f;
    constexpr float PRESSURE  = 100.0f;
    constexpr float TEMP      = 100.0f;
    constexpr float HUMIDITY  = 100.0f;
    constexpr float ANGLE     = 100.0f;
    constexpr float DISTANCE  = 1000.0f;
    constexpr float GPS_ALT   = 100.0f;
    constexpr float GPS_SPEED = 100.0f;
    constexpr float GPS_AZIMUTH = 100.0f;
    constexpr float GPS_HDOP  = 100.0f;
    constexpr float BATTERY   = 100.0f;
}

#endif // CAN_PROTOCOL_HPP
