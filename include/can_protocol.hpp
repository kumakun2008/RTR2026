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
