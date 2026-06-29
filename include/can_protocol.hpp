/**
 * @file can_protocol.hpp
 * @brief Common CAN communication protocol definitions, message IDs, and scaling factors.
 * @author Team ЯTR
 * @date 2026-06-25
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

// ==========================================
// NEW CAN ID ASSIGNMENTS (1Mbps / Standard 11-bit)
// ==========================================

// [High Priority] Flight Control & Safety
constexpr uint32_t CAN_ID_ATTITUDE      = 0x010; ///< Pitch (float) + Roll (float) [Source: Main / Rudder]
constexpr uint32_t CAN_ID_AIRSPEED      = 0x011; ///< SDP32 press (float) + Airspeed (float) [Source: Pitot]
constexpr uint32_t CAN_ID_RUDDER_ANGLE  = 0x012; ///< Rudder angle (float) [Source: Rudder]

// [Medium Priority] Environment & Navigation
constexpr uint32_t CAN_ID_ALTITUDE      = 0x020; ///< Main: Press (float) / Altimeter: LiDAR (float) + US (float)
constexpr uint32_t CAN_ID_GPS_POS       = 0x021; ///< Lat (int32_t * 1e7) + Lon (int32_t * 1e7) [Source: GPS]
constexpr uint32_t CAN_ID_AOA_AOS       = 0x022; ///< SDP31_1 press (float) + SDP31_2 press (float) [Source: Pitot]

// [Low Priority] Status & UI
constexpr uint32_t CAN_ID_BATTERY_VOLT  = 0x050; ///< Battery voltage (float) [Source: Main]
constexpr uint32_t CAN_ID_VOICE_CMD     = 0x060; ///< Voice alert index (uint8_t) [Source: Any -> Speaker]
constexpr uint32_t CAN_ID_CALIB_ZERO    = 0x070; ///< Calibration Trigger command [Source: Main]
constexpr uint32_t CAN_ID_OTA_START     = 0x080; ///< OTA Trigger command [Source: Main]

// Voice alert codes for CAN_ID_VOICE_CMD payload
enum VoiceAlertCode : uint8_t {
    ALERT_CALIB_START = 1,
    ALERT_CALIB_END   = 2,
    ALERT_LOW_BATTERY = 3,
    ALERT_LOW_ALTITUDE = 4
};

/**
 * @brief Scaling Factors for CAN bus data serialization.
 */
namespace CAN_Scale {
    constexpr float GPS_DEG = 10000000.0f; ///< 1 degree = 1e7 units
}

#endif // CAN_PROTOCOL_HPP
