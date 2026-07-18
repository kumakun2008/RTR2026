#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "can_protocol.hpp"

// ======== GPIO Pin Assignments (ESP32-C3) ========
// AS5600 I2C Pins
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// CAN (TWAI) Pins (GPIO20/21 に戻し、シリアル競合を避けるためSerialを完全無効化)
#define CAN_TX_PIN GPIO_NUM_20
#define CAN_RX_PIN GPIO_NUM_21

// ======== Sensor & CAN Timing Settings ========
#define SEND_INTERVAL_MS   10   // 100 Hz (10ms)

// ======== Global Variables ========
volatile float rudderAngle_deg = 0.0f;  // Latest Angle [°]
bool canReady = false;

// --------------------------------------------------------------------------
// AS5600 Utility Functions (Direct register access via Wire)
// --------------------------------------------------------------------------
bool as5600CheckConnection() {
    Wire.beginTransmission(0x36);
    return (Wire.endTransmission() == 0);
}

bool as5600MagnetDetected() {
    Wire.beginTransmission(0x36);
    Wire.write(0x0B); // STATUS register
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(0x36, 1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        return (status & 0x20); // MD (Magnet Detected) bit
    }
    return false;
}

bool as5600ReadAngle(float& angle) {
    Wire.beginTransmission(0x36);
    Wire.write(0x0E); // RAW ANGLE register (high byte)
    if (Wire.endTransmission() != 0) return false;
    
    Wire.requestFrom(0x36, 2);
    if (Wire.available() >= 2) {
        uint16_t rawAngle = (uint16_t)(Wire.read() << 8);
        rawAngle |= Wire.read();
        angle = (float)rawAngle * (360.0f / 4096.0f);
        return true;
    }
    return false;
}

// --------------------------------------------------------------------------
// CAN (TWAI) Driver Initialization (1Mbps)
// --------------------------------------------------------------------------
bool initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

// --------------------------------------------------------------------------
// CAN Frame Transmission (scaled int32: value * CAN_Scale::ANGLE)
// --------------------------------------------------------------------------
bool sendAngleCAN(float angleDeg) {
    int32_t val = (int32_t)(angleDeg * CAN_Scale::ANGLE);
    uint8_t data[4];
    memcpy(data, &val, 4);

    twai_message_t msg = {};
    msg.identifier       = CAN_ID_RUDDER_ANGLE;
    msg.extd             = 0;
    msg.rtr              = 0;
    msg.data_length_code = 4;
    memcpy(msg.data, data, 4);

    return (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK);
}

// --------------------------------------------------------------------------
// setup
// --------------------------------------------------------------------------
void setup() {
    // Serial communication completely removed to avoid conflict on GPIO20/21

    // I2C Initialize
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 400kHz Fast Mode

    // AS5600 Connection Check
    int retry = 0;
    while (!as5600CheckConnection() && retry < 5) {
        delay(500);
        retry++;
    }

    // CAN Initialize
    canReady = initCAN();
}

// --------------------------------------------------------------------------
// loop
// --------------------------------------------------------------------------
void loop() {
    static uint32_t lastSend  = 0;
    uint32_t now = millis();

    // 100 Hz : AS5600 angle measurement & CAN transmission
    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;

        float angle = 0.0f;
        if (as5600CheckConnection() && as5600ReadAngle(angle)) {
            rudderAngle_deg = angle;
        } else {
            rudderAngle_deg = 0.0f;
        }

        if (canReady) {
            sendAngleCAN(rudderAngle_deg);
        }
    }

    // 1Hz CAN Heartbeat
    static uint32_t lastHBrudder = 0;
    if (now - lastHBrudder >= 1000) {
        lastHBrudder = now;
        twai_message_t hbMsg = {};
        hbMsg.identifier = CAN_ID_HB_RUDDER;
        hbMsg.data_length_code = 1;
        hbMsg.data[0] = NODE_ID_RUDDER;
        twai_transmit(&hbMsg, pdMS_TO_TICKS(10));
    }
}
