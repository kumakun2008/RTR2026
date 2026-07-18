#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "can_protocol.hpp"

// ======== GPIO Pin Assignments (ESP32-C3) ========
// AS5600 I2C Pins
#define I2C_SDA_PIN  8
#define I2C_SCL_PIN  9

// CAN (TWAI) Pins (UART0競合回避のため D3/D10 に変更)
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_10

// ======== Sensor & CAN Timing Settings ========
#define SEND_INTERVAL_MS   10   // 100 Hz (10ms)
#define DEBUG_INTERVAL_MS  100  // 10 Hz (100ms)

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
        Serial.println("[ERR] Failed to install TWAI Driver");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERR] Failed to start TWAI Driver");
        twai_driver_uninstall();
        return false;
    }
    Serial.println("[OK] TWAI (CAN) 1Mbps started");
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
    Serial.begin(115200);
    // USB CDC シリアルの接続を最大3秒待機（PC接続時のデバッグ用）
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(10);
    }
    delay(500);
    Serial.println("=== Rudder Node (ESP32-C3 + AS5600) Start ===");

    // I2C Initialize
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 400kHz Fast Mode
    Serial.printf("[OK] I2C @ SDA=%d SCL=%d 400kHz\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // AS5600 Connection Check
    int retry = 0;
    while (!as5600CheckConnection() && retry < 5) {
        Serial.printf("AS5600 not found, retry %d/5...\n", ++retry);
        delay(500);
    }
    if (!as5600CheckConnection()) {
        Serial.println("[ERR] AS5600 connection failed! Operating with offline logic (0 deg).");
    } else {
        Serial.println("[OK] AS5600 connected");
        // Magnet check
        if (!as5600MagnetDetected()) {
            Serial.println("[WARN] AS5600: magnet not detected (MD=0). Check magnet placement.");
        } else {
            Serial.println("[OK] AS5600: magnet detected");
        }
    }

    // CAN Initialize
    canReady = initCAN();
    if (!canReady) {
        Serial.println("[WARN] CAN unavailable – data-only mode (Serial output only)");
    }

    Serial.println("=== Rudder Node initialized ===");
}

// --------------------------------------------------------------------------
// loop
// --------------------------------------------------------------------------
void loop() {
    static uint32_t lastSend  = 0;
    static uint32_t lastDebug = 0;
    uint32_t now = millis();

    // 100 Hz : AS5600 angle measurement & CAN transmission
    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;

        float angle = 0.0f;
        // AS5600 is connected and read successfully? Use it. Otherwise use 0.0f.
        if (as5600CheckConnection() && as5600ReadAngle(angle)) {
            rudderAngle_deg = angle;
        } else {
            rudderAngle_deg = 0.0f;
        }

        if (canReady) {
            if (!sendAngleCAN(rudderAngle_deg)) {
                Serial.println("[WARN] CAN TX failed");
            }
        }
    }

    // Debug output (Teleplot compatible)
    if (now - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = now;
        Serial.printf(">rudder_angle:%.2f\n", rudderAngle_deg);
        Serial.printf("[DBG] Rudder angle: %.2f deg | CAN: %s\n",
                      rudderAngle_deg, canReady ? "OK" : "FAIL");
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
