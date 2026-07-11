#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"

// TSD20 Lidar
#define TSD_RX 6  // 基板上のD4 (GPIO6)
#define TSD_TX 7  // 基板上のD5 (GPIO7)
#define OFFSET_CORRECTION 0  // 補正値

// URM37v5.0 Ultrasonic Sensor
#define URM_ECHO_PIN 2  // 基板上のD0 (GPIO2)
#define URM_TRIG_PIN 3  // 基板上のD1 (GPIO3)

// CAN GPIO Pins
#define CAN_TX_PIN GPIO_NUM_20
#define CAN_RX_PIN GPIO_NUM_21

// CAN IDs (11-bit standard)
#define LIDAR_CAN_ID      0x100
#define ULTRASONIC_CAN_ID 0x101

// I2C Slave Settings (per altimeter_spec.md)
#define I2C_SLAVE_ADDR 0x30
volatile uint8_t i2cRegisterPointer = 0x00;

// Shared data variables
volatile uint16_t lidarDistance_mm = 0;
volatile uint8_t ultrasoundDistance_cm = 0;

void onI2CReceive(int numBytes) {
    if (numBytes > 0) {
        i2cRegisterPointer = Wire.read();
        // Discard any additional bytes written
        while (Wire.available()) {
            Wire.read();
        }
    }
}

void onI2CRequest() {
    uint8_t buffer[4];
    buffer[0] = (lidarDistance_mm >> 8) & 0xFF;
    buffer[1] = lidarDistance_mm & 0xFF;
    buffer[2] = ultrasoundDistance_cm;
    buffer[3] = 0xAA; // Normal status flag

    if (i2cRegisterPointer < 4) {
        int bytesToSend = 4 - i2cRegisterPointer;
        Wire.write((const uint8_t*)&buffer[i2cRegisterPointer], bytesToSend);
    } else {
        Wire.write(0x00);
    }
}

void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    
    // Set timing to 1Mbps (Unified with other nodes to prevent bus errors)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("[OK] TWAI (CAN) Driver Installed");
    } else {
        Serial.println("[ERR] Failed to install TWAI Driver");
        return;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("[OK] TWAI (CAN) Driver Started");
    } else {
        Serial.println("[ERR] Failed to start TWAI Driver");
    }
}

void sendSensorData(uint32_t id, const uint8_t* data, uint8_t dlc) {
    twai_message_t message = {};
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = dlc;
    
    for (int i = 0; i < dlc; i++) {
        message.data[i] = data[i];
    }

    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(0));
    if (res == ESP_OK) {
        Serial.printf("[TX] Sent to ID 0x%03X, DLC: %d\n", id, dlc);
    } else {
        Serial.printf("[TX ERR] Failed to send to ID 0x%03X (Error: %d)\n", id, res);
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("--- Altimeter Node (ESP32-C3) Init ---");

    // Initialize TSD20 Lidar Serial at 460800 bps
    Serial1.begin(460800, SERIAL_8N1, TSD_RX, TSD_TX);

    // Initialize URM37v5.0 Pins
    pinMode(URM_TRIG_PIN, OUTPUT);
    pinMode(URM_ECHO_PIN, INPUT);
    digitalWrite(URM_TRIG_PIN, LOW);

    // Initialize I2C Slave at 0x30 (per altimeter_spec.md)
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.println("[OK] I2C Slave initialized at 0x30");

    // Initialize CAN (500kbps)
    initCAN();
}

void loop() {
    // 1. Read and parse Serial stream from TSD20 Lidar
    while (Serial1.available() >= 4) {
        if (Serial1.read() == 0x5C) {
            uint8_t dL = Serial1.read();
            uint8_t dH = Serial1.read();
            uint8_t chk = Serial1.read(); // checksum (ignored)

            uint16_t rawDistance = (uint16_t)((dH << 8) | dL);
            if (rawDistance != 50000) { // filter out-of-bounds readings
                int corrected = (int)rawDistance + OFFSET_CORRECTION;
                lidarDistance_mm = (corrected < 0) ? 0 : (uint16_t)corrected;
            }
        }
    }

    // 2. Measure Ultrasonic distance and broadcast both metrics (500ms cycle)
    static uint32_t lastMeasurement = 0;
    if (millis() - lastMeasurement >= 500) {
        lastMeasurement = millis();

        // URM37v5.0 trigger pulse
        digitalWrite(URM_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(URM_TRIG_PIN, LOW);

        long duration = pulseIn(URM_ECHO_PIN, HIGH, 30000); // 30ms timeout (approx 5m range)
        if (duration > 0) {
            float urm_mm = duration * 0.17f;
            ultrasoundDistance_cm = (uint8_t)(urm_mm / 10.0f); // Convert mm to cm
        } else {
            ultrasoundDistance_cm = 0; // out of range
        }

        // Broadcast Lidar range via CAN ID 0x100
        uint8_t lidarData[2];
        lidarData[0] = (lidarDistance_mm >> 8) & 0xFF;
        lidarData[1] = lidarDistance_mm & 0xFF;
        sendSensorData(LIDAR_CAN_ID, lidarData, 2);

        // Broadcast Ultrasonic range via CAN ID 0x101
        uint8_t ultraData[2];
        ultraData[0] = ultrasoundDistance_cm;
        ultraData[1] = 0xAA;
        sendSensorData(ULTRASONIC_CAN_ID, ultraData, 2);

        // Teleplot Output
        Serial.printf(">alt_lidar:%d\n", lidarDistance_mm);
        Serial.printf(">alt_ultrasonic:%d\n", ultrasoundDistance_cm);
    }
}
