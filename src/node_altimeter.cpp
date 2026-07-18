#include <Arduino.h>
#include <Wire.h>
#include "can_protocol.hpp"
#include "can_manager.hpp"

// TSD20 Lidar
#define TSD_RX 6  // 基板上のD4 (GPIO6)
#define TSD_TX 7  // 基板上のD5 (GPIO7)
#define OFFSET_CORRECTION 0  // 補正値

// URM37v5.0 Ultrasonic Sensor
#define URM_ECHO_PIN 3  // 基板上のD1 (GPIO3)
#define URM_TRIG_PIN 4  // 基板上のD2 (GPIO4) (Corrected based on altimeter.txt)

// CAN GPIO Pins (GPIO20/21 に戻し、シリアル競合を避けるためSerialを完全無効化)
#define CAN_TX_PIN 20
#define CAN_RX_PIN 21

// CAN IDs (11-bit standard)
#define LIDAR_CAN_ID      0x100
#define ULTRASONIC_CAN_ID 0x101

// I2C Slave Settings (per altimeter_spec.md)
#define I2C_SLAVE_ADDR 0x30
volatile uint8_t i2cRegisterPointer = 0x00;

// Shared data variables
volatile uint16_t lidarDistance_mm = 65535;
volatile uint16_t ultrasoundDistance_cm = 65535; // uint16_t: max 65535cm

// CAN Manager Instance
CANManager canBus;

// TSD20受信バッファ (From altimeter.txt)
uint8_t rxBuffer[4];
uint8_t bufferIndex = 0;
bool packetStarted = false;

// TSD20 チェックサム計算 (From altimeter.txt)
uint8_t calculateChecksum(uint8_t *_pbuff, uint16_t _cmdLen) {
    uint8_t cmd_sum = 0;
    for (uint16_t i = 0; i < _cmdLen; i++) {
        cmd_sum += _pbuff[i];
    }
    return ~cmd_sum;
}

void onI2CReceive(int numBytes) {
    if (numBytes > 0) {
        i2cRegisterPointer = Wire.read();
        while (Wire.available()) {
            Wire.read();
        }
    }
}

void onI2CRequest() {
    // Buffer layout (5 bytes):
    // [0] lidar_high, [1] lidar_low, [2] us_high, [3] us_low, [4] status
    uint8_t buffer[5];
    buffer[0] = (lidarDistance_mm >> 8) & 0xFF;
    buffer[1] = lidarDistance_mm & 0xFF;
    buffer[2] = (ultrasoundDistance_cm >> 8) & 0xFF;
    buffer[3] = ultrasoundDistance_cm & 0xFF;
    buffer[4] = 0xAA; // Normal status flag

    if (i2cRegisterPointer < 5) {
        int bytesToSend = 5 - i2cRegisterPointer;
        Wire.write((const uint8_t*)&buffer[i2cRegisterPointer], bytesToSend);
    } else {
        Wire.write(0x00);
    }
}

void initCAN() {
    canBus.begin(CAN_TX_PIN, CAN_RX_PIN);
}

void sendSensorData(uint32_t id, const uint8_t* data, uint8_t dlc) {
    canBus.transmitRaw(id, data, dlc);
}

void setup() {
    // Serial communication completely removed to avoid conflict on GPIO20/21

    // Initialize TSD20 Lidar Serial at 460800 bps with expanded buffer (From altimeter.txt)
    Serial1.setRxBufferSize(2048); 
    Serial1.begin(460800, SERIAL_8N1, TSD_RX, TSD_TX);

    // Initialize URM37v5.0 Pins (Wait-HIGH, From altimeter.txt)
    pinMode(URM_TRIG_PIN, OUTPUT);
    pinMode(URM_ECHO_PIN, INPUT);
    digitalWrite(URM_TRIG_PIN, HIGH);

    // Initialize I2C Slave at 0x30
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    // Initialize CAN
    initCAN();
}

void loop() {
    // 1. Read and parse Serial stream from TSD20 Lidar (Robust parser, From altimeter.txt)
    bool lidarFresh = false;
    while (Serial1.available() > 0) {
        uint8_t inByte = Serial1.read();
        
        if (!packetStarted) {
            if (inByte == 0x5C) {
                rxBuffer[0] = inByte;
                bufferIndex = 1;
                packetStarted = true;
            }
        } else {
            rxBuffer[bufferIndex++] = inByte;
            
            if (bufferIndex >= 4) {
                packetStarted = false; 
                
                uint8_t checksum = calculateChecksum(&rxBuffer[1], 2);
                if (checksum == rxBuffer[3]) {
                    uint16_t rawDistance = (uint16_t)((rxBuffer[2] << 8) | rxBuffer[1]);
                    if (rawDistance != 50000) {
                        int correctedDistance = (int)rawDistance + OFFSET_CORRECTION;
                        lidarDistance_mm = (correctedDistance < 0) ? 0 : (uint16_t)correctedDistance;
                        lidarFresh = true;
                    } else {
                        lidarDistance_mm = 65535; // out-of-range
                    }
                }
            }
        }
    }

    // Keep track of Lidar packet timeout. If no packet for 2 seconds, mark as invalid.
    static uint32_t lastLidarRxTime = 0;
    if (lidarFresh) {
        lastLidarRxTime = millis();
    }
    if (millis() - lastLidarRxTime > 2000) {
        lidarDistance_mm = 65535;
    }

    // 2. Measure Ultrasonic distance and broadcast both metrics (500ms cycle)
    static uint32_t lastMeasurement = 0;
    if (millis() - lastMeasurement >= 500) {
        lastMeasurement = millis();

        // URM37v5.0 trigger pulse (Wait-HIGH, From altimeter.txt)
        digitalWrite(URM_TRIG_PIN, LOW);
        delayMicroseconds(50);  // URM37v5.0 spec: trigger pulse width must be at least 50us
        digitalWrite(URM_TRIG_PIN, HIGH);

        long duration = pulseIn(URM_ECHO_PIN, LOW, 18000); // Timeout reduced to 18ms (~3m range) to reduce blocking
        if (duration > 0 && duration < 18000) {
            float urm_mm = duration * 0.172f; // URM37v5.0 spec: 1us = 0.172mm
            uint16_t cm = (uint16_t)(urm_mm / 10.0f);
            ultrasoundDistance_cm = cm;
        } else {
            ultrasoundDistance_cm = 65535; // out of range -> 65535
        }

        // Broadcast Lidar range via CAN ID 0x100
        uint8_t lidarData[2];
        lidarData[0] = (lidarDistance_mm >> 8) & 0xFF;
        lidarData[1] = lidarDistance_mm & 0xFF;
        sendSensorData(LIDAR_CAN_ID, lidarData, 2);

        // Broadcast Ultrasonic range via CAN ID 0x101
        uint8_t ultraData[2];
        ultraData[0] = (ultrasoundDistance_cm >> 8) & 0xFF;
        ultraData[1] = ultrasoundDistance_cm & 0xFF;
        sendSensorData(ULTRASONIC_CAN_ID, ultraData, 2);
    }

    // 1Hz CAN Heartbeat
    static uint32_t lastHBalt = 0;
    if (millis() - lastHBalt >= 1000) {
        lastHBalt = millis();
        uint8_t hbPayload = NODE_ID_ALT;
        canBus.transmitRaw(CAN_ID_HB_ALT, &hbPayload, 1);
    }

    delay(1); // Yield CPU to FreeRTOS scheduler to prevent WDT reset loop & stabilize CAN/I2C
}
