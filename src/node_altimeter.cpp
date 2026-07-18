#include <Arduino.h>
#include <Wire.h>
#include "can_protocol.hpp"
#include "can_manager.hpp"

// ======== GPIO Pin Assignments (ESP32-C3) ========
// TSD20 Lidar
#define TSD_RX 6  // D4 (GPIO6)
#define TSD_TX 7  // D5 (GPIO7)
#define OFFSET_CORRECTION 0

// URM37v5.0 Ultrasonic Sensor
#define URM_ECHO_PIN 3  // D1 (GPIO3)
#define URM_TRIG_PIN 4  // D2 (GPIO4)

// CAN GPIO Pins (GPIO20/21 に戻し、シリアル競合を避けるためSerialを完全無効化)
#define CAN_TX_PIN 20
#define CAN_RX_PIN 21

// CAN IDs
#define LIDAR_CAN_ID      0x100
#define ULTRASONIC_CAN_ID 0x101

// I2C Slave Settings
#define I2C_SLAVE_ADDR 0x30
volatile uint8_t i2cRegisterPointer = 0x00;

// Shared data variables (written by sensor tasks, read by I2C/CAN)
volatile uint16_t lidarDistance_mm = 65535;
volatile uint16_t ultrasoundDistance_cm = 65535;

// CAN Manager Instance
CANManager canBus;

// FreeRTOS Task Handles
TaskHandle_t hLidarTask = NULL;
TaskHandle_t hUSonicTask = NULL;
TaskHandle_t hCANTask = NULL;

// TSD20 チェックサム計算
uint8_t calculateChecksum(uint8_t *_pbuff, uint16_t _cmdLen) {
    uint8_t cmd_sum = 0;
    for (uint16_t i = 0; i < _cmdLen; i++) {
        cmd_sum += _pbuff[i];
    }
    return ~cmd_sum;
}

// I2C Slave Callback Functions
void onI2CReceive(int numBytes) {
    if (numBytes > 0) {
        i2cRegisterPointer = Wire.read();
        while (Wire.available()) {
            Wire.read();
        }
    }
}

void onI2CRequest() {
    uint8_t buffer[5];
    buffer[0] = (lidarDistance_mm >> 8) & 0xFF;
    buffer[1] = lidarDistance_mm & 0xFF;
    buffer[2] = (ultrasoundDistance_cm >> 8) & 0xFF;
    buffer[3] = ultrasoundDistance_cm & 0xFF;
    buffer[4] = 0xAA; // Status normal

    if (i2cRegisterPointer < 5) {
        int bytesToSend = 5 - i2cRegisterPointer;
        Wire.write((const uint8_t*)&buffer[i2cRegisterPointer], bytesToSend);
    } else {
        Wire.write(0x00);
    }
}

// --------------------------------------------------------------------------
// Task 1: TSD20 LiDAR Serial Parser Task (Priority: Medium)
// --------------------------------------------------------------------------
void taskLidar(void* pvParameters) {
    uint8_t rxBuf[4];
    uint8_t bufIdx = 0;
    bool pktStarted = false;
    uint32_t lastLidarRxTime = millis();

    while (true) {
        bool processedAny = false;
        while (Serial1.available() > 0) {
            uint8_t inByte = Serial1.read();
            processedAny = true;
            
            if (!pktStarted) {
                if (inByte == 0x5C) {
                    rxBuf[0] = inByte;
                    bufIdx = 1;
                    pktStarted = true;
                }
            } else {
                rxBuf[bufIdx++] = inByte;
                
                if (bufIdx >= 4) {
                    pktStarted = false; 
                    
                    uint8_t checksum = calculateChecksum(&rxBuf[1], 2);
                    if (checksum == rxBuf[3]) {
                        uint16_t rawDistance = (uint16_t)((rxBuf[2] << 8) | rxBuf[1]);
                        if (rawDistance != 50000) {
                            int correctedDistance = (int)rawDistance + OFFSET_CORRECTION;
                            lidarDistance_mm = (correctedDistance < 0) ? 0 : (uint16_t)correctedDistance;
                            lastLidarRxTime = millis();
                        } else {
                            lidarDistance_mm = 65535; // out-of-range
                        }
                    }
                }
            }
        }

        // Lidar timeout check (2 seconds)
        if (millis() - lastLidarRxTime > 2000) {
            lidarDistance_mm = 65535;
        }

        // If no bytes were available, sleep for 2ms to yield CPU
        if (!processedAny) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

// --------------------------------------------------------------------------
// Task 2: URM37 Ultrasonic Measurement Task (Priority: Low)
// --------------------------------------------------------------------------
void taskUltrasonic(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Run every 500ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

        // 1. Wait for ECHO pin to go HIGH (standby state) to avoid immediate timeout
        uint32_t waitStart = micros();
        bool ready = true;
        while (digitalRead(URM_ECHO_PIN) == LOW) {
            if (micros() - waitStart > 10000) { // 10ms max wait
                ready = false;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1)); // let other tasks run while waiting
        }

        if (!ready) {
            ultrasoundDistance_cm = 65535;
            continue;
        }

        // 2. Generate Trigger Pulse (Wait-HIGH, LOW active)
        digitalWrite(URM_TRIG_PIN, LOW);
        delayMicroseconds(50); // URM37 spec: triggers on >=50us LOW
        digitalWrite(URM_TRIG_PIN, HIGH);

        // 3. Measure Echo Pulse duration (Active-LOW, 18ms max timeout = ~3m)
        long duration = pulseIn(URM_ECHO_PIN, LOW, 18000); 
        if (duration > 0 && duration < 18000) {
            float urm_mm = duration * 0.172f; // URM37v5.0 spec: 1us = 0.172mm
            uint16_t cm = (uint16_t)(urm_mm / 10.0f);
            ultrasoundDistance_cm = cm;
        } else {
            ultrasoundDistance_cm = 65535; // out-of-range
        }
    }
}

// --------------------------------------------------------------------------
// Task 3: CAN Transmission & Heartbeat Task (Priority: High)
// --------------------------------------------------------------------------
void taskCANTransmit(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t lastHBTime = 0;

    while (true) {
        // Run every 500ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

        // 1. Broadcast LiDAR data
        uint8_t lidarPayload[2];
        lidarPayload[0] = (lidarDistance_mm >> 8) & 0xFF;
        lidarPayload[1] = lidarDistance_mm & 0xFF;
        canBus.transmitRaw(LIDAR_CAN_ID, lidarPayload, 2);

        // 2. Broadcast Ultrasonic data
        uint8_t ultraPayload[2];
        ultraPayload[0] = (ultrasoundDistance_cm >> 8) & 0xFF;
        ultraPayload[1] = ultrasoundDistance_cm & 0xFF;
        canBus.transmitRaw(ULTRASONIC_CAN_ID, ultraPayload, 2);

        // 3. 1Hz Heartbeat
        uint32_t now = millis();
        if (now - lastHBTime >= 1000) {
            lastHBTime = now;
            uint8_t hbPayload = NODE_ID_ALT;
            canBus.transmitRaw(CAN_ID_HB_ALT, &hbPayload, 1);
        }
    }
}

void setup() {
    // Initialize TSD20 Lidar Serial at 460800 bps with expanded buffer (From altimeter.txt)
    Serial1.setRxBufferSize(2048); 
    Serial1.begin(460800, SERIAL_8N1, TSD_RX, TSD_TX);

    // Initialize URM37v5.0 Pins
    pinMode(URM_TRIG_PIN, OUTPUT);
    pinMode(URM_ECHO_PIN, INPUT_PULLUP); // Use pull-up for open-collector/noise immunity
    digitalWrite(URM_TRIG_PIN, HIGH);

    // Initialize I2C Slave at 0x30
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    // Initialize CAN (TWAI)
    canBus.begin(CAN_TX_PIN, CAN_RX_PIN);

    // Create FreeRTOS tasks (specifically optimized for single-core ESP32-C3)
    // taskCANTransmit has high priority, taskLidar medium, taskUltrasonic low.
    xTaskCreate(taskCANTransmit, "CAN_TX", 2048, NULL, 4, &hCANTask);
    xTaskCreate(taskLidar,       "LIDAR",  2048, NULL, 3, &hLidarTask);
    xTaskCreate(taskUltrasonic,  "USONIC", 2048, NULL, 2, &hUSonicTask);
}

void loop() {
    // Empty, all processing is handled by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
