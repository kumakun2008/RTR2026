/**
 * @file main.cpp
 * @brief Glider avionics main orchestration file with cross-platform and multi-node configurations.
 * Supports ESP32 FreeRTOS core pinning and STM32 standard Arduino polling.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include <Arduino.h>

#ifdef ESP32
#include <esp_task_wdt.h>
#endif

#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "sensor_manager.hpp"
#include "sd_logger.hpp"
#include "can_manager.hpp"
#include "time_sync.hpp"
#include "telemetry.hpp"
#include "battery.hpp"

// Default compile profile if none specified
#if !defined(NODE_MAIN) && !defined(NODE_PITOT) && !defined(NODE_DISPLAY) && !defined(NODE_GPS) && !defined(NODE_SPEAKER) && !defined(NODE_ALTIMETER) && !defined(NODE_RUDDER)
#define NODE_MAIN
#endif

// ==========================================
// PIN CONFIGURATIONS
// ==========================================
#ifdef ESP32
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_SCK_PIN  18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SD_CS_PIN    5
#define CAN_TX_PIN   4
#define CAN_RX_PIN   14 
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_PPS_PIN  34
#define BATTERY_ADC_PIN 36
#define BUTTON_PAGE_PIN 12 
#else
// STM32 specific pins (adjust based on Bluepill pinout)
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define CAN_TX_PIN   PA12
#define CAN_RX_PIN   PA11
#endif

// ==========================================
// SYSTEM OBJECTS
// ==========================================
I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
SDLogger sdLogger;
CANManager canBus;

// ==========================================
// NODE SPECIFIC OBJECTS & STATE VARIABLES
// ==========================================

#if defined(NODE_MAIN)
ICM42688Sensor mainIMU(i2cBus, 0x68);
BM1422Sensor mainMag(i2cBus, 0x0F);
LPS22Sensor mainBaro(i2cBus, 0x5C);
TimeSync timeSync;
Telemetry telemetry;
BatteryMonitor battery(BATTERY_ADC_PIN);

#elif defined(NODE_PITOT)
SDP3xSensor pitotSDP32(i2cBus, 0x21, true);   
SDP3xSensor pitotSDP31_1(i2cBus, 0x22, false); 
SDP3xSensor pitotSDP31_2(i2cBus, 0x23, false); 
BNO055Sensor pitotIMU(i2cBus, 0x28);
SHT41Sensor pitotSHT(i2cBus, 0x44);

volatile bool isCalibrating = false;
float calibBufferSDP32[300];
float calibBufferSDP31_1[300];
float calibBufferSDP31_2[300];
int calibSampleCount = 0;

#elif defined(NODE_DISPLAY)
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

struct DisplayTelemetry {
    float accel[3] = {0};
    float gyro[3] = {0};
    float baroPress = 0;
    float baroTemp = 0;
    float pitotPress32 = 0;
    float batteryVolt = 0;
} flightData;

volatile int displayPage = 0;

#elif defined(NODE_GPS)
// Dedicated GPS Node
TimeSync gpsSync;

#elif defined(NODE_SPEAKER)
// Dedicated Audio Synthesizer Speaker Node
char audioMsgBuffer[64] = {0};
volatile bool audioPending = false;

#elif defined(NODE_ALTIMETER)
// Dedicated Altimeter Node (Ultrasonic + LiDAR)
float rangeLiDAR = 0.0f;
float rangeUltrasonic = 0.0f;

#elif defined(NODE_RUDDER)
// Rudder board (STM32F303)
ICM42688Sensor rudderIMU(i2cBus, 0x68);
// Mock AS5600 logic (AS5600 is at 0x36)
float rawRudderAngle = 0.0f;
#endif

// ==========================================
// FREERTOS TASKS DECLARATIONS (ESP32 ONLY)
// ==========================================
#ifdef ESP32
void taskSensorAcquisition(void* pvParameters);
void taskCANReceive(void* pvParameters);
void taskSDSync(void* pvParameters);

#if defined(NODE_MAIN)
void taskTelemetry(void* pvParameters);
void taskBatteryVoltage(void* pvParameters);
void onCalibZeroCommandTriggered();
void onOTAModeTriggered();
#endif

#if defined(NODE_DISPLAY)
void taskUIDraw(void* pvParameters);
#endif
#endif

// ==========================================
// SYSTEM SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- RTR2026 Avionics Initialization ---");

    // Initialize I2C Bus
    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    // Initialize SD Logger (Only nodes with SD slots: Main and Pitot)
#if defined(NODE_MAIN) || defined(NODE_PITOT)
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    if (sdLogger.begin(SD_CS_PIN, SPI, 20000000)) {
        Serial.print("[OK] SD Logger Mounted. File: ");
        Serial.println(sdLogger.getActiveFilename());
    }
#endif

    // Initialize CAN Driver
    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

    // Initialize Hardware & FreeRTOS Tasks for each node
#if defined(NODE_MAIN)
    Serial.println("Running: RTR_Main_Board Configuration");
    mainIMU.begin();
    mainMag.begin();
    mainBaro.begin();
    battery.begin();
    timeSync.begin(Serial2, GPS_PPS_PIN, GPS_RX_PIN, GPS_TX_PIN);
    
    telemetry.registerCalibCallback(onCalibZeroCommandTriggered);
    telemetry.registerOTACallback(onOTAModeTriggered);
    telemetry.begin("RTR_Main_Avionics");

    esp_task_wdt_init(3, true);
    xTaskCreatePinnedToCore(taskSensorAcquisition, "SensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskBatteryVoltage, "BatteryTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(taskSDSync, "SDSyncTask", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(taskTelemetry, "TelemetryTask", 4096, NULL, 1, NULL, 0);

#elif defined(NODE_PITOT)
    Serial.println("Running: RTR_Pitot_Board Configuration");
    pitotSDP32.begin();
    pitotSDP31_1.begin();
    pitotSDP31_2.begin();
    pitotIMU.begin();
    pitotSHT.begin();

    esp_task_wdt_init(3, true);
    xTaskCreatePinnedToCore(taskSensorAcquisition, "SensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskSDSync, "SDSyncTask", 2048, NULL, 3, NULL, 0);

#elif defined(NODE_DISPLAY)
    Serial.println("Running: RTR_Display_Board Configuration");
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.println("RTR2026 Display Active");

    pinMode(BUTTON_PAGE_PIN, INPUT_PULLUP);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskUIDraw, "UIDrawTask", 4096, NULL, 1, NULL, 0);

#elif defined(NODE_GPS)
    Serial.println("Running: RTR_GPS_Board Configuration");
    gpsSync.begin(Serial2, GPS_PPS_PIN, GPS_RX_PIN, GPS_TX_PIN);
    xTaskCreatePinnedToCore(taskSensorAcquisition, "GPSTask", 3072, NULL, 4, NULL, 1);

#elif defined(NODE_SPEAKER)
    Serial.println("Running: RTR_Speaker_Board Configuration");
    // Connect voice synth module on Serial2
    Serial2.begin(9600, SERIAL_8N1, 16, 17); 
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

#elif defined(NODE_ALTIMETER)
    Serial.println("Running: RTR_Altimeter_Board Configuration");
    // ESP32-C3 uses single-threaded or standard FreeRTOS pins
    xTaskCreate(taskSensorAcquisition, "AltimeterTask", 3072, NULL, 4, NULL);

#elif defined(NODE_RUDDER)
    Serial.println("Running: RTR_Rudder_Board Configuration (STM32)");
    rudderIMU.begin();
#endif

    Serial.println("--- Initialization Complete ---");
}

// ==========================================
// EXECUTION LOOPS
// ==========================================

#ifdef ESP32
void loop() {
    vTaskDelay(portMAX_DELAY); // Handled by FreeRTOS scheduler
}
#else
// Regular loop for non-ESP32 platforms (RTR_Rudder_Board on STM32)
void loop() {
#if defined(NODE_RUDDER)
    static uint32_t lastRead = 0;
    if (millis() - lastRead >= 10) { // 100Hz Rate
        lastRead = millis();
        
        // 1. Read ICM-42688 Gyro Z
        IMUPayload imuData;
        float gyroZ = 0.0f;
        if (rudderIMU.read(imuData)) {
            gyroZ = imuData.gyro_z;
        }
        
        // 2. Read AS5600 mock logic (Register read at 0x0E for Angle)
        uint8_t angleBytes[2];
        if (i2cBus.readRegister(0x36, 0x0E, angleBytes, 2)) {
            uint16_t rawAngle = (angleBytes[0] << 8) | angleBytes[1];
            rawRudderAngle = (float)rawAngle * (360.0f / 4096.0f);
        }

        // 3. Transmit scaled values on CAN
        int16_t rawAngleScale = (int16_t)(rawRudderAngle * CAN_Scale::ANGLE);
        int16_t rawGyroZScale = (int16_t)(gyroZ * CAN_Scale::GYRO);
        
        uint8_t canPayload[4];
        canPayload[0] = (uint8_t)(rawAngleScale >> 8);
        canPayload[1] = (uint8_t)(rawAngleScale & 0xFF);
        canPayload[2] = (uint8_t)(rawGyroZScale >> 8);
        canPayload[3] = (uint8_t)(rawGyroZScale & 0xFF);
        
        canBus.transmitRaw(CAN_MSG_RUDDER_ANGLE, canPayload, 4);
    }
#endif
}
#endif

// ==========================================
// FREERTOS TASKS IMPLEMENTATIONS (ESP32 ONLY)
// ==========================================
#ifdef ESP32

void taskSensorAcquisition(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t loopCounter = 0;

    while (true) {
#if defined(NODE_ALTIMETER)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz for altimeter
#elif defined(NODE_GPS)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz for GPS
#else
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz standard
#endif

        uint64_t timestamp = esp_timer_get_time();

#if defined(NODE_MAIN)
        timestamp = timeSync.getAbsoluteTimeUs();
        IMUPayload imuData;
        if (mainIMU.read(imuData)) {
            sdLogger.logPacket(LOG_ID_MAIN_IMU, &imuData, sizeof(imuData), timestamp);
            canBus.transmitAccel(imuData.accel_x, imuData.accel_y, imuData.accel_z);
            canBus.transmitGyro(imuData.gyro_x, imuData.gyro_y, imuData.gyro_z);
        }
        MagPayload magData;
        if (mainMag.read(magData)) {
            sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
            canBus.transmitMag(magData.mag_x, magData.mag_y, magData.mag_z);
        }
        if (loopCounter % 4 != 0) {
            BaroPayload baroData;
            if (mainBaro.read(baroData)) {
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitBaro(baroData.pressure * 100.0f, baroData.temperature);
            }
        }

#elif defined(NODE_PITOT)
        float press32 = 0, temp32 = 0;
        float press31_1 = 0, temp31_1 = 0;
        float press31_2 = 0, temp31_2 = 0;
        pitotSDP32.read(press32, temp32);
        pitotSDP31_1.read(press31_1, temp31_1);
        pitotSDP31_2.read(press31_2, temp31_2);

        if (isCalibrating && calibSampleCount < 300) {
            calibBufferSDP32[calibSampleCount] = press32 + pitotSDP32.getCalibrationOffset();
            calibBufferSDP31_1[calibSampleCount] = press31_1 + pitotSDP31_1.getCalibrationOffset();
            calibBufferSDP31_2[calibSampleCount] = press31_2 + pitotSDP31_2.getCalibrationOffset();
            calibSampleCount++;
        }

        PitotPayload pitotData = { press32, press31_1, press31_2, temp32 };
        sdLogger.logPacket(LOG_ID_PITOT_DATA, &pitotData, sizeof(pitotData), timestamp);
        
        int32_t raw32 = (int32_t)(press32 * CAN_Scale::PRESS);
        int32_t raw31_1 = (int32_t)(press31_1 * CAN_Scale::PRESS);
        uint8_t pData[8];
        memcpy(pData, &raw32, 4);
        memcpy(pData + 4, &raw31_1, 4);
        canBus.transmitRaw(CAN_MSG_PITOT_PRESSURES, pData, 8);

        float roll = 0, pitch = 0, yaw = 0;
        if (pitotIMU.read(roll, pitch, yaw)) {
            int16_t r = (int16_t)(roll * CAN_Scale::ANGLE);
            int16_t p = (int16_t)(pitch * CAN_Scale::ANGLE);
            int16_t y = (int16_t)(yaw * CAN_Scale::ANGLE);
            uint8_t data[6] = { (uint8_t)(r >> 8), (uint8_t)(r & 0xFF), (uint8_t)(p >> 8), (uint8_t)(p & 0xFF), (uint8_t)(y >> 8), (uint8_t)(y & 0xFF) };
            canBus.transmitRaw(CAN_MSG_PITOT_IMU, data, 6);
        }

#elif defined(NODE_GPS)
        // Read GPS UTC Time and Position, broadcast on CAN
        if (gpsSync.isSynced()) {
            uint64_t utcUs = gpsSync.getAbsoluteTimeUs();
            uint8_t timePayload[8];
            memcpy(timePayload, &utcUs, 8);
            canBus.transmitRaw(CAN_MSG_GPS_TIME, timePayload, 8);
            
            // Mock latitude and longitude (example values)
            int32_t lat = (int32_t)(35.6812f * CAN_Scale::GPS_DEG);
            int32_t lon = (int32_t)(139.7671f * CAN_Scale::GPS_DEG);
            uint8_t posPayload[8];
            memcpy(posPayload, &lat, 4);
            memcpy(posPayload + 4, &lon, 4);
            canBus.transmitRaw(CAN_MSG_GPS_POS_LAT_LON, posPayload, 8);
        }

#elif defined(NODE_ALTIMETER)
        // 50Hz Altimeter read logic
        // Read ultrasonic distance (mock analog read converted to mm)
        rangeUltrasonic = (analogRead(0) / 4095.0f) * 5000.0f; // 0-5m range
        
        // Read LiDAR distance (mock single point LiDAR)
        uint8_t lidarBytes[4];
        if (i2cBus.readRegister(0x10, 0x00, lidarBytes, 2)) { // Assume I2C address 0x10
            rangeLiDAR = (float)((lidarBytes[0] << 8) | lidarBytes[1]);
        } else {
            rangeLiDAR = 1200.0f; // placeholder mm
        }

        uint16_t rawUS = (uint16_t)rangeUltrasonic;
        uint16_t rawLiDAR = (uint16_t)rangeLiDAR;
        uint8_t data[4] = { (uint8_t)(rawUS >> 8), (uint8_t)(rawUS & 0xFF), (uint8_t)(rawLiDAR >> 8), (uint8_t)(rawLiDAR & 0xFF) };
        canBus.transmitRaw(CAN_MSG_ALTIMETER_DATA, data, 4);
#endif

        loopCounter++;
    }
}

void taskCANReceive(void* pvParameters) {
    uint32_t rxId = 0;
    uint8_t rxData[8];
    uint8_t rxDlc = 0;

    while (true) {
        if (canBus.receiveRaw(rxId, rxData, rxDlc, 20)) {
#if defined(NODE_MAIN)
            uint64_t timestamp = timeSync.getAbsoluteTimeUs();
            if (rxId == CAN_MSG_PITOT_PRESSURES) {
                int32_t press32;
                memcpy(&press32, rxData, 4);
                float p32 = (float)press32 / CAN_Scale::PRESS;
                sdLogger.logPacket(LOG_ID_PITOT_DATA, &p32, sizeof(p32), timestamp);
            }

#elif defined(NODE_PITOT)
            if (rxId == CAN_MSG_CMD_CALIB_ZERO) {
                calibSampleCount = 0;
                isCalibrating = true;
            }

#elif defined(NODE_DISPLAY)
            if (rxId == CAN_MSG_MAIN_IMU_ACCEL) {
                int16_t rx = (int16_t)((rxData[0] << 8) | rxData[1]);
                flightData.accel[0] = (float)rx / CAN_Scale::ACCEL;
            } else if (rxId == CAN_MSG_MAIN_BATTERY) {
                uint16_t volt = (uint16_t)((rxData[0] << 8) | rxData[1]);
                flightData.batteryVolt = (float)volt / CAN_Scale::VOLT;
            }

#elif defined(NODE_SPEAKER)
            // Receive text message triggers from main board
            if (rxId == CAN_MSG_CMD_CALIB_ZERO) {
                snprintf(audioMsgBuffer, sizeof(audioMsgBuffer), "ゼロ点調整を開始します。");
                audioPending = true;
            }
#endif
        }

#if defined(NODE_PITOT)
        if (isCalibrating && calibSampleCount >= 300) {
            isCalibrating = false;
            float finalOffset32 = Telemetry::calculateCalibZeroIQR(calibBufferSDP32, 300);
            float finalOffset31_1 = Telemetry::calculateCalibZeroIQR(calibBufferSDP31_1, 300);
            float finalOffset31_2 = Telemetry::calculateCalibZeroIQR(calibBufferSDP31_2, 300);
            pitotSDP32.setCalibrationOffset(finalOffset32);
            pitotSDP31_1.setCalibrationOffset(finalOffset31_1);
            pitotSDP31_2.setCalibrationOffset(finalOffset31_2);
            
            uint64_t ts = esp_timer_get_time();
            float loggedOffsets[3] = { finalOffset32, finalOffset31_1, finalOffset31_2 };
            sdLogger.logPacket(LOG_ID_EVENT_MARK, loggedOffsets, sizeof(loggedOffsets), ts);
        }
#endif

#if defined(NODE_SPEAKER)
        if (audioPending) {
            audioPending = false;
            // Output text string to speech synthesis chip on Serial2
            Serial2.print(audioMsgBuffer); 
            Serial.printf("[SPEAKER] Speaking: %s\n", audioMsgBuffer);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskSDSync(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
        sdLogger.triggerSync();
        esp_task_wdt_reset();
    }
}

#if defined(NODE_MAIN)
void taskBatteryVoltage(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        float cellVoltage = battery.readVoltage();
        uint64_t timestamp = timeSync.getAbsoluteTimeUs();
        BatteryPayload batPayload = { cellVoltage, 0.0f };
        sdLogger.logPacket(LOG_ID_BATTERY, &batPayload, sizeof(batPayload), timestamp);
        canBus.transmitBattery(cellVoltage, 0.0f);
    }
}

void taskTelemetry(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        if (telemetry.isOTAModeActive()) {
            telemetry.handleOTA();
        } else {
            telemetry.process();
            char stateStr[128];
            snprintf(stateStr, sizeof(stateStr), "$TEL,%.3f,%.2f*", battery.readVoltage(), (float)(timeSync.getAbsoluteTimeUs() / 1000000ULL));
            telemetry.sendText(stateStr);
        }
        esp_task_wdt_reset();
    }
}

void onCalibZeroCommandTriggered() {
    canBus.transmitCalibZero();
}

void onOTAModeTriggered() {
    esp_task_wdt_delete(NULL);
}
#endif

#if defined(NODE_DISPLAY)
void taskUIDraw(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(66));
        if (digitalRead(BUTTON_PAGE_PIN) == LOW) {
            displayPage = (displayPage + 1) % 2;
            tft.fillScreen(ILI9341_BLACK);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        tft.setCursor(0, 0);
        tft.setTextColor(ILI9341_WHITE);
        tft.printf("V_Bat : %.2f V\n", flightData.batteryVolt);
        tft.printf("AccelX: %.2f g\n", flightData.accel[0]);
    }
}
#endif

#endif // ESP32
