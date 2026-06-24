/**
 * @file main.cpp
 * @brief Glider avionics main orchestration file with FreeRTOS task pinning.
 * @author Team ЯTR
 * @date 2026-06-24
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "sensor_manager.hpp"
#include "sd_logger.hpp"
#include "can_manager.hpp"
#include "time_sync.hpp"
#include "telemetry.hpp"
#include "battery.hpp"

// Default to Main Board if no node is defined in compiler flags
#if !defined(NODE_MAIN) && !defined(NODE_PITOT) && !defined(NODE_DISPLAY)
#define NODE_MAIN
#endif

// ==========================================
// PIN CONFIGURATIONS
// ==========================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define SPI_SCK_PIN  18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23

#define SD_CS_PIN    5
#define CAN_TX_PIN   4
#define CAN_RX_PIN   14 // Shifted to 14 to avoid conflict on some boards

#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_PPS_PIN  34

#define BATTERY_ADC_PIN 36

#define BUTTON_PAGE_PIN 12 // Display board page button

// ==========================================
// SYSTEM OBJECTS
// ==========================================
I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
SDLogger sdLogger;
CANManager canBus;

#if defined(NODE_MAIN)
// Main Board Specifics
ICM42688Sensor mainIMU(i2cBus, 0x68);
BM1422Sensor mainMag(i2cBus, 0x0F);
LPS22Sensor mainBaro(i2cBus, 0x5C);
TimeSync timeSync;
Telemetry telemetry;
BatteryMonitor battery(BATTERY_ADC_PIN);

#elif defined(NODE_PITOT)
// Pitot Board Specifics
SDP3xSensor pitotSDP32(i2cBus, 0x21, true);   // SDP32 primary
SDP3xSensor pitotSDP31_1(i2cBus, 0x22, false); // SDP31 backup 1
SDP3xSensor pitotSDP31_2(i2cBus, 0x23, false); // SDP31 backup 2
BNO055Sensor pitotIMU(i2cBus, 0x28);
SHT41Sensor pitotSHT(i2cBus, 0x44);

// Calibration Buffering State (Section 5)
volatile bool isCalibrating = false;
float calibBufferSDP32[300];
float calibBufferSDP31_1[300];
float calibBufferSDP31_2[300];
int calibSampleCount = 0;

#elif defined(NODE_DISPLAY)
// Display Board Specifics (ILI9341 hardware SPI)
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// Visual variables
struct GliderTelemetryData {
    float accel[3] = {0};
    float gyro[3] = {0};
    float baroPress = 0;
    float baroTemp = 0;
    float pitotPress32 = 0;
    float batteryVolt = 0;
    double gpsLat = 0;
    double gpsLon = 0;
    float gpsAlt = 0;
    uint8_t gpsSats = 0;
} flightData;

volatile int displayPage = 0;
#endif

// ==========================================
// FREERTOS TASKS DECLARATIONS
// ==========================================
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

// ==========================================
// SETUP ENTRY POINT
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- RTR2026 Avionics Initialization Start ---");

    // Initialize Thread-Safe I2C Bus Manager
    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Bus Manager initialized at 400kHz");
    } else {
        Serial.println("[ERROR] Failed to initialize I2C Bus!");
    }

    // Initialize SPI and SD Logger (Main and Pitot boards have SD)
#if defined(NODE_MAIN) || defined(NODE_PITOT)
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    if (sdLogger.begin(SD_CS_PIN, SPI, 20000000)) {
        Serial.print("[OK] SD Logger active. Target file: ");
        Serial.println(sdLogger.getActiveFilename());
    } else {
        Serial.println("[WARN] SD Card not mounted. Logging inactive.");
    }
#endif

    // Initialize CAN bus transceiver
    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus TWAI driver initialized at 1 Mbps");
    } else {
        Serial.println("[ERROR] Failed to start CAN TWAI driver!");
    }

    // Initialize Node-Specific Devices & Core assignments
#if defined(NODE_MAIN)
    Serial.println("Configuring Node: MAIN_BOARD");
    
    // Core sensor initialization
    if (mainIMU.begin()) Serial.println("[OK] ICM-42688-P IMU detected & active");
    if (mainMag.begin()) Serial.println("[OK] BM1422AGMV Mag detected & active");
    if (mainBaro.begin()) Serial.println("[OK] LPS22 Barometer detected & active");
    
    battery.begin();

    // High-precision clock calibration
    if (timeSync.begin(Serial2, GPS_PPS_PIN, GPS_RX_PIN, GPS_TX_PIN)) {
        Serial.println("[OK] GPS NMEA & PPS synchronization task started");
    }

    // Telemetry and callbacks
    telemetry.registerCalibCallback(onCalibZeroCommandTriggered);
    telemetry.registerOTACallback(onOTAModeTriggered);
    if (telemetry.begin("RTR_Main_Avionics")) {
        Serial.println("[OK] Bluetooth SPP Telemetry advertising active");
    }

    // FreeRTOS task mappings for Main Board
    // Task Watchdog Timer (TWDT) configuration for Core 0 tasks
    esp_task_wdt_init(3, true); // 3-second watchdog timeout with hardware reset trigger

    // Core 1 (APP_CPU): Sensor acquisition [HIGH], CAN transceiver processing [MEDIUM], Battery reading [LOW]
    xTaskCreatePinnedToCore(taskSensorAcquisition, "SensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskBatteryVoltage, "BatteryTask", 2048, NULL, 1, NULL, 1);

    // Core 0 (PRO_CPU): SD card batch writing [MEDIUM], Telemetry parsing & OTA [LOW]
    xTaskCreatePinnedToCore(taskSDSync, "SDSyncTask", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(taskTelemetry, "TelemetryTask", 4096, NULL, 1, NULL, 0);

#elif defined(NODE_PITOT)
    Serial.println("Configuring Node: PITOT_BOARD");
    
    // Core sensor initialization
    if (pitotSDP32.begin()) Serial.println("[OK] SDP32 primary Pitot sensor active");
    if (pitotSDP31_1.begin()) Serial.println("[OK] SDP31 Backup 1 Pitot sensor active");
    if (pitotSDP31_2.begin()) Serial.println("[OK] SDP31 Backup 2 Pitot sensor active");
    if (pitotIMU.begin()) Serial.println("[OK] BNO055 Orientation sensor active");
    if (pitotSHT.begin()) Serial.println("[OK] SHT41 Temp/Hum sensor active");

    esp_task_wdt_init(3, true);

    // Core 1 (APP_CPU): Pitot acquisition [HIGH] and CAN monitoring [MEDIUM]
    xTaskCreatePinnedToCore(taskSensorAcquisition, "SensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

    // Core 0 (PRO_CPU): SD card batch writing [MEDIUM]
    xTaskCreatePinnedToCore(taskSDSync, "SDSyncTask", 2048, NULL, 3, NULL, 0);

#elif defined(NODE_DISPLAY)
    Serial.println("Configuring Node: DISPLAY_BOARD");
    
    // Setup display pins
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println("RTR2026 Display Board");
    tft.println("Awaiting CAN Telemetry...");

    pinMode(BUTTON_PAGE_PIN, INPUT_PULLUP);

    // Core 1 (APP_CPU): CAN listener [MEDIUM]
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

    // Core 0 (PRO_CPU): UI Draw Task [LOW, ~15Hz]
    xTaskCreatePinnedToCore(taskUIDraw, "UIDrawTask", 4096, NULL, 1, NULL, 0);
#endif

    Serial.println("--- RTR2026 Avionics Initialization Finished ---");
}

void loop() {
    // FreeRTOS handles task execution; loop is suspended
    vTaskDelay(portMAX_DELAY);
}

// ==========================================
// CORE 1: SENSOR ACQUISITION TASK (100Hz)
// ==========================================
void taskSensorAcquisition(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz (10ms)
    uint32_t loopCounter = 0;

    Serial.println("Sensor Acquisition Task Started on Core 1");

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        uint64_t timestamp = 0;
#if defined(NODE_MAIN)
        timestamp = timeSync.getAbsoluteTimeUs();
#else
        timestamp = esp_timer_get_time(); // fallback local microsecond uptime
#endif

#if defined(NODE_MAIN)
        // 1. Read ICM-42688 IMU
        IMUPayload imuData;
        if (mainIMU.read(imuData)) {
            sdLogger.logPacket(LOG_ID_MAIN_IMU, &imuData, sizeof(imuData), timestamp);
            canBus.transmitAccel(imuData.accel_x, imuData.accel_y, imuData.accel_z);
            canBus.transmitGyro(imuData.gyro_x, imuData.gyro_y, imuData.gyro_z);
        }

        // 2. Read BM1422AGMV Magnetometer
        MagPayload magData;
        if (mainMag.read(magData)) {
            sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
            canBus.transmitMag(magData.mag_x, magData.mag_y, magData.mag_z);
        }

        // 3. Read LPS22 Barometer (75Hz -> sample roughly 3 out of 4 ticks)
        if (loopCounter % 4 != 0) {
            BaroPayload baroData;
            if (mainBaro.read(baroData)) {
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitBaro(baroData.pressure * 100.0f, baroData.temperature); // hPa to Pa conversion
            }
        }

#elif defined(NODE_PITOT)
        // 1. Read Differential Pressures (SDP3x)
        float press32 = 0, temp32 = 0;
        float press31_1 = 0, temp31_1 = 0;
        float press31_2 = 0, temp31_2 = 0;

        bool ok32 = pitotSDP32.read(press32, temp32);
        bool ok31_1 = pitotSDP31_1.read(press31_1, temp31_1);
        bool ok31_2 = pitotSDP31_2.read(press31_2, temp31_2);

        // If calibration mode is active, buffer the raw readings (without offsets)
        if (isCalibrating) {
            if (calibSampleCount < 300) {
                // Read raw by adding back zero offsets to get raw input values
                calibBufferSDP32[calibSampleCount] = press32 + pitotSDP32.getCalibrationOffset();
                calibBufferSDP31_1[calibSampleCount] = press31_1 + pitotSDP31_1.getCalibrationOffset();
                calibBufferSDP31_2[calibSampleCount] = press31_2 + pitotSDP31_2.getCalibrationOffset();
                calibSampleCount++;
            }
        }

        if (ok32 && ok31_1 && ok31_2) {
            PitotPayload pitotData = { press32, press31_1, press31_2, temp32 };
            sdLogger.logPacket(LOG_ID_PITOT_DATA, &pitotData, sizeof(pitotData), timestamp);
            
            // Broadcast pressures on CAN bus: scale is centi-Pascals
            uint8_t data[12];
            int32_t raw32 = (int32_t)(press32 * CAN_Scale::PRESS);
            int32_t raw31_1 = (int32_t)(press31_1 * CAN_Scale::PRESS);
            int32_t raw31_2 = (int32_t)(press31_2 * CAN_Scale::PRESS);
            
            // We split into CAN payload (Note: max 8 bytes, so we split pressures or send in single standard frame)
            // Let's send SDP32 in one packet, and backups in another, or standard layouts
            // Message ID: CAN_MSG_PITOT_PRESSURES (transmits raw32, raw31_1, raw31_2)
            // We fit: SDP32 (4B) and SDP31_1 (4B) in CAN_MSG_PITOT_PRESSURES, and SDP31_2 in another
            uint8_t pData1[8];
            memcpy(pData1, &raw32, 4);
            memcpy(pData1 + 4, &raw31_1, 4);
            canBus.transmitRaw(CAN_MSG_PITOT_PRESSURES, pData1, 8);
        }

        // 2. Read BNO055 IMU
        float roll = 0, pitch = 0, yaw = 0;
        if (pitotIMU.read(roll, pitch, yaw)) {
            int16_t rawRoll = (int16_t)(roll * CAN_Scale::ANGLE);
            int16_t rawPitch = (int16_t)(pitch * CAN_Scale::ANGLE);
            int16_t rawYaw = (int16_t)(yaw * CAN_Scale::ANGLE);
            
            uint8_t data[6];
            data[0] = (uint8_t)(rawRoll >> 8);
            data[1] = (uint8_t)(rawRoll & 0xFF);
            data[2] = (uint8_t)(rawPitch >> 8);
            data[3] = (uint8_t)(rawPitch & 0xFF);
            data[4] = (uint8_t)(rawYaw >> 8);
            data[5] = (uint8_t)(rawYaw & 0xFF);
            canBus.transmitRaw(CAN_MSG_PITOT_IMU, data, 6);
        }

        // 3. Read SHT41 Environment Sensor
        float hum = 0, tempSHT = 0;
        if (pitotSHT.read(tempSHT, hum)) {
            int16_t rawTemp = (int16_t)(tempSHT * CAN_Scale::TEMP);
            uint16_t rawHum = (uint16_t)(hum * CAN_Scale::TEMP);
            
            uint8_t data[4];
            data[0] = (uint8_t)(rawTemp >> 8);
            data[1] = (uint8_t)(rawTemp & 0xFF);
            data[2] = (uint8_t)(rawHum >> 8);
            data[3] = (uint8_t)(rawHum & 0xFF);
            canBus.transmitRaw(CAN_MSG_PITOT_ENV, data, 4);
        }
#endif

        loopCounter++;
    }
}

// ==========================================
// CORE 1: CAN RECEIVE LISTENER TASK
// ==========================================
void taskCANReceive(void* pvParameters) {
    uint32_t rxId = 0;
    uint8_t rxData[8];
    uint8_t rxDlc = 0;

    Serial.println("CAN Receive Listener Task Started on Core 1");

    while (true) {
        if (canBus.receiveRaw(rxId, rxData, rxDlc, 20)) {
            // Process incoming CAN frames based on node type
#if defined(NODE_MAIN)
            // Log incoming remote telemetry nodes into main SD binary log file
            uint64_t timestamp = timeSync.getAbsoluteTimeUs();
            
            if (rxId == CAN_MSG_PITOT_PRESSURES) {
                // SDP32 & SDP31_1 dynamic pressures
                int32_t press32, press31_1;
                memcpy(&press32, rxData, 4);
                memcpy(&press31_1, rxData + 4, 4);
                
                float p32 = (float)press32 / CAN_Scale::PRESS;
                float p31_1 = (float)press31_1 / CAN_Scale::PRESS;
                
                // Pack for logging
                float loggedPressures[2] = { p32, p31_1 };
                sdLogger.logPacket(LOG_ID_PITOT_DATA, loggedPressures, sizeof(loggedPressures), timestamp);
            }

#elif defined(NODE_PITOT)
            // Listen for command requests from the main board (Section 5)
            if (rxId == CAN_MSG_CMD_CALIB_ZERO) {
                Serial.println("[CAN_CMD] Received Zero Calibration Request!");
                calibSampleCount = 0;
                isCalibrating = true;
            }

#elif defined(NODE_DISPLAY)
            // Extract display variables
            if (rxId == CAN_MSG_MAIN_IMU_ACCEL) {
                int16_t rx = (int16_t)((rxData[0] << 8) | rxData[1]);
                int16_t ry = (int16_t)((rxData[2] << 8) | rxData[3]);
                int16_t rz = (int16_t)((rxData[4] << 8) | rxData[5]);
                flightData.accel[0] = (float)rx / CAN_Scale::ACCEL;
                flightData.accel[1] = (float)ry / CAN_Scale::ACCEL;
                flightData.accel[2] = (float)rz / CAN_Scale::ACCEL;
            }
            else if (rxId == CAN_MSG_MAIN_ENV) {
                int32_t press = (int32_t)((rxData[0] << 24) | (rxData[1] << 16) | (rxData[2] << 8) | rxData[3]);
                int16_t temp = (int16_t)((rxData[4] << 8) | rxData[5]);
                flightData.baroPress = (float)press / CAN_Scale::PRESS;
                flightData.baroTemp = (float)temp / CAN_Scale::TEMP;
            }
            else if (rxId == CAN_MSG_MAIN_BATTERY) {
                uint16_t volt = (uint16_t)((rxData[0] << 8) | rxData[1]);
                flightData.batteryVolt = (float)volt / CAN_Scale::VOLT;
            }
            else if (rxId == CAN_MSG_PITOT_PRESSURES) {
                int32_t press32;
                memcpy(&press32, rxData, 4);
                flightData.pitotPress32 = (float)press32 / CAN_Scale::PRESS;
            }
#endif
        }
        
#if defined(NODE_PITOT)
        // Perform calibration calculations if data buffering completes (300 samples)
        if (isCalibrating && calibSampleCount >= 300) {
            isCalibrating = false;
            
            float finalOffset32 = Telemetry::calculateCalibZeroIQR(calibBufferSDP32, 300);
            float finalOffset31_1 = Telemetry::calculateCalibZeroIQR(calibBufferSDP31_1, 300);
            float finalOffset31_2 = Telemetry::calculateCalibZeroIQR(calibBufferSDP31_2, 300);
            
            pitotSDP32.setCalibrationOffset(finalOffset32);
            pitotSDP31_1.setCalibrationOffset(finalOffset31_1);
            pitotSDP31_2.setCalibrationOffset(finalOffset31_2);
            
            Serial.println("--- Pitot Zero Calibration Finished ---");
            Serial.printf("SDP32 Offset  : %.3f Pa\n", finalOffset32);
            Serial.printf("SDP31_1 Offset: %.3f Pa\n", finalOffset31_1);
            Serial.printf("SDP31_2 Offset: %.3f Pa\n", finalOffset31_2);
            
            // Log Event Mark Packet
            uint64_t timestamp = esp_timer_get_time();
            float loggedOffsets[3] = { finalOffset32, finalOffset31_1, finalOffset31_2 };
            sdLogger.logPacket(LOG_ID_EVENT_MARK, loggedOffsets, sizeof(loggedOffsets), timestamp);
        }
#endif
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to core scheduler
    }
}

// ==========================================
// CORE 1: BATTERY VOLTAGE MONITORING TASK (1Hz)
// ==========================================
#if defined(NODE_MAIN)
void taskBatteryVoltage(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1Hz

    Serial.println("Battery Monitoring Task Started on Core 1");

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        float cellVoltage = battery.readVoltage();
        uint64_t timestamp = timeSync.getAbsoluteTimeUs();
        
        // Log to double buffer
        BatteryPayload batPayload = { cellVoltage, 0.0f }; // Only voltage monitored
        sdLogger.logPacket(LOG_ID_BATTERY, &batPayload, sizeof(batPayload), timestamp);
        
        // Broadcast on CAN
        canBus.transmitBattery(cellVoltage, 0.0f);
    }
}
#endif

// ==========================================
// CORE 0: SD CARD BUFFER SYNC TASK (5Hz)
// ==========================================
void taskSDSync(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz (200ms)

    Serial.println("SD Sync Task Started on Core 0");

    // Subscribe current task to Watchdog Timer (TWDT)
    esp_task_wdt_add(NULL);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Swaps the active/inactive log buffers and triggers background DMA write
        sdLogger.triggerSync();
        
        // Feed the task watchdog
        esp_task_wdt_reset();
    }
}

// ==========================================
// CORE 0: BLUETOOTH TELEMETRY TASK (10Hz)
// ==========================================
#if defined(NODE_MAIN)
void taskTelemetry(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz

    Serial.println("Bluetooth SPP Telemetry Task Started on Core 0");

    esp_task_wdt_add(NULL);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Process Bluetooth SPP serial buffers
        if (telemetry.isOTAModeActive()) {
            telemetry.handleOTA();
        } else {
            telemetry.process();
            
            // Print brief state values (Section 7-4 Android Telemetry stream)
            char stateStr[128];
            snprintf(stateStr, sizeof(stateStr), 
                     "$TEL,%.3f,%.3f,%.2f,%.2f*", 
                     battery.readVoltage(), 
                     mainBaro.isInitialized() ? 1013.25f : 0.0f, // Example payload placeholders
                     mainIMU.isInitialized() ? 1.0f : 0.0f,
                     (float)(timeSync.getAbsoluteTimeUs() / 1000000ULL));
            telemetry.sendText(stateStr);
        }
        
        esp_task_wdt_reset();
    }
}

void onCalibZeroCommandTriggered() {
    Serial.println("[SPP_CMD] Zero calibration signal received! Broadcasting to CAN...");
    canBus.transmitCalibZero(); // Send broadcast frame to the Pitot Board node
}

void onOTAModeTriggered() {
    Serial.println("[SPP_CMD] OTA request registered. Entering Wifi SoftAP server mode...");
    // Unsubscribe from TWDT as OTA update loop could block processor timing
    esp_task_wdt_delete(NULL);
}
#endif

// ==========================================
// CORE 0: DISPLAY DRAWING TASK (15Hz)
// ==========================================
#if defined(NODE_DISPLAY)
void taskUIDraw(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(66); // ~15Hz (66ms)
    
    Serial.println("Display UI Update Task Started on Core 0");

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Page Selection Button check
        if (digitalRead(BUTTON_PAGE_PIN) == LOW) {
            displayPage = (displayPage + 1) % 2;
            tft.fillScreen(ILI9341_BLACK);
            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce delay
        }

        // Draw Pages
        tft.setCursor(0, 0);
        if (displayPage == 0) {
            tft.setTextColor(ILI9341_GREEN);
            tft.println("--- RUNTIME TELEMETRY ---");
            tft.setTextColor(ILI9341_WHITE);
            tft.printf("Battery : %.2f V\n\n", flightData.batteryVolt);
            tft.printf("Baro    : %.2f hPa\n", flightData.baroPress);
            tft.printf("Baro Temp: %.1f C\n\n", flightData.baroTemp);
            tft.printf("Pitot   : %.1f Pa\n", flightData.pitotPress32);
        } else {
            tft.setTextColor(ILI9341_CYAN);
            tft.println("--- MOTION TELEMETRY ---");
            tft.setTextColor(ILI9341_WHITE);
            tft.printf("Accel X : %.2f g\n", flightData.accel[0]);
            tft.printf("Accel Y : %.2f g\n", flightData.accel[1]);
            tft.printf("Accel Z : %.2f g\n\n", flightData.accel[2]);
            tft.printf("Gyro X  : %.1f dps\n", flightData.gyro[0]);
            tft.printf("Gyro Y  : %.1f dps\n", flightData.gyro[1]);
            tft.printf("Gyro Z  : %.1f dps\n", flightData.gyro[2]);
        }
    }
}
#endif
