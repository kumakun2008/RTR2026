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
#include <WiFi.h>
#include <ArduinoOTA.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_SCK_PIN  18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SD_CS_PIN    4
#define CAN_TX_PIN   32
#define CAN_RX_PIN   33 
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_PPS_PIN  34
#define BATTERY_ADC_PIN 36

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
SDLogger sdLogger;
CANManager canBus;

ICM42688Sensor mainIMU(i2cBus, 0x68);
BM1422Sensor mainMag(i2cBus, 0x0F);
LPS22Sensor mainBaro(i2cBus, 0x5C);
TimeSync timeSync;
Telemetry telemetry;
BatteryMonitor battery(BATTERY_ADC_PIN);

struct DisplayTelemetry {
    float accel[3] = {0.0f};
    float gyro[3] = {0.0f};
    float mag[3] = {0.0f};
    float baroPress = 0.0f;
    float baroTemp = 0.0f;
    float pitotPress32 = 0.0f;
    float pitotPress31_1 = 0.0f;
    float pitotPress31_2 = 0.0f;
    float pitotTemp32 = 0.0f;
    float batteryVolt = 0.0f;
    double gpsLat = 0.0;
    double gpsLon = 0.0;
    float gpsAlt = 0.0f;
    float gpsSpeed = 0.0f;
    uint8_t gpsSats = 0;
    uint8_t gpsFix = 0;
    uint16_t gpsHeading = 0;
    float altLidar = 0.0f;
    float altUS = 0.0f;
    float rudderAngle = 0.0f;
} flightData;

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

void taskSensorAcquisition(void* pvParameters);
void taskCANReceive(void* pvParameters);
void taskSDSync(void* pvParameters);
void taskTelemetry(void* pvParameters);
void taskBatteryVoltage(void* pvParameters);
void onCalibZeroCommandTriggered();
void onOTAModeTriggered();
void enterOTAMode();

void enterOTAMode() {
    isOtaMode = true;
    esp_task_wdt_delete(NULL);
    Serial.println("Entering multi-node OTA Update Mode...");
    ArduinoOTA.setHostname("RTR_Main_Board");
    ArduinoOTA.onStart([]() { Serial.println("[OTA] Download starting..."); });
    ArduinoOTA.onEnd([]() { Serial.println("[OTA] Completed. Restarting node..."); ESP.restart(); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("[OTA] Error[%u]. Rebooting...\n", error); ESP.restart(); });
    ArduinoOTA.begin();
    otaTimeoutStart = millis();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- RTR2026 Avionics Initialization ---");
    Serial.println("Running: RTR_Main_Board Configuration");

    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    if (sdLogger.begin(SD_CS_PIN, SPI, 20000000)) {
        Serial.print("[OK] SD Logger Mounted. File: ");
        Serial.println(sdLogger.getActiveFilename());
    }

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

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

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

void taskSensorAcquisition(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t loopCounter = 0;
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        uint64_t timestamp = timeSync.getAbsoluteTimeUs();

        IMUPayload imuData;
        if (mainIMU.read(imuData)) {
            sdLogger.logPacket(LOG_ID_MAIN_IMU, &imuData, sizeof(imuData), timestamp);
            float pitch = atan2(-imuData.accel_x, sqrt(imuData.accel_y * imuData.accel_y + imuData.accel_z * imuData.accel_z)) * 180.0f / PI;
            float roll = atan2(imuData.accel_y, imuData.accel_z) * 180.0f / PI;
            canBus.transmitAttitude(pitch, roll);
        }
        MagPayload magData;
        if (mainMag.read(magData)) {
            sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
        }
        if (loopCounter % 4 != 0) {
            BaroPayload baroData;
            if (mainBaro.read(baroData)) {
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitAltitude(baroData.pressure * 100.0f, 0.0f, false);
            }
        }
        loopCounter++;
    }
}

void taskCANReceive(void* pvParameters) {
    uint32_t rxId = 0;
    uint8_t rxData[8];
    uint8_t rxDlc = 0;

    while (true) {
        if (isOtaMode) {
            enterOTAMode();
            while (true) {
                ArduinoOTA.handle();
                if (millis() - otaTimeoutStart > 300000) {
                    Serial.println("[OTA] Timeout. Rebooting...");
                    delay(1000);
                    ESP.restart();
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }

        if (canBus.receiveRaw(rxId, rxData, rxDlc, 20)) {
            if (rxId == CAN_ID_OTA_START) {
                isOtaMode = true;
                continue;
            }
            uint64_t timestamp = timeSync.getAbsoluteTimeUs();
            
            if (rxId == CAN_ID_AIRSPEED) {
                memcpy(&flightData.pitotPress32, rxData, 4);
                memcpy(&flightData.gpsSpeed, rxData + 4, 4);
                
                PitotPayload pPayload = {
                    flightData.pitotPress32,
                    flightData.pitotPress31_1,
                    flightData.pitotPress31_2,
                    flightData.pitotTemp32
                };
                sdLogger.logPacket(LOG_ID_PITOT_DATA, &pPayload, sizeof(pPayload), timestamp);
            }
            else if (rxId == CAN_ID_AOA_AOS) {
                memcpy(&flightData.pitotPress31_1, rxData, 4);
                memcpy(&flightData.pitotPress31_2, rxData + 4, 4);
            }
            else if (rxId == CAN_ID_GPS_POS) {
                int32_t latVal, lonVal;
                memcpy(&latVal, rxData, 4);
                memcpy(&lonVal, rxData + 4, 4);
                flightData.gpsLat = (double)latVal / CAN_Scale::GPS_DEG;
                flightData.gpsLon = (double)lonVal / CAN_Scale::GPS_DEG;
                
                GPSPayload gPayload = {
                    flightData.gpsLat,
                    flightData.gpsLon,
                    0.0f,
                    flightData.gpsSpeed,
                    flightData.gpsSats,
                    flightData.gpsFix,
                    flightData.gpsHeading
                };
                sdLogger.logPacket(LOG_ID_GPS, &gPayload, sizeof(gPayload), timestamp);
            }
            else if (rxId == CAN_ID_ALTITUDE) {
                memcpy(&flightData.altLidar, rxData, 4);
                memcpy(&flightData.altUS, rxData + 4, 4);
                
                AltimeterPayload aPayload = {
                    flightData.altUS,
                    flightData.altLidar
                };
                sdLogger.logPacket(LOG_ID_ALTIMETER, &aPayload, sizeof(aPayload), timestamp);
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE) {
                memcpy(&flightData.rudderAngle, rxData, 4);
                
                RudderPayload rPayload = {
                    flightData.rudderAngle,
                    0.0f
                };
                sdLogger.logPacket(LOG_ID_RUDDER, &rPayload, sizeof(rPayload), timestamp);
            }
        }
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

void taskBatteryVoltage(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        float cellVoltage = battery.readVoltage();
        uint64_t timestamp = timeSync.getAbsoluteTimeUs();
        BatteryPayload batPayload = { cellVoltage, 0.0f };
        sdLogger.logPacket(LOG_ID_BATTERY, &batPayload, sizeof(batPayload), timestamp);
        canBus.transmitBattery(cellVoltage);
        
        if (cellVoltage < 7.0f) {
            canBus.transmitVoiceCmd(ALERT_LOW_BATTERY);
        }
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
            snprintf(stateStr, sizeof(stateStr), 
                     "$TEL,%.2f,%.2f,%.2f,%d*", 
                     battery.readVoltage(), 
                     flightData.baroPress,
                     flightData.altLidar,
                     flightData.gpsSats);
            telemetry.sendText(stateStr);
        }
        esp_task_wdt_reset();
    }
}

void onCalibZeroCommandTriggered() {
    canBus.transmitCalibZero();
    canBus.transmitVoiceCmd(ALERT_CALIB_START);
}

void onOTAModeTriggered() {
    canBus.transmitOtaStart();
    delay(100);
    esp_task_wdt_delete(NULL);
}
