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
    
    // Status flags
    bool has_altLidar = false;
    bool has_altUS = false;
} flightData;

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

// CAN node communication timestamps
volatile uint32_t lastRxMain = 0;
volatile uint32_t lastRxPitot = 0;
volatile uint32_t lastRxRudder = 0;
volatile uint32_t lastRxGPS = 0;
volatile uint32_t lastRxAlt = 0;
volatile uint32_t lastRxBridge = 0;

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
    
    IMUPayload imuData = {0};
    MagPayload magData = {0};
    BaroPayload baroData = {0};

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        uint64_t timestamp = timeSync.getAbsoluteTimeUs();

        mainIMU.read(imuData);
        sdLogger.logPacket(LOG_ID_MAIN_IMU, &imuData, sizeof(imuData), timestamp);
        canBus.transmitScaled(CAN_ID_MAIN_ACC_X, imuData.accel_x, CAN_Scale::ACCEL);
        canBus.transmitScaled(CAN_ID_MAIN_ACC_Y, imuData.accel_y, CAN_Scale::ACCEL);
        canBus.transmitScaled(CAN_ID_MAIN_ACC_Z, imuData.accel_z, CAN_Scale::ACCEL);
        canBus.transmitScaled(CAN_ID_MAIN_GYRO_X, imuData.gyro_x, CAN_Scale::GYRO);
        canBus.transmitScaled(CAN_ID_MAIN_GYRO_Y, imuData.gyro_y, CAN_Scale::GYRO);
        canBus.transmitScaled(CAN_ID_MAIN_GYRO_Z, imuData.gyro_z, CAN_Scale::GYRO);

        mainMag.read(magData);
        sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_X, magData.mag_x, CAN_Scale::MAG);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_Y, magData.mag_y, CAN_Scale::MAG);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_Z, magData.mag_z, CAN_Scale::MAG);

        if (loopCounter % 4 != 0) {
            if (mainBaro.read(baroData)) {
                flightData.baroPress = baroData.pressure;
                flightData.baroTemp = baroData.temperature;
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitScaled(CAN_ID_MAIN_PRESS, baroData.pressure * 100.0f, CAN_Scale::PRESSURE);
                canBus.transmitScaled(CAN_ID_MAIN_TEMP, baroData.temperature, CAN_Scale::TEMP);
            }
        }

        // Teleplot Output (10Hz)
        static uint32_t lastPlot = 0;
        if (millis() - lastPlot >= 100) {
            lastPlot = millis();
            Serial.printf(">main_acc_x:%.3f\n", imuData.accel_x);
            Serial.printf(">main_acc_y:%.3f\n", imuData.accel_y);
            Serial.printf(">main_acc_z:%.3f\n", imuData.accel_z);
            Serial.printf(">main_gyro_x:%.3f\n", imuData.gyro_x);
            Serial.printf(">main_gyro_y:%.3f\n", imuData.gyro_y);
            Serial.printf(">main_gyro_z:%.3f\n", imuData.gyro_z);
            Serial.printf(">main_mag_x:%.3f\n", magData.mag_x);
            Serial.printf(">main_mag_y:%.3f\n", magData.mag_y);
            Serial.printf(">main_mag_z:%.3f\n", magData.mag_z);
            Serial.printf(">main_press:%.2f\n", baroData.pressure * 100.0f);
            Serial.printf(">main_temp:%.2f\n", baroData.temperature);
            Serial.printf(">main_bat:%.2f\n", battery.readVoltage());
            Serial.printf(">gps_speed:%.2f\n", flightData.gpsSpeed);
        }

        loopCounter++;
    }
}

void taskCANReceive(void* pvParameters) {
    uint32_t rxId = 0;
    uint8_t rxData[8];
    uint8_t rxDlc = 0;

    static uint64_t gpsLatRaw = 0;
    static uint64_t gpsLonRaw = 0;
    static bool updateGPSLat = false;
    static bool updateGPSLon = false;

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
            
            auto getFloat = [&](const uint8_t* d, float scale) {
                int32_t raw;
                memcpy(&raw, d, 4);
                return (float)raw / scale;
            };

            lastRxMain = millis();

            if (rxId == CAN_ID_PITOT_AIRSPEED) {
                flightData.pitotPress32 = getFloat(rxData, CAN_Scale::GPS_SPEED);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_AOA) {
                flightData.pitotPress31_1 = getFloat(rxData, CAN_Scale::PRESSURE);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_AOS) {
                flightData.pitotPress31_2 = getFloat(rxData, CAN_Scale::PRESSURE);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_PITCH) {
                flightData.gyro[0] = getFloat(rxData, CAN_Scale::ANGLE);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_ROLL) {
                flightData.gyro[1] = getFloat(rxData, CAN_Scale::ANGLE);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_TEMP) {
                flightData.pitotTemp32 = getFloat(rxData, CAN_Scale::TEMP);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE) {
                flightData.rudderAngle = getFloat(rxData, CAN_Scale::ANGLE);
                lastRxRudder = millis();
            }
            else if (rxId == CAN_ID_ALT_LIDAR) {
                uint16_t rawLidar = (uint16_t)((rxData[0] << 8) | rxData[1]);
                flightData.altLidar = (float)rawLidar / 1000.0f;
                flightData.has_altLidar = true;
                lastRxAlt = millis();
            }
            else if (rxId == CAN_ID_ALT_US) {
                uint8_t rawUS = rxData[0];
                flightData.altUS = (float)rawUS / 100.0f;
                flightData.has_altUS = true;
                lastRxAlt = millis();
            }
            else if (rxId == CAN_ID_GPS_LAT_UPPER) {
                uint32_t upper;
                memcpy(&upper, rxData, 4);
                gpsLatRaw = ((uint64_t)upper << 32) | (gpsLatRaw & 0xFFFFFFFF);
                updateGPSLat = true;
            }
            else if (rxId == CAN_ID_GPS_LAT_LOWER) {
                uint32_t lower;
                memcpy(&lower, rxData, 4);
                gpsLatRaw = (gpsLatRaw & 0xFFFFFFFF00000000ULL) | lower;
                updateGPSLat = true;
            }
            else if (rxId == CAN_ID_GPS_LON_UPPER) {
                uint32_t upper;
                memcpy(&upper, rxData, 4);
                gpsLonRaw = ((uint64_t)upper << 32) | (gpsLonRaw & 0xFFFFFFFF);
                updateGPSLon = true;
            }
            else if (rxId == CAN_ID_GPS_LON_LOWER) {
                uint32_t lower;
                memcpy(&lower, rxData, 4);
                gpsLonRaw = (gpsLonRaw & 0xFFFFFFFF00000000ULL) | lower;
                updateGPSLon = true;
            }
            else if (rxId == CAN_ID_GPS_ALT) {
                flightData.gpsAlt = getFloat(rxData, CAN_Scale::GPS_ALT);
                lastRxGPS = millis();
            }
            else if (rxId == CAN_ID_GPS_SPEED) {
                flightData.gpsSpeed = getFloat(rxData, CAN_Scale::GPS_SPEED);
                lastRxGPS = millis();
            }
            else if (rxId == CAN_ID_GPS_AZIMUTH) {
                flightData.gpsHeading = (uint16_t)(getFloat(rxData, CAN_Scale::GPS_AZIMUTH));
                lastRxGPS = millis();
            }
            
            if (updateGPSLat) {
                memcpy(&flightData.gpsLat, &gpsLatRaw, 8);
                updateGPSLat = false;
                lastRxGPS = millis();
                flightData.gpsSats = 8;
                flightData.gpsFix = 2;
            }
            if (updateGPSLon) {
                memcpy(&flightData.gpsLon, &gpsLonRaw, 8);
                updateGPSLon = false;
                lastRxGPS = millis();
            }

            // Central Blackbox SD Logging
            if (rxId == CAN_ID_PITOT_AIRSPEED || rxId == CAN_ID_PITOT_TEMP) {
                PitotPayload pPayload = {
                    flightData.pitotPress32,
                    flightData.pitotPress31_1,
                    flightData.pitotPress31_2,
                    flightData.pitotTemp32
                };
                sdLogger.logPacket(LOG_ID_PITOT_DATA, &pPayload, sizeof(pPayload), timestamp);
            }
            else if (rxId == CAN_ID_GPS_LAT_LOWER || rxId == CAN_ID_GPS_LON_LOWER) {
                GPSPayload gPayload = {
                    flightData.gpsLat,
                    flightData.gpsLon,
                    flightData.gpsAlt,
                    flightData.gpsSpeed,
                    flightData.gpsSats,
                    flightData.gpsFix,
                    flightData.gpsHeading
                };
                sdLogger.logPacket(LOG_ID_GPS, &gPayload, sizeof(gPayload), timestamp);
            }
            else if (rxId == CAN_ID_ALT_LIDAR || rxId == CAN_ID_ALT_US) {
                AltimeterPayload aPayload = {
                    flightData.altUS,
                    flightData.altLidar
                };
                sdLogger.logPacket(LOG_ID_ALTIMETER, &aPayload, sizeof(aPayload), timestamp);
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE) {
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
        
        canBus.transmitScaled(CAN_ID_BATTERY_VOLT, cellVoltage, CAN_Scale::BATTERY);
        canBus.transmitInt32(CAN_ID_SD_STATUS, sdLogger.isActive() ? 1 : 0);
        
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
            
            // Format expanded telemetry data to Bluetooth Android app
            // Format: $TEL,battery,pressure,altitude,gpsSats,airspeed,pitch,roll,heading,rudder,lat,lon,gpsAlt,lastRxMain,lastRxPitot,lastRxRudder,lastRxGPS,lastRxAlt,lastRxBridge*
            char stateStr[256];
            snprintf(stateStr, sizeof(stateStr), 
                     "$TEL,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%d,%.2f,%.6f,%.6f,%.2f,%u,%u,%u,%u,%u,%u*", 
                     battery.readVoltage(), 
                     flightData.baroPress,
                     flightData.altLidar,
                     flightData.gpsSats,
                     flightData.pitotPress32, // airspeed
                     flightData.gyro[0],      // pitch
                     flightData.gyro[1],      // roll
                     flightData.gpsHeading,   // heading
                     flightData.rudderAngle,  // rudder angle
                     flightData.gpsLat,       // lat
                     flightData.gpsLon,       // lon
                     flightData.gpsAlt,       // gpsAlt
                     millis(),                // lastRxMain
                     lastRxPitot,
                     lastRxRudder,
                     lastRxGPS,
                     lastRxAlt,
                     lastRxBridge);
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
