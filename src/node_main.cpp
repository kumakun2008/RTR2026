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

#define I2C_SDA_PIN 22
#define I2C_SCL_PIN 21
#define SPI_SCK_PIN  18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
#define SD_CS_PIN    4
#define CAN_TX_PIN   32
#define CAN_RX_PIN   33 
// Note: CAN_STB (MCP2561 pin 8) is hardware-grounded - no software control needed
#define BATTERY_ADC_PIN 36

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 100000); // Lowered from 400kHz to 100kHz for longer extension line stability
SDLogger sdLogger;
CANManager canBus;

ICM42688Sensor mainIMU(i2cBus, 0x69);
BM1422Sensor mainMag(i2cBus, 0x0E);
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
    float sdpTemp32 = 0.0f;
    float sdpTemp31_1 = 0.0f;
    float sdpTemp31_2 = 0.0f;
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
// lastRxXxx = last time ANY sensor data frame from that node was received
// lastHbXxx = last time the node's heartbeat frame (0x0Fx) was received
volatile uint32_t lastRxMain   = 0;
volatile uint32_t lastRxPitot  = 0;
volatile uint32_t lastRxRudder = 0;
volatile uint32_t lastRxGPS    = 0;
volatile uint32_t lastRxAlt    = 0;
volatile uint32_t lastRxBridge = 0;
volatile uint32_t lastRxElevator = 0;

volatile uint32_t lastHbMain     = 0;
volatile uint32_t lastHbPitot    = 0;
volatile uint32_t lastHbRudder   = 0;
volatile uint32_t lastHbGPS      = 0;
volatile uint32_t lastHbAlt      = 0;
volatile uint32_t lastHbElevator = 0;
volatile uint32_t lastHbSpeaker  = 0;

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
    Serial.printf("[INFO] I2C configured: SDA=%d, SCL=%d, Speed=400kHz\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.printf("[INFO] SPI configured: SCK=%d, MISO=%d, MOSI=%d, CS=%d\n", SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
    Serial.printf("[INFO] CAN configured: TX=%d, RX=%d\n", CAN_TX_PIN, CAN_RX_PIN);

    Serial.println("Initializing I2C bus...");
    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
        Serial.println("Forcing I2C bus recovery to clear any power-on locks...");
        i2cBus.recoverBus();
    } else {
        Serial.println("[ERROR] I2C Manager Failed to start!");
    }

    Serial.println("Initializing SPI & SD Logger...");
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    if (sdLogger.begin(SD_CS_PIN, SPI, 8000000)) { // Reduced from 20MHz to 8MHz for better noise margin
        Serial.print("[OK] SD Logger Mounted. File: ");
        Serial.println(sdLogger.getActiveFilename());
    } else {
        Serial.println("[ERROR] SD Logger Initialization Failed! (Check card or SPI configuration)");
    }

    Serial.println("Initializing CAN bus...");
    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    } else {
        Serial.println("[ERROR] CAN Bus Driver Failed!");
    }

    Serial.println("Initializing Main IMU (ICM42688)...");
    if (mainIMU.begin()) {
        Serial.println("[OK] Main IMU (ICM42688) Initialized");
    } else {
        Serial.println("[ERROR] Main IMU (ICM42688) Initialization Failed!");
    }

    Serial.println("Initializing Main Magnetometer (BM1422)...");
    if (mainMag.begin()) {
        Serial.println("[OK] Main Magnetometer (BM1422) Initialized");
    } else {
        Serial.println("[ERROR] Main Magnetometer (BM1422) Initialization Failed!");
    }

    Serial.println("Initializing Main Barometer (LPS22)...");
    if (mainBaro.begin()) {
        Serial.println("[OK] Main Barometer (LPS22) Initialized");
    } else {
        Serial.println("[ERROR] Main Barometer (LPS22) Initialization Failed!");
    }

    Serial.println("Initializing Battery monitor...");
    battery.begin();
    
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
        taskYIELD(); // Fix #3: yield after IMU burst to allow other nodes bus access

        mainMag.read(magData);
        sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_X, magData.mag_x, CAN_Scale::MAG);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_Y, magData.mag_y, CAN_Scale::MAG);
        canBus.transmitScaled(CAN_ID_MAIN_MAG_Z, magData.mag_z, CAN_Scale::MAG);
        taskYIELD(); // Fix #3: yield after MAG burst

        // Fallback pitch/roll calculation from local IMU if Pitot board is offline (no CAN packet for 2 seconds)
        if (millis() - lastRxPitot > 2000) {
            float ax = imuData.accel_x;
            float ay = imuData.accel_y;
            float az = imuData.accel_z;
            float pitch_local = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / 3.14159265f;
            float roll_local = atan2(-ax, az) * 180.0f / 3.14159265f;
            flightData.gyro[0] = -roll_local;
            flightData.gyro[1] = pitch_local;
        }

        // Fallback heading calculation from local Magnetometer if GPS board is offline (no CAN packet for 2 seconds)
        if (millis() - lastRxGPS > 2000) {
            float mx = magData.mag_x;
            float my = magData.mag_y;
            float heading_local = atan2(my, mx) * 180.0f / 3.14159265f;
            if (heading_local < 0) heading_local += 360.0f;
            flightData.gpsHeading = (uint16_t)heading_local;
        }

        if (loopCounter % 4 != 0) {
            if (mainBaro.read(baroData)) {
                flightData.baroPress = baroData.pressure;
                flightData.baroTemp = baroData.temperature;
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitScaled(CAN_ID_MAIN_PRESS, baroData.pressure * 100.0f, CAN_Scale::PRESSURE);
                canBus.transmitScaled(CAN_ID_MAIN_TEMP, baroData.temperature, CAN_Scale::TEMP);
                taskYIELD(); // Fix #3: yield after BARO burst
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
            Serial.printf(">main_gps_speed:%.2f\n", flightData.gpsSpeed);
            Serial.printf(">sdp_temp32:%.2f\n", flightData.sdpTemp32);
            Serial.printf(">sdp_temp31_1:%.2f\n", flightData.sdpTemp31_1);
            Serial.printf(">sdp_temp31_2:%.2f\n", flightData.sdpTemp31_2);
        }

        static uint32_t lastCANStatus = 0;
        if (millis() - lastCANStatus >= 2000) {
            lastCANStatus = millis();
            canBus.printStatus();
        }

        loopCounter++;

        static uint32_t lastHBmain = 0;
        if (millis() - lastHBmain >= 1000) {
            lastHBmain = millis();
            uint8_t hbPayload = NODE_ID_MAIN;
            canBus.transmitRaw(CAN_ID_HB_MAIN, &hbPayload, 1);
        }
    }
}

void taskCANReceive(void* pvParameters) {
    uint32_t rxId = 0;
    uint8_t rxData[8];
    uint8_t rxDlc = 0;

    static uint64_t gpsLatRaw = 0;
    static uint64_t gpsLonRaw = 0;
    static bool hasLatUpper = false;
    static bool hasLatLower = false;
    static bool hasLonUpper = false;
    static bool hasLonLower = false;

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

            // Heartbeat frames (0x0F0 range)
            if (rxId == CAN_ID_HB_MAIN)     { lastHbMain     = millis(); }
            else if (rxId == CAN_ID_HB_PITOT)    { lastHbPitot    = millis(); }
            else if (rxId == CAN_ID_HB_RUDDER)   { lastHbRudder   = millis(); }
            else if (rxId == CAN_ID_HB_GPS)      { lastHbGPS      = millis(); }
            else if (rxId == CAN_ID_HB_ALT)      { lastHbAlt      = millis(); }
            else if (rxId == CAN_ID_HB_ELEVATOR) { lastHbElevator = millis(); }
            else if (rxId == CAN_ID_HB_SPEAKER)  { lastHbSpeaker  = millis(); }

            // Sensor data frames — update node-specific lastRx timestamps
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
            else if (rxId == CAN_ID_PITOT_TEMP_RAW_SDP) {
                flightData.sdpTemp32 = getFloat(rxData, CAN_Scale::TEMP);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_TEMP_RAW_SDP31_1) {
                flightData.sdpTemp31_1 = getFloat(rxData, CAN_Scale::TEMP);
                lastRxPitot = millis();
            }
            else if (rxId == CAN_ID_PITOT_TEMP_RAW_SDP31_2) {
                flightData.sdpTemp31_2 = getFloat(rxData, CAN_Scale::TEMP);
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
                // Fix #5: decode uint16_t big-endian (matches updated altimeter node)
                uint16_t rawUS = (uint16_t)((rxData[0] << 8) | rxData[1]);
                flightData.altUS = (float)rawUS / 100.0f;
                flightData.has_altUS = true;
                lastRxAlt = millis();
            }
            else if (rxId == CAN_ID_GPS_LAT_UPPER) {
                uint32_t upper;
                memcpy(&upper, rxData, 4);
                gpsLatRaw = ((uint64_t)upper << 32) | (gpsLatRaw & 0xFFFFFFFF);
                hasLatUpper = true;
            }
            else if (rxId == CAN_ID_GPS_LAT_LOWER) {
                uint32_t lower;
                memcpy(&lower, rxData, 4);
                gpsLatRaw = (gpsLatRaw & 0xFFFFFFFF00000000ULL) | lower;
                hasLatLower = true;
            }
            else if (rxId == CAN_ID_GPS_LON_UPPER) {
                uint32_t upper;
                memcpy(&upper, rxData, 4);
                gpsLonRaw = ((uint64_t)upper << 32) | (gpsLonRaw & 0xFFFFFFFF);
                hasLonUpper = true;
            }
            else if (rxId == CAN_ID_GPS_LON_LOWER) {
                uint32_t lower;
                memcpy(&lower, rxData, 4);
                gpsLonRaw = (gpsLonRaw & 0xFFFFFFFF00000000ULL) | lower;
                hasLonLower = true;
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
            else if (rxId == CAN_ID_GPS_SATS) {
                int32_t sats;
                memcpy(&sats, rxData, 4);
                flightData.gpsSats = (uint8_t)sats;
                lastRxGPS = millis();
            }
            else if (rxId == CAN_ID_GPS_FIX) {
                int32_t fix;
                memcpy(&fix, rxData, 4);
                flightData.gpsFix = (uint8_t)fix;
                lastRxGPS = millis();
            }
            
            if (hasLatUpper && hasLatLower) {
                memcpy(&flightData.gpsLat, &gpsLatRaw, 8);
                hasLatUpper = false;
                hasLatLower = false;
                lastRxGPS = millis();
            }
            if (hasLonUpper && hasLonLower) {
                memcpy(&flightData.gpsLon, &gpsLonRaw, 8);
                hasLonUpper = false;
                hasLonLower = false;
                lastRxGPS = millis();
            }

            // Central Blackbox SD Logging
            if (rxId == CAN_ID_PITOT_AIRSPEED || rxId == CAN_ID_PITOT_TEMP || 
                rxId == CAN_ID_PITOT_TEMP_RAW_SDP || 
                rxId == CAN_ID_PITOT_TEMP_RAW_SDP31_1 || 
                rxId == CAN_ID_PITOT_TEMP_RAW_SDP31_2) {
                PitotPayload pPayload = {
                    flightData.pitotPress32,
                    flightData.pitotPress31_1,
                    flightData.pitotPress31_2,
                    flightData.sdpTemp32,
                    flightData.sdpTemp31_1,
                    flightData.sdpTemp31_2
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
            
            // Format: $TEL2,battery,pressure,altitude,gpsSats,airspeed,pitch,roll,heading,rudder,lat,lon,gpsAlt,
            //         rxMain,rxPitot,rxRudder,rxGPS,rxAlt,rxBridge,rxLadder,rxElev,
            //         hbMain,hbPitot,hbRudder,hbGPS,hbAlt,hbLadder,hbElev,hbSpk,curTime*
            char stateStr[384];
            snprintf(stateStr, sizeof(stateStr),
                     "$TEL2,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%d,%.2f,%.6f,%.6f,%.2f,%d,%d,"
                     "%u,%u,%u,%u,%u,%u,%u,%u,"
                     "%u,%u,%u,%u,%u,%u,%u,%u,%u*",
                     battery.readVoltage(),
                     flightData.baroPress,
                     flightData.altLidar,
                     flightData.gpsSats,
                     flightData.pitotPress32,
                     flightData.gyro[0],
                     flightData.gyro[1],
                     flightData.gpsHeading,
                     flightData.rudderAngle,
                     flightData.gpsLat,
                     flightData.gpsLon,
                     flightData.gpsAlt,
                     flightData.gpsFix,
                     flightData.gpsSats,
                     // データ受信タイムスタンプ (センサーフレーム)
                     millis(),        // rxMain (self)
                     lastRxPitot,
                     lastRxRudder,
                     lastRxGPS,
                     lastRxAlt,
                     lastRxBridge,
                     0U,              // rxLadder (deprecated)
                     lastRxElevator,
                     // ハートビートタイムスタンプ
                     lastHbMain,
                     lastHbPitot,
                     lastHbRudder,
                     lastHbGPS,
                     lastHbAlt,
                     0U,              // hbLadder (deprecated)
                     lastHbElevator,
                     lastHbSpeaker,
                     millis());
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
