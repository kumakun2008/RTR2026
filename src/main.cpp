/**
 * @file main.cpp
 * @brief Glider avionics main orchestration file with cross-platform and multi-node configurations.
 * Contains thorough implementations for Main, Pitot, Speaker, Display, GPS, Altimeter, and Rudder boards.
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
#define SD_CS_PIN    4
#define CAN_TX_PIN   32
#define CAN_RX_PIN   33 
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_PPS_PIN  34
#define BATTERY_ADC_PIN 36
#define BUTTON_PAGE_PIN 12 

// Altimeter Ultrasonic Sensor Pinouts
#define ULTRASONIC_TRIG_PIN 26
#define ULTRASONIC_ECHO_PIN 27

// Node-Specific Pin Overrides
#if defined(NODE_DISPLAY)
#undef CAN_TX_PIN
#undef CAN_RX_PIN
#define CAN_TX_PIN 25
#define CAN_RX_PIN 26
#define TFT_CS   5
#define TFT_DC   17
#define TFT_RST  16
#undef SPI_SCK_PIN
#undef SPI_MISO_PIN
#undef SPI_MOSI_PIN
#undef SD_CS_PIN
#define SPI_SCK_PIN 14
#define SPI_MISO_PIN 33
#define SPI_MOSI_PIN 13
#define SD_CS_PIN 15
#elif defined(NODE_SPEAKER)
#undef CAN_TX_PIN
#undef CAN_RX_PIN
#define CAN_TX_PIN 16
#define CAN_RX_PIN 17
#endif

#else
// STM32 specific pins (Bluepill configuration)
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

// Telemetry cache compiled from CAN (accessible globally across all tasks)
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
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

volatile int displayPage = 0;

#elif defined(NODE_GPS)
TimeSync gpsSync;

#elif defined(NODE_SPEAKER)
// Voice synthesizer pending alerts
volatile bool alertLowBattery = false;
volatile bool alertLowAltitude = false;
volatile bool alertCalibStart = false;
volatile bool alertCalibEnd = false;

#elif defined(NODE_ALTIMETER)
float rangeLiDAR = 0.0f;       // Meters
float rangeUltrasonic = 0.0f;  // Meters

#elif defined(NODE_RUDDER)
ICM42688Sensor rudderIMU(i2cBus, 0x68);
float rawRudderAngle = 0.0f;   // Degrees
#endif

// ==========================================
// FREERTOS TASKS DECLARATIONS (ESP32 ONLY)
// ==========================================
#ifdef ESP32
#include <WiFi.h>
#include <ArduinoOTA.h>

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

// Multi-node OTA mode variables & configuration helper
volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

void enterOTAMode() {
    isOtaMode = true;
    
    // Shut down task watchdog to prevent bootloops during download
    esp_task_wdt_delete(NULL);
    
    Serial.println("Entering multi-node OTA Update Mode...");
    
#if defined(NODE_MAIN)
    // Main board handles Wi-Fi AP generation via Telemetry module
#else
    // Connect to the Main Board's Wi-Fi Access Point
    WiFi.mode(WIFI_STA);
    WiFi.begin("RTR_Glider_OTA_AP", "rtr2026glider");
    
    Serial.print("Connecting to Glider OTA Network");
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[OK] Connected to OTA AP");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n[ERR] Connection timed out. Rebooting...");
        delay(1000);
        ESP.restart();
    }
#endif

    // Setup node-specific hostname for PlatformIO detection
#if defined(NODE_MAIN)
    ArduinoOTA.setHostname("RTR_Main_Board");
#elif defined(NODE_PITOT)
    ArduinoOTA.setHostname("RTR_Pitot_Board");
#elif defined(NODE_DISPLAY)
    ArduinoOTA.setHostname("RTR_Display_Board");
#elif defined(NODE_GPS)
    ArduinoOTA.setHostname("RTR_GPS_Board");
#elif defined(NODE_SPEAKER)
    ArduinoOTA.setHostname("RTR_Speaker_Board");
#elif defined(NODE_ALTIMETER)
    ArduinoOTA.setHostname("RTR_Altimeter_Board");
#endif

    ArduinoOTA.onStart([]() { Serial.println("[OTA] Download starting..."); });
    ArduinoOTA.onEnd([]() { Serial.println("[OTA] Completed. Restarting node..."); ESP.restart(); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("[OTA] Error[%u]. Rebooting...\n", error); ESP.restart(); });
    ArduinoOTA.begin();
    
    otaTimeoutStart = millis();
}
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

    // Initialize SPI and SD Logger (Only nodes with SD slots: Main and Pitot)
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
    xTaskCreatePinnedToCore(taskSensorAcquisition, "GPSTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

#elif defined(NODE_SPEAKER)
    Serial.println("Running: RTR_Speaker_Board Configuration");
    // Connect voice synth module on Serial2 (AquesTalk Pico LSI at 9600 bps: RX=IO33, TX=IO32)
    Serial2.begin(9600, SERIAL_8N1, 33, 32); 
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

#elif defined(NODE_ALTIMETER)
    Serial.println("Running: RTR_Altimeter_Board Configuration (ESP32-C3)");
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    // ESP32-C3 runs altimeter sensors task
    xTaskCreate(taskSensorAcquisition, "AltimeterTask", 3072, NULL, 5, NULL);
    xTaskCreate(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL);

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
        
        // 1. Read ICM-42688 to calculate pitch/roll
        IMUPayload imuData;
        float pitch = 0.0f;
        float roll = 0.0f;
        if (rudderIMU.read(imuData)) {
            pitch = atan2(-imuData.accel_x, sqrt(imuData.accel_y * imuData.accel_y + imuData.accel_z * imuData.accel_z)) * 180.0f / PI;
            roll = atan2(imuData.accel_y, imuData.accel_z) * 180.0f / PI;
            canBus.transmitAttitude(pitch, roll);
        }
        
        // 2. Read AS5600 I2C Angle Sensor (Address 0x36)
        // AS5600 Angle registers: 0x0E (high byte), 0x0F (low byte)
        uint8_t angleBytes[2];
        if (i2cBus.readRegister(0x36, 0x0E, angleBytes, 2)) {
            uint16_t rawAngle = (uint16_t)((angleBytes[0] << 8) | angleBytes[1]);
            // Convert to degrees (12-bit resolution: 0 - 4095)
            rawRudderAngle = (float)rawAngle * (360.0f / 4096.0f);
        }

        // 3. Transmit scaled values on CAN
        canBus.transmitRudderAngle(rawRudderAngle);
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
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz (20ms) for Altimeter
#elif defined(NODE_GPS)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz (100ms) for GPS
#else
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz (10ms) standard
#endif

        uint64_t timestamp = esp_timer_get_time();

#if defined(NODE_MAIN)
        timestamp = timeSync.getAbsoluteTimeUs();
        IMUPayload imuData;
        if (mainIMU.read(imuData)) {
            sdLogger.logPacket(LOG_ID_MAIN_IMU, &imuData, sizeof(imuData), timestamp);
            
            // Calculate Pitch & Roll from Accel
            float pitch = atan2(-imuData.accel_x, sqrt(imuData.accel_y * imuData.accel_y + imuData.accel_z * imuData.accel_z)) * 180.0f / PI;
            float roll = atan2(imuData.accel_y, imuData.accel_z) * 180.0f / PI;
            canBus.transmitAttitude(pitch, roll);
        }
        MagPayload magData;
        if (mainMag.read(magData)) {
            sdLogger.logPacket(LOG_ID_MAIN_MAG, &magData, sizeof(magData), timestamp);
        }
        if (loopCounter % 4 != 0) { // 75Hz roughly polls 3 out of 4 cycles at 100Hz
            BaroPayload baroData;
            if (mainBaro.read(baroData)) {
                sdLogger.logPacket(LOG_ID_MAIN_BARO, &baroData, sizeof(baroData), timestamp);
                canBus.transmitAltitude(baroData.pressure * 100.0f, 0.0f, false);
            }
        }

#elif defined(NODE_PITOT)
        float press32 = 0, temp32 = 0;
        float press31_1 = 0, temp31_1 = 0;
        float press31_2 = 0, temp31_2 = 0;
        
        bool ok32 = pitotSDP32.read(press32, temp32);
        bool ok31_1 = pitotSDP31_1.read(press31_1, temp31_1);
        bool ok31_2 = pitotSDP31_2.read(press31_2, temp31_2);

        // Under calibration mode, buffer raw pressure offset adjustments
        if (isCalibrating && calibSampleCount < 300) {
            calibBufferSDP32[calibSampleCount] = press32 + pitotSDP32.getCalibrationOffset();
            calibBufferSDP31_1[calibSampleCount] = press31_1 + pitotSDP31_1.getCalibrationOffset();
            calibBufferSDP31_2[calibSampleCount] = press31_2 + pitotSDP31_2.getCalibrationOffset();
            calibSampleCount++;
        }

        if (ok32 && ok31_1 && ok31_2) {
            PitotPayload pitotData = { press32, press31_1, press31_2, temp32 };
            sdLogger.logPacket(LOG_ID_PITOT_DATA, &pitotData, sizeof(pitotData), timestamp);
            
            // Broadcast dynamic pressure and calculated airspeed on CAN
            float airspeed = (press32 > 0.0f) ? sqrt(2.0f * press32 / 1.225f) : 0.0f;
            canBus.transmitAirspeed(press32, airspeed);
            canBus.transmitAoaAos(press31_1, press31_2);
        }

        float roll = 0, pitch = 0, yaw = 0;
        if (pitotIMU.read(roll, pitch, yaw)) {
            // Read Pitot IMU (locally logged or processed if needed)
        }

        float hum = 0, tempSHT = 0;
        if (pitotSHT.read(tempSHT, hum)) {
            // Read Pitot SHT Env (locally logged or processed if needed)
        }

#elif defined(NODE_GPS)
        // Read GPS UTC Time and Position from actual stream parser
        GPSPayload gpsData;
        if (gpsSync.getGPSData(gpsData)) {
            // Transmit scaled Lat/Lon coordinates on CAN
            canBus.transmitGPSPos(gpsData.latitude, gpsData.longitude);
        }

#elif defined(NODE_ALTIMETER)
        // 1. Read Ultrasonic Sensor (HC-SR04 Trigger / Echo)
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        
        // Measure high pulse width with 15ms timeout (approx 2.5m range)
        long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 15000); 
        if (duration > 0) {
            // Speed of sound = 343 m/s. Range = duration * 343 / 2 / 10^6
            rangeUltrasonic = (float)duration * 0.0001715f; 
        } else {
            rangeUltrasonic = 0.0f; // Timeout/error
        }
        
        // 2. Read Single-Point LiDAR (TFmini I2C Address 0x10)
        // Standard TFmini command to trigger read: write 5 bytes request
        uint8_t cmd[5] = {0x5A, 0x05, 0x07, 0x01, 0x67};
        if (i2cBus.writeRaw(0x10, cmd, 5)) {
            delayMicroseconds(200);
            uint8_t rawLiDARBytes[9];
            if (i2cBus.readRaw(0x10, rawLiDARBytes, 9)) {
                // Verify TFmini header bytes (0x59, 0x59)
                if (rawLiDARBytes[0] == 0x59 && rawLiDARBytes[1] == 0x59) {
                    uint16_t distCm = (uint16_t)(rawLiDARBytes[2] | (rawLiDARBytes[3] << 8));
                    rangeLiDAR = (float)distCm / 100.0f; // Convert cm to meters
                }
            }
        }

        // 3. Broadcast Altimeter values on CAN
        canBus.transmitAltitude(rangeLiDAR, rangeUltrasonic, true);
#endif

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
                if (millis() - otaTimeoutStart > 300000) { // 5 minutes
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
#if defined(NODE_MAIN)
            // Central Blackbox SD Logging
            uint64_t timestamp = timeSync.getAbsoluteTimeUs();
            
            if (rxId == CAN_ID_AIRSPEED) {
                memcpy(&flightData.pitotPress32, rxData, 4);
                memcpy(&flightData.gpsSpeed, rxData + 4, 4);
                
                // Construct and log Pitot data
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
                
                // Construct and log GPS data (mocking/retaining struct for binary compatibility)
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
                
                // Construct and log Altimeter data
                AltimeterPayload aPayload = {
                    flightData.altUS,
                    flightData.altLidar
                };
                sdLogger.logPacket(LOG_ID_ALTIMETER, &aPayload, sizeof(aPayload), timestamp);
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE) {
                memcpy(&flightData.rudderAngle, rxData, 4);
                
                // Construct and log Rudder data
                RudderPayload rPayload = {
                    flightData.rudderAngle,
                    0.0f
                };
                sdLogger.logPacket(LOG_ID_RUDDER, &rPayload, sizeof(rPayload), timestamp);
            }

#elif defined(NODE_PITOT)
            if (rxId == CAN_ID_CALIB_ZERO) {
                calibSampleCount = 0;
                isCalibrating = true;
            }

#elif defined(NODE_DISPLAY)
            // Parse CAN frames into local flightData cache
            if (rxId == CAN_ID_ATTITUDE && rxDlc >= 8) {
                memcpy(&flightData.gyro[0], rxData, 4); // Pitch
                memcpy(&flightData.gyro[1], rxData + 4, 4); // Roll
            }
            else if (rxId == CAN_ID_AIRSPEED && rxDlc >= 8) {
                memcpy(&flightData.pitotPress32, rxData, 4);
                memcpy(&flightData.gpsSpeed, rxData + 4, 4); // Airspeed
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE && rxDlc >= 4) {
                memcpy(&flightData.rudderAngle, rxData, 4);
            }
            else if (rxId == CAN_ID_ALTITUDE && rxDlc >= 8) {
                float val1, val2;
                memcpy(&val1, rxData, 4);
                memcpy(&val2, rxData + 4, 4);
                if (val2 > 0.001f) {
                    flightData.altLidar = val1;
                    flightData.altUS = val2;
                } else {
                    flightData.baroPress = val1 / 100.0f; // Pa to hPa
                }
            }
            else if (rxId == CAN_ID_GPS_POS && rxDlc >= 8) {
                int32_t latVal, lonVal;
                memcpy(&latVal, rxData, 4);
                memcpy(&lonVal, rxData + 4, 4);
                flightData.gpsLat = (double)latVal / CAN_Scale::GPS_DEG;
                flightData.gpsLon = (double)lonVal / CAN_Scale::GPS_DEG;
                flightData.gpsSats = 8; // Simulated satellite status when receiving positions
                flightData.gpsFix = 2;
            }
            else if (rxId == CAN_ID_BATTERY_VOLT && rxDlc >= 4) {
                memcpy(&flightData.batteryVolt, rxData, 4);
            }

#elif defined(NODE_SPEAKER)
            // Parse alerts based on voice command packets
            if (rxId == CAN_ID_VOICE_CMD && rxDlc >= 1) {
                uint8_t alertCode = rxData[0];
                if (alertCode == ALERT_CALIB_START) {
                    alertCalibStart = true;
                }
                else if (alertCode == ALERT_CALIB_END) {
                    alertCalibEnd = true;
                }
                else if (alertCode == ALERT_LOW_BATTERY) {
                    alertLowBattery = true;
                }
                else if (alertCode == ALERT_LOW_ALTITUDE) {
                    alertLowAltitude = true;
                }
            }
            else if (rxId == CAN_ID_ALTITUDE && rxDlc >= 8) {
                float currentAlt;
                memcpy(&currentAlt, rxData, 4); // LiDAR is first float in Altimeter payload
                if (currentAlt > 0.1f && currentAlt < 5.0f) {
                    alertLowAltitude = true;
                }
            }
#endif
        }

#if defined(NODE_PITOT)
        // Perform calibration zero parsing
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
            
            // Broadcast calibration complete voice alert frame
            canBus.transmitVoiceCmd(ALERT_CALIB_END);
        }
#endif

#if defined(NODE_SPEAKER)
        // Process alerts and send phonetic codes to synthesized voice module (AquesTalk)
        if (alertCalibStart) {
            alertCalibStart = false;
            Serial2.print("cho'useiwo/ka'ishishimasu.\r"); // "調整を開始します"
        }
        else if (alertCalibEnd) {
            alertCalibEnd = false;
            Serial2.print("cho'useiga/kanryo'ushimashita.\r"); // "調整が完了しました"
        }
        else if (alertLowBattery) {
            alertLowBattery = false;
            Serial2.print("den'atsuga/te'ika/shiteimasu.\r"); // "電圧が低下しています"
        }
        else if (alertLowAltitude) {
            alertLowAltitude = false;
            Serial2.print("ko'udoga/te'ika/shiteimasu.\r"); // "高度が低下しています"
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
            
            // Format telemetry dashboard string to Bluetooth Android app (Section 7-4)
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
    // Broadcast OTA start command to all other ESP32 nodes via CAN
    canBus.transmitOtaStart();
    delay(100);
    esp_task_wdt_delete(NULL);
}
#endif

#if defined(NODE_DISPLAY)
void taskUIDraw(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(66)); // 15Hz
        
        if (digitalRead(BUTTON_PAGE_PIN) == LOW) {
            displayPage = (displayPage + 1) % 3; // 3 unique visual dashboards
            tft.fillScreen(ILI9341_BLACK);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        
        if (displayPage == 0) {
            tft.setTextColor(ILI9341_GREEN);
            tft.println("=== RTR DASHBOARD ===");
            tft.setTextColor(ILI9341_WHITE);
            // Calculate airspeed from pitot SDP32 pressure
            float pressPa = flightData.pitotPress32;
            float airspeed = (pressPa > 0.0f) ? sqrt(2.0f * pressPa / 1.225f) : 0.0f;
            
            tft.printf("Battery : %.2f V\n", flightData.batteryVolt);
            tft.printf("Airspeed: %.2f m/s\n", airspeed);
            tft.printf("LidarAlt: %.2f m\n", flightData.altLidar);
            tft.printf("US Alt  : %.2f m\n", flightData.altUS);
            tft.printf("GPS Sat : %d (Fix:%d)\n", flightData.gpsSats, flightData.gpsFix);
        } 
        else if (displayPage == 1) {
            tft.setTextColor(ILI9341_CYAN);
            tft.println("=== MOTION STATUS ===");
            tft.setTextColor(ILI9341_WHITE);
            tft.printf("Pitch : %+5.1f deg\n", flightData.gyro[0]);
            tft.printf("Roll  : %+5.1f deg\n", flightData.gyro[1]);
            tft.printf("Rudder: %+5.2f deg\n", flightData.rudderAngle);
        }
        else {
            tft.setTextColor(ILI9341_YELLOW);
            tft.println("=== GPS POSITION ===");
            tft.setTextColor(ILI9341_WHITE);
            tft.printf("Lat : %.6f deg\n", flightData.gpsLat);
            tft.printf("Lon : %.6f deg\n", flightData.gpsLon);
            tft.printf("Alt : %.2f m\n", flightData.gpsAlt);
            tft.printf("Temp: %.2f C\n", flightData.baroTemp);
            tft.printf("Press: %.2f hPa\n", flightData.baroPress);
        }
    }
}
#endif

#endif // ESP32
