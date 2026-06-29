#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "sensor_manager.hpp"
#include "sd_logger.hpp"
#include "can_manager.hpp"
#include "telemetry.hpp"
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

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
SDLogger sdLogger;
CANManager canBus;

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

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

void taskSensorAcquisition(void* pvParameters);
void taskCANReceive(void* pvParameters);
void taskSDSync(void* pvParameters);
void enterOTAMode();

void enterOTAMode() {
    isOtaMode = true;
    esp_task_wdt_delete(NULL);
    Serial.println("Entering multi-node OTA Update Mode...");
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

    ArduinoOTA.setHostname("RTR_Pitot_Board");
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
    Serial.println("Running: RTR_Pitot_Board Configuration");

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

    pitotSDP32.begin();
    pitotSDP31_1.begin();
    pitotSDP31_2.begin();
    pitotIMU.begin();
    pitotSHT.begin();

    esp_task_wdt_init(3, true);
    xTaskCreatePinnedToCore(taskSensorAcquisition, "SensorTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskSDSync, "SDSyncTask", 2048, NULL, 3, NULL, 0);

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
        uint64_t timestamp = esp_timer_get_time();

        float press32 = 0, temp32 = 0;
        float press31_1 = 0, temp31_1 = 0;
        float press31_2 = 0, temp31_2 = 0;
        
        bool ok32 = pitotSDP32.read(press32, temp32);
        bool ok31_1 = pitotSDP31_1.read(press31_1, temp31_1);
        bool ok31_2 = pitotSDP31_2.read(press31_2, temp31_2);

        if (isCalibrating && calibSampleCount < 300) {
            calibBufferSDP32[calibSampleCount] = press32 + pitotSDP32.getCalibrationOffset();
            calibBufferSDP31_1[calibSampleCount] = press31_1 + pitotSDP31_1.getCalibrationOffset();
            calibBufferSDP31_2[calibSampleCount] = press31_2 + pitotSDP31_2.getCalibrationOffset();
            calibSampleCount++;
        }

        if (ok32 && ok31_1 && ok31_2) {
            PitotPayload pitotData = { press32, press31_1, press31_2, temp32 };
            sdLogger.logPacket(LOG_ID_PITOT_DATA, &pitotData, sizeof(pitotData), timestamp);
            
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
            if (rxId == CAN_ID_CALIB_ZERO) {
                calibSampleCount = 0;
                isCalibrating = true;
            }
        }

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
            
            canBus.transmitVoiceCmd(ALERT_CALIB_END);
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
