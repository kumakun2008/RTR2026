#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "can_manager.hpp"
#include "time_sync.hpp"
#include <WiFi.h>
#include <ArduinoOTA.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define CAN_TX_PIN   33
#define CAN_RX_PIN   32 
// Note: CAN_STB (MCP2561 pin 8) is hardware-grounded - no software control needed
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_PPS_PIN  34

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 100000); // Lowered from 400kHz to 100kHz for longer extension line stability
CANManager canBus;
TimeSync gpsSync;

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

void taskSensorAcquisition(void* pvParameters);
void taskCANReceive(void* pvParameters);
void taskSerialPassthrough(void* pvParameters);
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

    ArduinoOTA.setHostname("RTR_GPS_Board");
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
    Serial.println("Running: RTR_GPS_Board Configuration");

    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

    gpsSync.begin(Serial2, GPS_PPS_PIN, GPS_RX_PIN, GPS_TX_PIN);
    xTaskCreatePinnedToCore(taskSensorAcquisition, "GPSTask", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskSerialPassthrough, "SerialPassTask", 2048, NULL, 4, NULL, 1);

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

void taskSerialPassthrough(void* pvParameters) {
    String cmdBuffer = "";
    while (true) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            Serial2.write(c);
            
            // Accumulate command for PC screen feedback
            if (c == '\r' || c == '\n') {
                if (cmdBuffer.length() > 0) {
                    Serial.printf("\n[ESP32 -> GPS: %s]\n", cmdBuffer.c_str());
                    cmdBuffer = "";
                }
            } else if (c >= 32 && c <= 126) {
                cmdBuffer += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms responsiveness
    }
}

void taskSensorAcquisition(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz
        GPSPayload gpsData;
        if (gpsSync.getGPSData(gpsData)) {
            // Split double Lat/Lon and transmit
            canBus.transmitDoubleSplit(CAN_ID_GPS_LAT_UPPER, CAN_ID_GPS_LAT_LOWER, gpsData.latitude);
            canBus.transmitDoubleSplit(CAN_ID_GPS_LON_UPPER, CAN_ID_GPS_LON_LOWER, gpsData.longitude);
            
            // Transmit scaled floats
            canBus.transmitScaled(CAN_ID_GPS_ALT, gpsData.altitude, CAN_Scale::GPS_ALT);
            canBus.transmitScaled(CAN_ID_GPS_SPEED, gpsData.speed, CAN_Scale::GPS_SPEED);
            canBus.transmitScaled(CAN_ID_GPS_AZIMUTH, (float)gpsData.heading, CAN_Scale::GPS_AZIMUTH);
            
            // JST time as int32 (stored in utc field)
            canBus.transmitInt32(CAN_ID_GPS_UTC, (int32_t)gpsData.utc);
            
            // Transmit new GPS metrics
            canBus.transmitInt32(CAN_ID_GPS_SATS, (int32_t)gpsData.sat_count);
            canBus.transmitInt32(CAN_ID_GPS_FIX, (int32_t)gpsData.fix_status);
            canBus.transmitScaled(CAN_ID_GPS_HDOP, gpsData.hdop, CAN_Scale::GPS_HDOP);

            // Teleplot Output (10Hz)
            Serial.printf(">gps_lat:%.6f\n", gpsData.latitude);
            Serial.printf(">gps_lon:%.6f\n", gpsData.longitude);
            Serial.printf(">gps_alt:%.2f\n", gpsData.altitude);
            Serial.printf(">gps_speed:%.2f\n", gpsData.speed);
            Serial.printf(">gps_heading:%.2f\n", (float)gpsData.heading / 100.0f);
            Serial.printf(">gps_sats:%d\n", gpsData.sat_count);
            Serial.printf(">gps_fix:%d\n", gpsData.fix_status);
        }
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
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
