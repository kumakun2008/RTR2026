#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "can_manager.hpp"
#include <WiFi.h>
#include <ArduinoOTA.h>

#define CAN_TX_PIN 16
#define CAN_RX_PIN 17

CANManager canBus;

volatile bool alertLowBattery = false;
volatile bool alertLowAltitude = false;
volatile bool alertCalibStart = false;
volatile bool alertCalibEnd = false;

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

void taskCANReceive(void* pvParameters);
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

    ArduinoOTA.setHostname("RTR_Speaker_Board");
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
    Serial.println("Running: RTR_Speaker_Board Configuration");

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

    // Connect voice synth module on Serial2 (AquesTalk Pico LSI at 9600 bps: RX=IO33, TX=IO32)
    Serial2.begin(9600, SERIAL_8N1, 33, 32); 
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
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
        }

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

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
