#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "can_manager.hpp"
#include <WiFi.h>
#include <ArduinoOTA.h>

#define CAN_TX_PIN 16
#define CAN_RX_PIN 17

CANManager canBus;

// Global flight data variables
volatile float currentAirspeed = 0.0f;
volatile bool hasAirspeed = false;
volatile float currentAltitude = 999.0f;
volatile bool hasAltitude = false;

volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

uint32_t lastSpeedSpeakTime = 0;
uint32_t lastBeepTime = 0;

// Converts speed value to AquesTalk phonetic string (e.g., 7.2 -> "nanaten'ni.\r")
String getNumberSpeech(float val) {
    if (val < 0.0f) val = 0.0f;
    if (val > 99.9f) val = 99.9f;

    int whole = (int)val;
    int tenth = (int)((val - whole) * 10.0f + 0.5f);
    if (tenth >= 10) {
        whole += 1;
        tenth = 0;
    }

    String speech = "";

    // Tens & Ones digits
    if (whole >= 10) {
        int tens = whole / 10;
        int ones = whole % 10;

        if (tens == 2) speech += "ni'";
        else if (tens == 3) speech += "sa'n";
        else if (tens == 4) speech += "yo'n";
        else if (tens == 5) speech += "go";
        else if (tens == 6) speech += "roku";
        else if (tens == 7) speech += "nana";
        else if (tens == 8) speech += "hachi";
        else if (tens == 9) speech += "kyu'u";

        speech += "ju'u";

        if (ones == 1) speech += "ichi";
        else if (ones == 2) speech += "ni";
        else if (ones == 3) speech += "sa'n";
        else if (ones == 4) speech += "yo'n";
        else if (ones == 5) speech += "go";
        else if (ones == 6) speech += "roku";
        else if (ones == 7) speech += "nana";
        else if (ones == 8) speech += "hachi";
        else if (ones == 9) speech += "kyu'u";
    } else {
        if (whole == 0) speech += "ze'ro";
        else if (whole == 1) speech += "ichi";
        else if (whole == 2) speech += "ni";
        else if (whole == 3) speech += "sa'n";
        else if (whole == 4) speech += "yo'n";
        else if (whole == 5) speech += "go";
        else if (whole == 6) speech += "roku";
        else if (whole == 7) speech += "nana";
        else if (whole == 8) speech += "hachi";
        else if (whole == 9) speech += "kyu'u";
    }

    // Decimal point
    speech += "te'n";

    // Tenths digit
    if (tenth == 0) speech += "re'i";
    else if (tenth == 1) speech += "ichi";
    else if (tenth == 2) speech += "ni";
    else if (tenth == 3) speech += "sa'n";
    else if (tenth == 4) speech += "yo'n";
    else if (tenth == 5) speech += "go";
    else if (tenth == 6) speech += "roku";
    else if (tenth == 7) speech += "nana";
    else if (tenth == 8) speech += "hachi";
    else if (tenth == 9) speech += "kyu'u";

    speech += ".\r";
    return speech;
}

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
    
    // Set speech speed to 180% to fit 1Hz speech rate (per user request)
    Serial2.print("?S180\r");
    delay(50);
    
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

        if (canBus.receiveRaw(rxId, rxData, rxDlc, 10)) {
            if (rxId == CAN_ID_OTA_START) {
                isOtaMode = true;
                continue;
            }
            
            // Extract Airspeed from CAN_ID_PITOT_AIRSPEED
            if (rxId == CAN_ID_PITOT_AIRSPEED) {
                int32_t raw;
                memcpy(&raw, rxData, 4);
                currentAirspeed = (float)raw / 100.0f; // Scale factor GPS_SPEED is 100.0f
                hasAirspeed = true;
            }
            // Extract Altitude from CAN_ID_ALT_US (Ultrasonic sensor)
            else if (rxId == CAN_ID_ALT_US && rxDlc >= 1) {
                uint8_t rawUS = rxData[0];
                currentAltitude = (float)rawUS / 100.0f; // Convert cm to meters
                hasAltitude = true;
            }
        }

        uint32_t now = millis();

        // Ground-effect warning region check (<= 3.0m)
        if (hasAltitude && currentAltitude <= 3.0f) {
            float alt = currentAltitude;
            if (alt < 0.0f) alt = 0.0f;
            float t = alt / 3.0f;
            // Sound pitch gets faster as distance to ground decreases
            // Interval: 150ms at 0m, 1000ms at 3m
            uint32_t interval_ms = 150 + (uint32_t)(t * 850.0f);

            if (now - lastBeepTime >= interval_ms) {
                Serial2.print("pi.\r"); // AquesTalk syllable "pi" acts as buzzer sound
                lastBeepTime = now;
            }
        } else {
            // Speak airspeed periodically every 1 second (1Hz) outside the 3m alarm region
            if (hasAirspeed && (now - lastSpeedSpeakTime >= 1000)) {
                String speedSpeech = getNumberSpeech(currentAirspeed);
                Serial2.print(speedSpeech);
                // Overwrite debug output
                Serial.printf("Speaking speed (1Hz): %s\n", speedSpeech.c_str());
                lastSpeedSpeakTime = now;
            }
        }

        static uint32_t lastHBspeaker = 0;
        if (now - lastHBspeaker >= 1000) {
            lastHBspeaker = now;
            uint8_t hbPayload = NODE_ID_SPEAKER;
            canBus.transmitRaw(CAN_ID_HB_SPEAKER, &hbPayload, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
