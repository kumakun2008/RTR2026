#include <Arduino.h>
#include <esp_task_wdt.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "can_manager.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define CAN_TX_PIN 25
#define CAN_RX_PIN 26
#define TFT_CS   5
#define TFT_DC   17
#define TFT_RST  16
#define SPI_SCK_PIN 14
#define SPI_MISO_PIN 33
#define SPI_MOSI_PIN 13
#define SD_CS_PIN 15
#define BUTTON_PAGE_PIN 12 

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
CANManager canBus;
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

volatile int displayPage = 0;
volatile bool isOtaMode = false;
uint32_t otaTimeoutStart = 0;

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

void taskCANReceive(void* pvParameters);
void taskUIDraw(void* pvParameters);
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

    ArduinoOTA.setHostname("RTR_Display_Board");
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
    Serial.println("Running: RTR_Display_Board Configuration");

    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.println("RTR2026 Display Active");

    pinMode(BUTTON_PAGE_PIN, INPUT_PULLUP);
    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskUIDraw, "UIDrawTask", 4096, NULL, 1, NULL, 0);

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
                flightData.gpsSats = 8;
                flightData.gpsFix = 2;
            }
            else if (rxId == CAN_ID_BATTERY_VOLT && rxDlc >= 4) {
                memcpy(&flightData.batteryVolt, rxData, 4);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUIDraw(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(66)); // 15Hz
        
        if (digitalRead(BUTTON_PAGE_PIN) == LOW) {
            displayPage = (displayPage + 1) % 3;
            tft.fillScreen(ILI9341_BLACK);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        
        if (displayPage == 0) {
            tft.setTextColor(ILI9341_GREEN);
            tft.println("=== RTR DASHBOARD ===");
            tft.setTextColor(ILI9341_WHITE);
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
