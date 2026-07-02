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
    
    // Status flags
    bool has_batteryVolt = false;
    bool has_pitotPress32 = false;
    bool has_altLidar = false;
    bool has_altUS = false;
    bool has_gyro = false;
    bool has_rudderAngle = false;
    bool has_gpsPos = false;
    bool has_gpsAlt = false;
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

    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);

    xTaskCreatePinnedToCore(taskCANReceive, "CANRxTask", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskUIDraw, "UIDrawTask", 8192, NULL, 1, NULL, 0);

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
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
            
            auto getFloat = [&](const uint8_t* d, float scale) {
                int32_t raw;
                memcpy(&raw, d, 4);
                return (float)raw / scale;
            };

            if (rxId == CAN_ID_PITOT_AIRSPEED) {
                flightData.pitotPress32 = getFloat(rxData, CAN_Scale::GPS_SPEED);
                flightData.has_pitotPress32 = true;
            }
            else if (rxId == CAN_ID_PITOT_AOA) {
                flightData.pitotPress31_1 = getFloat(rxData, CAN_Scale::PRESSURE);
            }
            else if (rxId == CAN_ID_PITOT_AOS) {
                flightData.pitotPress31_2 = getFloat(rxData, CAN_Scale::PRESSURE);
            }
            else if (rxId == CAN_ID_PITOT_PITCH) {
                flightData.gyro[0] = getFloat(rxData, CAN_Scale::ANGLE);
                flightData.has_gyro = true;
            }
            else if (rxId == CAN_ID_PITOT_ROLL) {
                flightData.gyro[1] = getFloat(rxData, CAN_Scale::ANGLE);
                flightData.has_gyro = true;
            }
            else if (rxId == CAN_ID_RUDDER_ANGLE) {
                flightData.rudderAngle = getFloat(rxData, CAN_Scale::ANGLE);
                flightData.has_rudderAngle = true;
            }
            else if (rxId == CAN_ID_ALT_LIDAR) {
                uint16_t rawLidar = (uint16_t)((rxData[0] << 8) | rxData[1]);
                flightData.altLidar = (float)rawLidar / 1000.0f;
                flightData.has_altLidar = true;
            }
            else if (rxId == CAN_ID_ALT_US) {
                uint8_t rawUS = rxData[0];
                flightData.altUS = (float)rawUS / 100.0f;
                flightData.has_altUS = true;
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
                flightData.has_gpsAlt = true;
            }
            else if (rxId == CAN_ID_GPS_SPEED) {
                flightData.gpsSpeed = getFloat(rxData, CAN_Scale::GPS_SPEED);
            }
            else if (rxId == CAN_ID_GPS_AZIMUTH) {
                flightData.gpsHeading = (uint16_t)(getFloat(rxData, CAN_Scale::GPS_AZIMUTH));
            }
            else if (rxId == CAN_ID_BATTERY_VOLT) {
                flightData.batteryVolt = getFloat(rxData, CAN_Scale::BATTERY);
                flightData.has_batteryVolt = true;
            }
            else if (rxId == CAN_ID_MAIN_PRESS) {
                flightData.baroPress = getFloat(rxData, CAN_Scale::PRESSURE) / 100.0f;
            }
            
            if (updateGPSLat) {
                memcpy(&flightData.gpsLat, &gpsLatRaw, 8);
                updateGPSLat = false;
                flightData.has_gpsPos = true;
            }
            if (updateGPSLon) {
                memcpy(&flightData.gpsLon, &gpsLonRaw, 8);
                updateGPSLon = false;
                flightData.has_gpsPos = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUIDraw(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250)); // 4Hz (Prevents watchdogs and high bus overhead)
        
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.println("=== RTR FLIGHT DATA ===");
        tft.println("");
        
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        
        // Battery
        if (flightData.has_batteryVolt) {
            tft.printf("Battery : %5.2f V    \n", flightData.batteryVolt);
        } else {
            tft.println("Battery : No Data    ");
        }
        
        // Airspeed
        if (flightData.has_pitotPress32) {
            tft.printf("Airspeed: %5.2f m/s  \n", flightData.pitotPress32);
        } else {
            tft.println("Airspeed: No Data    ");
        }
        
        // Altitude (LiDAR and Ultrasonic)
        if (flightData.has_altLidar) {
            tft.printf("LidarAlt: %5.2f m    \n", flightData.altLidar);
        } else {
            tft.println("LidarAlt: No Data    ");
        }
        
        if (flightData.has_altUS) {
            tft.printf("US Alt  : %5.2f m    \n", flightData.altUS);
        } else {
            tft.println("US Alt  : No Data    ");
        }
        
        // Motion (Pitch/Roll)
        if (flightData.has_gyro) {
            tft.printf("Pitch   : %+5.1f deg  \n", flightData.gyro[0]);
            tft.printf("Roll    : %+5.1f deg  \n", flightData.gyro[1]);
        } else {
            tft.println("Pitch   : No Data    ");
            tft.println("Roll    : No Data    ");
        }
        
        // Rudder Angle
        if (flightData.has_rudderAngle) {
            tft.printf("Rudder  : %+5.2f deg  \n", flightData.rudderAngle);
        } else {
            tft.println("Rudder  : No Data    ");
        }
        
        // GPS Positions
        if (flightData.has_gpsPos) {
            tft.printf("Lat     : %10.6f   \n", flightData.gpsLat);
            tft.printf("Lon     : %10.6f   \n", flightData.gpsLon);
        } else {
            tft.println("Lat     : No Data    ");
            tft.println("Lon     : No Data    ");
        }
        
        if (flightData.has_gpsAlt) {
            tft.printf("GPS Alt : %5.2f m    \n", flightData.gpsAlt);
        } else {
            tft.println("GPS Alt : No Data    ");
        }
    }
}
