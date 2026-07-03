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
#define SPI_SCK_PIN 18
#define SPI_MISO_PIN 19
#define SPI_MOSI_PIN 23
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

struct PrevDrawData {
    float airspeed = -999.0f;
    float pitch = -999.0f;
    float roll = -999.0f;
    float alt = -999.0f;
    uint16_t heading = 999;
    double gpsLat = 0.0;
    double gpsLon = 0.0;
    bool has_gpsPos = false;
    bool useArakawaMap = false;
} prevDraw;

volatile bool useArakawaMap = false;

// Global timestamps for tracking node communication status
volatile uint32_t lastRxMain = 0;
volatile uint32_t lastRxPitot = 0;
volatile uint32_t lastRxRudder = 0;
volatile uint32_t lastRxGPS = 0;
volatile uint32_t lastRxAlt = 0;
volatile uint32_t lastRxBridge = 0;

void drawNodeStatus(int x, int y, const char* name, uint32_t lastRx) {
    tft.setCursor(x, y);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.print(name);
    tft.print(":");
    if (millis() - lastRx < 2000) {
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.print("OK  ");
    } else {
        tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        tft.print("LOST");
    }
}

void drawStaticLayout() {
    tft.fillScreen(ILI9341_BLACK);
    
    // Draw Top Status Strip Line
    tft.drawLine(0, 15, 320, 15, ILI9341_DARKGREY);
    
    // Draw ASI Gauge Outline & Title
    tft.drawCircle(53, 65, 40, ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(45, 30);
    tft.print("ASI");
    
    // Draw AI Gauge Outline
    tft.drawCircle(160, 65, 40, ILI9341_WHITE);
    tft.setCursor(152, 30);
    tft.print("ATT");
    
    // Draw ALT Gauge Outline
    tft.drawCircle(267, 65, 40, ILI9341_WHITE);
    tft.setCursor(259, 30);
    tft.print("ALT");
    
    // Draw HI Gauge Outline & Cardinal Points
    tft.drawCircle(160, 180, 40, ILI9341_WHITE);
    tft.setCursor(152, 145);
    tft.print("HDG");
    tft.setCursor(157, 153); tft.print("N");
    tft.setCursor(157, 199); tft.print("S");
    tft.setCursor(125, 177); tft.print("W");
    tft.setCursor(189, 177); tft.print("E");
    
    // Draw Bottom-Left CAN Status Border
    tft.drawRect(5, 125, 100, 95, ILI9341_WHITE);
    
    // Draw Bottom-Right Map Border
    tft.drawRect(215, 125, 100, 95, ILI9341_WHITE);
}

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
    drawStaticLayout();

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
            
            // Update node communication status timestamps
            if ((rxId >= 0x010 && rxId <= 0x018) || rxId == CAN_ID_BATTERY_VOLT || rxId == CAN_ID_MAIN_PRESS || rxId == CAN_ID_MAIN_TEMP) {
                lastRxMain = millis();
            }
            else if (rxId >= 0x030 && rxId <= 0x037) {
                lastRxPitot = millis();
            }
            else if (rxId >= 0x020 && rxId <= 0x026) {
                lastRxRudder = millis();
            }
            else if (rxId >= 0x050 && rxId <= 0x057) {
                lastRxGPS = millis();
            }
            else if (rxId == CAN_ID_ALT_LIDAR || rxId == CAN_ID_ALT_US) {
                lastRxAlt = millis();
            }
            else if (rxId == 0x042 || rxId == CAN_ID_VOICE_CMD) {
                lastRxBridge = millis();
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
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200)); // 5Hz UI Update Rate
        
        // Process Serial Commands for Debug Map toggles
        while (Serial.available() > 0) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd.equalsIgnoreCase("MAP:SHIGA") || cmd.equalsIgnoreCase("MAP:SHI")) {
                useArakawaMap = false;
                Serial.println("[Display] Switched Map: Shiga Matsubara Beach");
            } else if (cmd.equalsIgnoreCase("MAP:ARAKAWA") || cmd.equalsIgnoreCase("MAP:ARA")) {
                useArakawaMap = true;
                Serial.println("[Display] Switched Map: Arakawa Campus (TMCIT)");
            }
        }

        // Determine current flight parameters
        float current_speed = flightData.has_pitotPress32 ? flightData.pitotPress32 : 0.0f;
        float current_pitch = flightData.has_gyro ? flightData.gyro[0] : 0.0f;
        float current_roll = flightData.has_gyro ? flightData.gyro[1] : 0.0f;
        float current_alt = flightData.has_altLidar ? flightData.altLidar : (flightData.has_altUS ? flightData.altUS : 0.0f);
        uint16_t current_hdg = flightData.gpsHeading % 360;

        // --- 0. Top Telemetry Status Strip ---
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(10, 3);
        if (flightData.has_batteryVolt) {
            tft.printf("BAT: %5.2f V", flightData.batteryVolt);
        } else {
            tft.print("BAT: No Data");
        }
        
        tft.setCursor(120, 3);
        if (flightData.has_rudderAngle) {
            tft.printf("RUD: %+5.1f d", flightData.rudderAngle);
        } else {
            tft.print("RUD: No Data");
        }

        tft.setCursor(230, 3);
        if (flightData.has_gpsAlt) {
            tft.printf("GALT: %4.1f m", flightData.gpsAlt);
        } else {
            tft.print("GALT:No Data");
        }

        // --- 1. ASI Needle Draw/Erase ---
        if (prevDraw.airspeed != -999.0f) {
            float prev_a = (135.0f + (prevDraw.airspeed / 30.0f) * 270.0f) * DEG_TO_RAD;
            int prev_nx = 53 + (int)(30.0f * cos(prev_a));
            int prev_ny = 65 + (int)(30.0f * sin(prev_a));
            tft.drawLine(53, 65, prev_nx, prev_ny, ILI9341_BLACK);
        }
        float speed_draw = constrain(current_speed, 0.0f, 30.0f);
        float new_a = (135.0f + (speed_draw / 30.0f) * 270.0f) * DEG_TO_RAD;
        int new_nx = 53 + (int)(30.0f * cos(new_a));
        int new_ny = 65 + (int)(30.0f * sin(new_a));
        tft.drawLine(53, 65, new_nx, new_ny, ILI9341_RED);

        if (prevDraw.airspeed != current_speed) {
            tft.fillRect(38, 77, 30, 8, ILI9341_BLACK);
            tft.setTextSize(1);
            tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
            tft.setCursor(38, 77);
            tft.printf("%4.1f", current_speed);
        }

        // --- 2. AI (Attitude Indicator) Draw/Erase ---
        if (prevDraw.pitch != -999.0f) {
            float prev_pitch_rad = prevDraw.pitch * DEG_TO_RAD;
            float prev_roll_rad = prevDraw.roll * DEG_TO_RAD;
            float prev_dy = constrain(prevDraw.pitch, -25.0f, 25.0f);
            float prev_cx = 160.0f - prev_dy * sin(prev_roll_rad);
            float prev_cy = 65.0f + prev_dy * cos(prev_roll_rad);
            float prev_dx = 28.0f * cos(prev_roll_rad);
            float prev_dz = 28.0f * sin(prev_roll_rad);
            tft.drawLine((int)(prev_cx - prev_dx), (int)(prev_cy - prev_dz), (int)(prev_cx + prev_dx), (int)(prev_cy + prev_dz), ILI9341_BLACK);
        }
        float new_pitch_rad = current_pitch * DEG_TO_RAD;
        float new_roll_rad = current_roll * DEG_TO_RAD;
        float new_dy = constrain(current_pitch, -25.0f, 25.0f);
        float new_cx = 160.0f - new_dy * sin(new_roll_rad);
        float new_cy = 65.0f + new_dy * cos(new_roll_rad);
        float new_dx = 28.0f * cos(new_roll_rad);
        float new_dz = 28.0f * sin(new_roll_rad);
        tft.drawLine((int)(new_cx - new_dx), (int)(new_cy - new_dz), (int)(new_cx + new_dx), (int)(new_cy + new_dz), ILI9341_CYAN);

        // Redraw Aircraft symbol (Yellow)
        tft.drawLine(152, 65, 168, 65, ILI9341_YELLOW);
        tft.fillCircle(160, 65, 2, ILI9341_YELLOW);

        // --- 3. ALT Needle Draw/Erase ---
        if (prevDraw.alt != -999.0f) {
            float prev_a = (-90.0f + (prevDraw.alt / 20.0f) * 360.0f) * DEG_TO_RAD;
            int prev_nx = 267 + (int)(30.0f * cos(prev_a));
            int prev_ny = 65 + (int)(30.0f * sin(prev_a));
            tft.drawLine(267, 65, prev_nx, prev_ny, ILI9341_BLACK);
        }
        float alt_draw = constrain(current_alt, 0.0f, 20.0f);
        float alt_a = (-90.0f + (alt_draw / 20.0f) * 360.0f) * DEG_TO_RAD;
        int alt_nx = 267 + (int)(30.0f * cos(alt_a));
        int alt_ny = 65 + (int)(30.0f * sin(alt_a));
        tft.drawLine(267, 65, alt_nx, alt_ny, ILI9341_RED);

        if (prevDraw.alt != current_alt) {
            tft.fillRect(252, 77, 30, 8, ILI9341_BLACK);
            tft.setTextSize(1);
            tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
            tft.setCursor(252, 77);
            tft.printf("%4.1f", current_alt);
        }

        // --- 4. HI Needle Draw/Erase ---
        if (prevDraw.heading != 999) {
            float prev_a = (-90.0f + (float)prevDraw.heading) * DEG_TO_RAD;
            int prev_nx = 160 + (int)(28.0f * cos(prev_a));
            int prev_ny = 180 + (int)(28.0f * sin(prev_a));
            tft.drawLine(160, 180, prev_nx, prev_ny, ILI9341_BLACK);
        }
        float hdg_a = (-90.0f + (float)current_hdg) * DEG_TO_RAD;
        int hdg_nx = 160 + (int)(28.0f * cos(hdg_a));
        int hdg_ny = 180 + (int)(28.0f * sin(hdg_a));
        tft.drawLine(160, 180, hdg_nx, hdg_ny, ILI9341_WHITE);

        if (prevDraw.heading != current_hdg) {
            tft.fillRect(150, 175, 20, 8, ILI9341_BLACK);
            tft.setTextSize(1);
            tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
            tft.setCursor(150, 175);
            tft.printf("%3d", current_hdg);
        }

        // --- 5. Bottom-Left GPS Vector Map Grid ---
        bool mapTypeChanged = (useArakawaMap != prevDraw.useArakawaMap);
        bool gpsStateChanged = (flightData.has_gpsPos != prevDraw.has_gpsPos);

        if (mapTypeChanged || gpsStateChanged) {
            // Clear Map inside frame
            tft.fillRect(6, 126, 98, 93, ILI9341_BLACK);
        }

        if (!flightData.has_gpsPos) {
            tft.setTextSize(1);
            tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
            tft.setCursor(38, 170);
            tft.print("NO GPS");
        } else {
            // Erase previous GPS Position dot
            if (prevDraw.has_gpsPos && !mapTypeChanged) {
                float old_mx, old_my;
                if (!prevDraw.useArakawaMap) {
                    old_mx = 5.0f + 100.0f * (prevDraw.gpsLon - 136.2590f) / 0.0070f;
                    old_my = 125.0f + 95.0f * (1.0f - (prevDraw.gpsLat - 35.2750f) / 0.0060f);
                } else {
                    old_mx = 5.0f + 100.0f * (prevDraw.gpsLon - 139.7950f) / 0.0080f;
                    old_my = 125.0f + 95.0f * (1.0f - (prevDraw.gpsLat - 35.7330f) / 0.0060f);
                }
                tft.fillCircle((int)constrain(old_mx, 8.0f, 102.0f), (int)constrain(old_my, 128.0f, 217.0f), 2, ILI9341_BLACK);
            }

            // Redraw Shoreline/Rivers (Fast and prevents erase artifacts)
            if (!useArakawaMap) {
                // Matsubara Beach Shoreline (Green)
                tft.drawLine(10, 190, 95, 135, ILI9341_GREEN);
                // Launch Platform (Yellow)
                tft.drawLine(50, 162, 35, 147, ILI9341_YELLOW);
                
                // Labels
                tft.setTextSize(1);
                tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);
                tft.setCursor(10, 203); tft.print("BIWA LAKE");
                tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
                tft.setCursor(10, 131); tft.print("MATSUBARA");
            } else {
                // Sumida River (Blue)
                tft.drawLine(10, 135, 95, 190, ILI9341_BLUE);
                // TMCIT Campus boundary (Green)
                tft.drawRect(50, 155, 16, 16, ILI9341_GREEN);
                
                // Labels
                tft.setTextSize(1);
                tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                tft.setCursor(10, 203); tft.print("ARAKAWA");
                tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
                tft.setCursor(10, 131); tft.print("TMCIT");
            }

            // Draw new GPS Position dot (Red)
            float mx, my;
            if (!useArakawaMap) {
                mx = 5.0f + 100.0f * (flightData.gpsLon - 136.2590f) / 0.0070f;
                my = 125.0f + 95.0f * (1.0f - (flightData.gpsLat - 35.2750f) / 0.0060f);
            } else {
                mx = 5.0f + 100.0f * (flightData.gpsLon - 139.7950f) / 0.0080f;
                my = 125.0f + 95.0f * (1.0f - (flightData.gpsLat - 35.7330f) / 0.0060f);
            }
            int draw_mx = (int)constrain(mx, 8.0f, 102.0f);
            int draw_my = (int)constrain(my, 128.0f, 217.0f);
            tft.fillCircle(draw_mx, draw_my, 2, ILI9341_RED);
        }

        // --- 6. Bottom-Right CAN Status Grid ---
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
        tft.setCursor(220, 130);
        tft.print("CAN STATUS:");
        
        drawNodeStatus(220, 142, "MN", lastRxMain);
        drawNodeStatus(265, 142, "PT", lastRxPitot);
        
        drawNodeStatus(220, 154, "RD", lastRxRudder);
        drawNodeStatus(265, 154, "GP", lastRxGPS);
        
        drawNodeStatus(220, 166, "AL", lastRxAlt);
        drawNodeStatus(265, 166, "SB", lastRxBridge);

        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setCursor(220, 182);
        tft.printf("MAP:%s ", useArakawaMap ? "TMCIT" : "SHIGA");
        
        tft.setCursor(220, 194);
        if (flightData.has_gpsPos) {
            tft.printf("SPD:%4.1f", flightData.gpsSpeed);
        } else {
            tft.print("SPD: No G");
        }

        // Update previous values for next refresh cycle
        prevDraw.airspeed = current_speed;
        prevDraw.pitch = current_pitch;
        prevDraw.roll = current_roll;
        prevDraw.alt = current_alt;
        prevDraw.heading = current_hdg;
        prevDraw.gpsLat = flightData.gpsLat;
        prevDraw.gpsLon = flightData.gpsLon;
        prevDraw.has_gpsPos = flightData.has_gpsPos;
        prevDraw.useArakawaMap = useArakawaMap;

        // Teleplot output
        if (flightData.has_batteryVolt) Serial.printf(">disp_bat:%.2f\n", flightData.batteryVolt);
        if (flightData.has_pitotPress32) Serial.printf(">disp_airspeed:%.2f\n", flightData.pitotPress32);
        if (flightData.has_altLidar) Serial.printf(">disp_alt_lidar:%.2f\n", flightData.altLidar);
        if (flightData.has_altUS) Serial.printf(">disp_alt_us:%.2f\n", flightData.altUS);
        if (flightData.has_gyro) {
            Serial.printf(">disp_pitch:%.2f\n", flightData.gyro[0]);
            Serial.printf(">disp_roll:%.2f\n", flightData.gyro[1]);
        }
        if (flightData.has_rudderAngle) Serial.printf(">disp_rudder:%.2f\n", flightData.rudderAngle);
        if (flightData.has_gpsPos) {
            Serial.printf(">disp_gps_lat:%.6f\n", flightData.gpsLat);
            Serial.printf(">disp_gps_lon:%.6f\n", flightData.gpsLon);
        }
        if (flightData.has_gpsAlt) Serial.printf(">disp_gps_alt:%.2f\n", flightData.gpsAlt);
    }
}
