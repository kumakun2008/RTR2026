#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <CAN.h>
#include "can_protocol.hpp"

// ========Pin settings==============
const int sndPin = 13;
const int PMOD0 = 22;
const int PMOD1 = 23;
// ==================================

// MAIN variables (simulated state)
float pitch = 0;
float roll = 0;
float yaw = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float mx = 0;
float my = 0;
float mz = 0;
float PcbTemp = 0;

// Node variables
float SpacePres = 0.00;
float SpaceTemp = 0.00;
volatile float Alt = 0.00;
float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;
// ==================================

// =======Network settings===========
const char *ssid = "B116_1_24_2";
const char *password = "msuwamsuwamsuwa";
IPAddress localIp(192, 168, 2, 71); // Own IP
IPAddress gateway(192, 168, 2, 254); // Router IP
IPAddress subnet(255, 255, 255, 0);  // Subnet mask
unsigned int localPort = 8888;       // Port to listen for UDP packets

WiFiUDP Udp;
byte packetBuffer[256];
// ==================================

// ==========Multi core =============
TaskHandle_t thp[2];
// ==================================

// ========Function prototypes=======
void printWifiStatus(void);
void CAN_SEND_NEW(uint32_t id, float value, float scale);
void CAN_SEND_INT32(uint32_t id, int32_t value);
void handleUdpSimData(byte oldAddr, float fVal);
void processRxPacket(uint32_t rxId, const uint8_t* rxData, int dlc);
void CAN_RECEIVE(void *pvParameters);
void toneAlt(void *pvParameters);
// ==================================

void setup()
{
  pinMode(sndPin, OUTPUT);

  Serial.begin(115200);
  Serial.println("Setup: Sim Bridge Active");

  Serial2.begin(9600);
  pinMode(PMOD0, OUTPUT);
  pinMode(PMOD1, OUTPUT);
  digitalWrite(PMOD0, HIGH);
  digitalWrite(PMOD1, HIGH);

  xTaskCreateUniversal(toneAlt, "toneAlt", 8192, NULL, 1, &thp[0], 1);
  xTaskCreateUniversal(CAN_RECEIVE, "CAN_RECEIVE", 8192, NULL, 2, &thp[1], 1);

  // CAN init
  // rxPin, txPin
  CAN.setPins(33, 32);
  // 通信速度を新仕様の 1000E3 (1Mbps) に設定
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
  }

  WiFi.config(localIp, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  printWifiStatus();

  Udp.begin(localPort);
}

void loop()
{
  typedef union
  {
    float val;
    byte binary[4];
  } value;
  value val1;

  byte oldAddr;
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(packetBuffer, 255);

    if (len >= 8 && (packetBuffer[0] == 255 && packetBuffer[1] == 255) && packetBuffer[7] == 255)
    {
      Serial.println("Sim Data Received via UDP");
      oldAddr = packetBuffer[2];
      val1.binary[0] = packetBuffer[3];
      val1.binary[1] = packetBuffer[4];
      val1.binary[2] = packetBuffer[5];
      val1.binary[3] = packetBuffer[6];

      Serial.printf("Old Addr: 0x%02X, Val: %.4f\n", oldAddr, val1.val);

      // UDPリプライの返信
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      for (int i = 0; i <= 3; i++)
      {
        Udp.write(val1.binary[i]);
      }
      Udp.endPacket();

      // 新プロトコルに合わせてCAN送信 & 内部変数更新
      handleUdpSimData(oldAddr, val1.val);
      Serial.println("Sent New CAN data packet");
    }
  }
}

void printWifiStatus()
{
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void CAN_SEND_NEW(uint32_t id, float value, float scale) {
    int32_t val = (int32_t)(value * scale);
    uint8_t data[4];
    memcpy(data, &val, 4);
    
    CAN.beginPacket(id);
    CAN.write(data, 4);
    CAN.endPacket();
}

void CAN_SEND_INT32(uint32_t id, int32_t value) {
    uint8_t data[4];
    memcpy(data, &value, 4);
    
    CAN.beginPacket(id);
    CAN.write(data, 4);
    CAN.endPacket();
}

void handleUdpSimData(byte oldAddr, float fVal) {
    switch (oldAddr) {
        case 0x01: // pitch
            CAN_SEND_NEW(CAN_ID_PITOT_PITCH, fVal, CAN_Scale::ANGLE);
            pitch = fVal;
            break;
        case 0x02: // roll
            CAN_SEND_NEW(CAN_ID_PITOT_ROLL, fVal, CAN_Scale::ANGLE);
            roll = fVal;
            break;
        case 0x03: // yaw
            CAN_SEND_NEW(CAN_ID_PITOT_YAW, fVal, CAN_Scale::ANGLE);
            yaw = fVal;
            break;
        case 0x04: // ax
            CAN_SEND_NEW(CAN_ID_PITOT_ACC_X, fVal, CAN_Scale::ACCEL);
            ax = fVal;
            break;
        case 0x05: // ay
            CAN_SEND_NEW(CAN_ID_PITOT_ACC_Y, fVal, CAN_Scale::ACCEL);
            ay = fVal;
            break;
        case 0x06: // az
            CAN_SEND_NEW(CAN_ID_PITOT_ACC_Z, fVal, CAN_Scale::ACCEL);
            az = fVal;
            break;
        case 0x07: // gx
            CAN_SEND_NEW(CAN_ID_PITOT_GYRO_X, fVal, CAN_Scale::GYRO);
            gx = fVal;
            break;
        case 0x08: // gy
            CAN_SEND_NEW(CAN_ID_PITOT_GYRO_Y, fVal, CAN_Scale::GYRO);
            gy = fVal;
            break;
        case 0x09: // gz
            CAN_SEND_NEW(CAN_ID_PITOT_GYRO_Z, fVal, CAN_Scale::GYRO);
            gz = fVal;
            break;
        case 0x0A: // mx
            CAN_SEND_NEW(CAN_ID_PITOT_MAG_X, fVal, CAN_Scale::MAG);
            mx = fVal;
            break;
        case 0x0B: // my
            CAN_SEND_NEW(CAN_ID_PITOT_MAG_Y, fVal, CAN_Scale::MAG);
            my = fVal;
            break;
        case 0x0C: // mz
            CAN_SEND_NEW(CAN_ID_PITOT_MAG_Z, fVal, CAN_Scale::MAG);
            mz = fVal;
            break;
        case 0x0D: // PcbTemp
            CAN_SEND_NEW(CAN_ID_MAIN_TEMP, fVal, CAN_Scale::TEMP);
            PcbTemp = fVal;
            break;
        case 0x0E: // SDstatus
            CAN_SEND_INT32(CAN_ID_SD_STATUS, (int32_t)fVal);
            break;
        case 0x0F: // SpacePres
            CAN_SEND_NEW(CAN_ID_MAIN_PRESS, fVal * 100.0f, CAN_Scale::PRESSURE); // hPa to Pa, then scaled
            SpacePres = fVal;
            break;
        case 0x10: // SpaceTemp
            CAN_SEND_NEW(CAN_ID_MAIN_TEMP, fVal, CAN_Scale::TEMP);
            SpaceTemp = fVal;
            break;
        case 0x11: // Alt
            CAN_SEND_NEW(CAN_ID_ALT_LIDAR, fVal, CAN_Scale::DISTANCE); // Alt is in meters
            Alt = fVal;
            break;
        case 0x12: // Pitot1 (Airspeed)
            CAN_SEND_NEW(CAN_ID_PITOT_AIRSPEED, fVal, CAN_Scale::GPS_SPEED);
            Pitot1 = fVal;
            break;
        case 0x13: // Pitot2 (AOA)
            CAN_SEND_NEW(CAN_ID_PITOT_AOA, fVal, CAN_Scale::PRESSURE);
            Pitot2 = fVal;
            break;
        case 0x14: // Pitot3 (AOS)
            CAN_SEND_NEW(CAN_ID_PITOT_AOS, fVal, CAN_Scale::PRESSURE);
            Pitot3 = fVal;
            break;
        default:
            break;
    }
}

float getFloatFromBytes(const uint8_t* d, float scale) {
    int32_t raw;
    memcpy(&raw, d, 4);
    return (float)raw / scale;
}

void processRxPacket(uint32_t rxId, const uint8_t* rxData, int dlc) {
    if (rxId == CAN_ID_PITOT_PITCH) {
        pitch = getFloatFromBytes(rxData, CAN_Scale::ANGLE);
    }
    else if (rxId == CAN_ID_PITOT_ROLL) {
        roll = getFloatFromBytes(rxData, CAN_Scale::ANGLE);
    }
    else if (rxId == CAN_ID_PITOT_YAW) {
        yaw = getFloatFromBytes(rxData, CAN_Scale::ANGLE);
    }
    else if (rxId == CAN_ID_PITOT_ACC_X) {
        ax = getFloatFromBytes(rxData, CAN_Scale::ACCEL);
    }
    else if (rxId == CAN_ID_PITOT_ACC_Y) {
        ay = getFloatFromBytes(rxData, CAN_Scale::ACCEL);
    }
    else if (rxId == CAN_ID_PITOT_ACC_Z) {
        az = getFloatFromBytes(rxData, CAN_Scale::ACCEL);
    }
    else if (rxId == CAN_ID_PITOT_GYRO_X) {
        gx = getFloatFromBytes(rxData, CAN_Scale::GYRO);
    }
    else if (rxId == CAN_ID_PITOT_GYRO_Y) {
        gy = getFloatFromBytes(rxData, CAN_Scale::GYRO);
    }
    else if (rxId == CAN_ID_PITOT_GYRO_Z) {
        gz = getFloatFromBytes(rxData, CAN_Scale::GYRO);
    }
    else if (rxId == CAN_ID_MAIN_PRESS) {
        SpacePres = getFloatFromBytes(rxData, CAN_Scale::PRESSURE) / 100.0f; // Pa to hPa
    }
    else if (rxId == CAN_ID_MAIN_TEMP) {
        SpaceTemp = getFloatFromBytes(rxData, CAN_Scale::TEMP);
    }
    else if (rxId == CAN_ID_ALT_LIDAR) {
        uint16_t rawLidar = (uint16_t)((rxData[0] << 8) | rxData[1]);
        Alt = (float)rawLidar / 1000.0f; // mm to meters
    }
    else if (rxId == CAN_ID_ALT_US) {
        uint8_t rawUS = rxData[0];
        Alt = (float)rawUS / 100.0f; // cm to meters
    }
    else if (rxId == CAN_ID_PITOT_AIRSPEED) {
        Pitot1 = getFloatFromBytes(rxData, CAN_Scale::GPS_SPEED);
    }
    else if (rxId == CAN_ID_PITOT_AOA) {
        Pitot2 = getFloatFromBytes(rxData, CAN_Scale::PRESSURE);
    }
    else if (rxId == CAN_ID_PITOT_AOS) {
        Pitot3 = getFloatFromBytes(rxData, CAN_Scale::PRESSURE);
    }
}

void CAN_RECEIVE(void *pvParameters)
{
  while (1)
  {
    if (CAN.parsePacket())
    {
      uint32_t rxId = CAN.packetId();
      int dlc = CAN.packetDlc();
      
      uint8_t rxData[8] = {0};
      for (int i = 0; i < dlc && i < 8; i++) {
          rxData[i] = CAN.read();
      }
      
      processRxPacket(rxId, rxData, dlc);
    }
    delay(1);
  }
}

void toneAlt(void *pvParameters)
{
  while (1)
  {
    int altInt = 10 - ((int)Alt);
    if (altInt > 8){
      altInt = 8;
    }else if(altInt < 0){
      altInt = 0;
    }
    int heltz = 1000 / altInt;
    int toneTimer = heltz / 2;

    tone(sndPin, 1000, toneTimer);
    delay(heltz);
  }
}
