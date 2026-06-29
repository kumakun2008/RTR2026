#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <CAN.h>

// ======== Configuration Settings ========
// スケーリング定数（例：1000.0f を掛けてミリメートル単位にする）
const float ALTITUDE_SCALE_FACTOR = 1000.0f;

// 送信先 CAN ID（超音波センサーのCAN ID）
const uint32_t CAN_ID_ULTRASONIC = 0x042;

// ======== Pin settings ==============
const int PMOD0 = 22;
const int PMOD1 = 23;
// ====================================

// ======= Network settings ===========
const char *ssid = "B113_3_MAIN";
const char *password = "msuwamsuwamsuwa";
IPAddress localIp(192, 168, 2, 151); // Own IP
IPAddress gateway(192, 168, 2, 254); // Router IP
IPAddress subnet(255, 255, 255, 0);  // Subnet mask
unsigned int localPort = 8888;       // Port to listen for UDP packets

WiFiUDP Udp;
byte packetBuffer[256];
// ====================================

// WiFiステータスの出力
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

void setup()
{
  Serial.begin(115200);
  
  // 1. 起動時のディレイ追加 (ネットワークルーターや他のCANノードの立ち上がりを待つため)
  delay(3000);
  Serial.println("Starting UDP-CAN Bridge...");

  // PMODの初期設定 (旧コードより継承)
  pinMode(PMOD0, OUTPUT);
  pinMode(PMOD1, OUTPUT);
  digitalWrite(PMOD0, HIGH);
  digitalWrite(PMOD1, HIGH);

  // 2. CAN初期化とエラーハンドリング
  CAN.setPins(33, 32);
  // 通信速度を 1000E3 (1Mbps) に変更
  if (!CAN.begin(1000E3))
  {
    Serial.println("[ERROR] CAN Initialization Failed!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("CAN Initialized at 1Mbps.");

  // 3. WiFi接続とエラーハンドリング
  WiFi.config(localIp, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  const int maxAttempts = 15; // 15秒タイムアウト
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts)
  {
    delay(1000);
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[ERROR] WiFi Connection Failed!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("Connected to WiFi.");
  printWifiStatus();

  // 4. UDP初期化とエラーハンドリング
  if (!Udp.begin(localPort))
  {
    Serial.println("[ERROR] UDP Initialization Failed!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("UDP Listener Initialized.");
}

void loop()
{
  typedef union
  {
    float val;
    byte binary[4];
  } value;
  value val1;

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(packetBuffer, 255);

    // UDP解析ロジックの保全
    if (len >= 8 && (packetBuffer[0] == 255 && packetBuffer[1] == 255) && packetBuffer[7] == 255)
    {
      val1.binary[0] = packetBuffer[3];
      val1.binary[1] = packetBuffer[4];
      val1.binary[2] = packetBuffer[5];
      val1.binary[3] = packetBuffer[6];

      // シミュレータへの返信 (Udp.beginPacket -> Udp.write -> Udp.endPacket)
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      for (int i = 0; i <= 3; i++)
      {
        Udp.write(val1.binary[i]);
      }
      Udp.endPacket();

      // 受信した float 型の高度データをスケーリング係数で乗算し、int32_t 型の整数にキャスト
      int32_t scaledVal = (int32_t)(val1.val * ALTITUDE_SCALE_FACTOR);

      // int32_t を 4バイトの標準的なペイロードとしてパッキング (リトルエンディアン)
      uint8_t payload[4];
      memcpy(payload, &scaledVal, sizeof(scaledVal));

      // CAN送信処理
      CAN.beginPacket(CAN_ID_ULTRASONIC);
      CAN.write(payload, sizeof(payload));
      CAN.endPacket();
    }
  }
}
