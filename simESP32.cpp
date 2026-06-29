#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <CAN.h>

// ========Pin settings==============
const int sndPin = 13;

const int PMOD0 = 22;
const int PMOD1 = 23;
// ==================================

// ========CAN settings==============
uint8_t sgnfcntDgt = 5; // 有効数字

// CAN address(MAIN)
const int pitchCANaddr = 0x01;
const int rollCANaddr = 0x02;
const int yawCANaddr = 0x03;
const int axCANaddr = 0x04;
const int ayCANaddr = 0x05;
const int azCANaddr = 0x06;
const int gxCANaddr = 0x07;
const int gyCANaddr = 0x08;
const int gzCANaddr = 0x09;
const int mxCANaddr = 0x0A;
const int myCANaddr = 0x0B;
const int mzCANaddr = 0x0C;
const int PcbTempCANaddr = 0x0D;
const int SDstatusCANaddr = 0x0E;

// CAN address(NODE)
const int SpacePresCANaddr = 0x0F;
const int SpaceTempCANaddr = 0x10;
const int AltCANaddr = 0x11;
const int Pitot1CANaddr = 0x12;
const int Pitot2CANaddr = 0x13;
const int Pitot3CANaddr = 0x14;

// MAIN
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

// Node(THIS)
float SpacePres = 0.00;
float SpaceTemp = 0.00;
volatile float Alt = 0.00;
float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;
// ==================================

// =======Network settings===========

// 材力
const char *ssid = "B113_3_MAIN";
const char *password = "msuwamsuwamsuwa";
IPAddress localIp(192, 168, 2, 151); // Own IP
IPAddress gateway(192, 168, 2, 254); // Router IP
IPAddress subnet(255, 255, 255, 0);  // Subnet mask

// 空力
// const char *ssid = "B102_1_RTR";
// const char *password = "33103310";
// IPAddress localIp(192, 168, 3, 110); // Own IP
// IPAddress gateway(192, 168, 3, 254); // Router IP
// IPAddress subnet(255, 255, 255, 0);  // Subnet mask

// IPAddress allowedIP(192, 168, 12, 100); // IP of the device that is allowed to send data
unsigned int localPort = 8888; // Port to listen for UDP packets

WiFiUDP Udp;

byte packetBuffer[256];
char ReplyBuffer[] = "acknowledged\n";
// ==================================

// ==========Multi core =============
TaskHandle_t thp[2];
// ==================================

// ========Function prototypes=======
void printWifiStatus(void);
void CAN_SEND(byte, uint32_t, byte, byte);
void CAN_RECEIVE(void *pvParameters);
void chng(byte CANaddr, int32_t data, byte exp);
void simChng(byte, float);
void toneAlt(void *pvParameters);
// ==================================

void setup()
{
  pinMode(sndPin, OUTPUT);

  Serial.begin(115200);
  Serial.println("Setup");

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
  if (!CAN.begin(500E3))
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

  byte CANaddr;
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    int len = Udp.read(packetBuffer, 255);

    if ((packetBuffer[0] == 255 && packetBuffer[1] == 255) && packetBuffer[7] == 255)
    {
      Serial.println("Sim Data");
      CANaddr = packetBuffer[2];
      val1.binary[0] = packetBuffer[3];
      val1.binary[1] = packetBuffer[4];
      val1.binary[2] = packetBuffer[5];
      val1.binary[3] = packetBuffer[6];

      Serial.println(CANaddr, HEX);
      Serial.println(val1.val);
    }

    // send a reply, to the IP address
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    for (int i = 0; i <= 3; i++)
    {
      Udp.write(val1.binary[i]);
    }
    Udp.endPacket();

    uint8_t valSign = val1.val < 0 ? 1 : 0;
    uint32_t valInt = (int)(abs(val1.val) * (pow(10, sgnfcntDgt)));

    // save and send CAN data
    simChng(CANaddr, val1.val);
    CAN_SEND(CANaddr, valInt, valSign, sgnfcntDgt); // sing 0:plus, 1:minus
    Serial.println("Sent CAN data");

    if (len > 0)
    {
      packetBuffer[len] = 0;
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

// 32bit仮定でバイト送る
void CAN_SEND(byte CANaddr, uint32_t data, byte sign, byte exp)
{
  // SEND 32bit data
  // devide data to 4byte
  CAN.beginPacket(CANaddr);
  CAN.write(sign); // 0:plus, 1:minus
  CAN.write(exp);  // 10^exp

  Serial.print("CANaddr: ");
  Serial.println(CANaddr, HEX);
  Serial.print("sign: ");
  Serial.println(sign);
  Serial.print("exp: ");
  Serial.println(exp);

  for (int i = 3; i >= 0; i--)
  {
    byte dataByte = (byte)(data >> (i * 8));
    CAN.write(dataByte);
    // Serial.println(dataByte);
  }
  CAN.endPacket();
  // Serial.println("done");
}

void toneAlt(void *pvParameters)
{
  while (1)
  {

    // String stringOne = String(Alt, 1);
    // Serial2.println("<NUMK VAL=" + stringOne + ">");
    // Serial.println("Alt: " + stringOne);
    // delay(500);
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

void CAN_RECEIVE(void *pvParameters)
{
  while (1)
  {
    if (CAN.parsePacket())
    {
      byte CANaddr = CAN.packetId(); // パケットのID（アドレス）を取得
      byte sign = CAN.read();        // 符号を取得
      byte exp = CAN.read();         // 指数を取得

      int32_t data = 0;
      for (int i = 0; i < 4; i++)
      {
        byte dataByte = CAN.read(); // データの各バイトを受信
        data = (data << 8) | dataByte;
      }

      if (sign == 1)
      {
        data = -data;
      }

      chng(CANaddr, data, exp);
    }

    delay(1);
  }
}

void chng(byte CANaddr, int32_t data, byte exp)
{
  float fData = (float)(data) * (pow(10, (-1 * exp)));
  switch (CANaddr)
  {
  case pitchCANaddr:
    pitch = fData;
    // Serial.print("pitch: ");
    // Serial.println(pitch);
    break;
  case rollCANaddr:
    roll = fData;
    // Serial.print("roll: ");
    // Serial.println(roll);
    break;
  case yawCANaddr:
    yaw = fData;
    // Serial.print("yaw: ");
    // Serial.println(yaw);
    break;
  case axCANaddr:
    ax = fData;
    // Serial.print("ax: ");
    // Serial.println(ax);
    break;
  case ayCANaddr:
    ay = fData;
    // Serial.print("ay: ");
    // Serial.println(ay);
    break;
  case azCANaddr:
    az = fData;
    // Serial.print("az: ");
    // Serial.println(az);
    break;
  case gxCANaddr:
    gx = fData;
    // Serial.print("gx: ");
    // Serial.println(gx);
    break;
  case gyCANaddr:
    gy = fData;
    // Serial.print("gy: ");
    // Serial.println(gy);
    break;
  case gzCANaddr:
    gz = fData;
    // Serial.print("gz: ");
    // Serial.println(gz);
    break;
  case mxCANaddr:
    mx = fData;
    // Serial.print("mx: ");
    // Serial.println(mx);
    break;
  case myCANaddr:
    my = fData;
    // Serial.print("my: ");
    // Serial.println(my);
    break;
  case mzCANaddr:
    mz = fData;
    // Serial.print("mz: ");
    // Serial.println(mz);
    break;
  case PcbTempCANaddr:
    PcbTemp = fData;
    // Serial.print("PcbTemp: ");
    // Serial.println(PcbTemp);
    break;
  case SpacePresCANaddr:
    SpacePres = fData;
    // Serial.print("SpacePres: ");
    // Serial.println(SpacePres);
    break;
  case SpaceTempCANaddr:
    SpaceTemp = fData;
    // Serial.print("SpaceTemp: ");
    // Serial.println(SpaceTemp);
    break;
  case AltCANaddr:
    Alt = fData;
    // Serial.print("Alt: ");
    // Serial.println(Alt);
    break;
  case Pitot1CANaddr:
    Pitot1 = fData;
    // Serial.print("Pitot1: ");
    // Serial.println(Pitot1);
    break;
  case Pitot2CANaddr:
    Pitot2 = fData;
    // Serial.print("Pitot2: ");
    // Serial.println(Pitot2);
    break;
  case Pitot3CANaddr:
    Pitot3 = fData;
    // Serial.print("Pitot3: ");
    // Serial.println(Pitot3);
    break;
  default:
    break;
  }
}

void simChng(byte CANaddr, float fData)
{
  switch (CANaddr)
  {
  case pitchCANaddr:
    pitch = fData;
    // Serial.print("pitch: ");
    // Serial.println(pitch);
    break;
  case rollCANaddr:
    roll = fData;
    // Serial.print("roll: ");
    // Serial.println(roll);
    break;
  case yawCANaddr:
    yaw = fData;
    // Serial.print("yaw: ");
    // Serial.println(yaw);
    break;
  case axCANaddr:
    ax = fData;
    // Serial.print("ax: ");
    // Serial.println(ax);
    break;
  case ayCANaddr:
    ay = fData;
    // Serial.print("ay: ");
    // Serial.println(ay);
    break;
  case azCANaddr:
    az = fData;
    // Serial.print("az: ");
    // Serial.println(az);
    break;
  case gxCANaddr:
    gx = fData;
    // Serial.print("gx: ");
    // Serial.println(gx);
    break;
  case gyCANaddr:
    gy = fData;
    // Serial.print("gy: ");
    // Serial.println(gy);
    break;
  case gzCANaddr:
    gz = fData;
    // Serial.print("gz: ");
    // Serial.println(gz);
    break;
  case mxCANaddr:
    mx = fData;
    // Serial.print("mx: ");
    // Serial.println(mx);
    break;
  case myCANaddr:
    my = fData;
    // Serial.print("my: ");
    // Serial.println(my);
    break;
  case mzCANaddr:
    mz = fData;
    // Serial.print("mz: ");
    // Serial.println(mz);
    break;
  case PcbTempCANaddr:
    PcbTemp = fData;
    // Serial.print("PcbTemp: ");
    // Serial.println(PcbTemp);
    break;
  case SpacePresCANaddr:
    SpacePres = fData;
    // Serial.print("SpacePres: ");
    // Serial.println(SpacePres);
    break;
  case SpaceTempCANaddr:
    SpaceTemp = fData;
    // Serial.print("SpaceTemp: ");
    // Serial.println(SpaceTemp);
    break;
  case AltCANaddr:
    Alt = fData;
    // Serial.print("Alt: ");
    // Serial.println(Alt);
    break;
  case Pitot1CANaddr:
    Pitot1 = fData;
    // Serial.print("Pitot1: ");
    // Serial.println(Pitot1);
    break;
  case Pitot2CANaddr:
    Pitot2 = fData;
    // Serial.print("Pitot2: ");
    // Serial.println(Pitot2);
    break;
  case Pitot3CANaddr:
    Pitot3 = fData;
    // Serial.print("Pitot3: ");
    // Serial.println(Pitot3);
    break;
  default:
    break;
  }
}
