#include <Arduino.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "sensor_manager.hpp"
#include "can_manager.hpp"

// ======== GPIO Pin Assignments (STM32F303K8T6) ========
// I2C通信 (センサーバス): SDA=PA14, SCL=PA15
#define I2C_SDA_PIN PA14
#define I2C_SCL_PIN PA15

// CAN通信 (トランシーバ: BD41041FJ-CE2)
// ※ 回路図上は CAN_RX=PA2, CAN_TX=PA3 となっているが、STM32F303K8T6ではそれらのピンにCAN代替機能がないため、
//    ハードウェアの改修（パターンカットおよび PA2->PA11, PA3->PA12 へのジャンパ配線）を行う前提とし、
//    コード上は本来のCANピンである PA11 (RX), PA12 (TX) を指定する。
// BD41041FJ-CE2 STB PIN: STB=LOW で Normal Mode (内部プルアップあり、未接続でスタンバイになる)
// このPCBではSTBはGNDに直結済みのため、ソフトウェア制御は不要
#define CAN_RX_PIN   PA11
#define CAN_TX_PIN   PA12

// ステータスLED (Active High)
#define LED_PIN      PA4

// 外部コネクタ(J1)用 UART デバッグピン参考: PA9, PA10
// IMU割り込み参考: INT1=PA0, INT2=PA1

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
CANManager canBus;

ICM42688Sensor rudderIMU(i2cBus, 0x68);
float rawRudderAngle = 0.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- RTR2026 Avionics Initialization ---");
    Serial.println("Running: RTR_Rudder_Board Configuration (STM32)");

    // ステータスLEDの初期化
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // 起動表示としてONにする

    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    } else {
        Serial.println("[ERROR] CAN Bus Initialization Failed!");
    }

    rudderIMU.begin();

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    static uint32_t lastRead = 0;
    static uint32_t lastLEDToggle = 0;
    static bool ledState = true;

    // ステータスLEDの点滅処理 (500ms周期)
    if (millis() - lastLEDToggle >= 500) {
        lastLEDToggle = millis();
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }

    static IMUPayload imuData = {0};

    if (millis() - lastRead >= 10) { // 100Hz Rate
        lastRead = millis();
        
        rudderIMU.read(imuData);
        // Acc X, Y, Z
        canBus.transmitScaled(CAN_ID_RUDDER_ACC_X, imuData.accel_x, CAN_Scale::ACCEL);
        canBus.transmitScaled(CAN_ID_RUDDER_ACC_Y, imuData.accel_y, CAN_Scale::ACCEL);
        canBus.transmitScaled(CAN_ID_RUDDER_ACC_Z, imuData.accel_z, CAN_Scale::ACCEL);
        
        // Gyro X, Y, Z
        canBus.transmitScaled(CAN_ID_RUDDER_GYRO_X, imuData.gyro_x, CAN_Scale::GYRO);
        canBus.transmitScaled(CAN_ID_RUDDER_GYRO_Y, imuData.gyro_y, CAN_Scale::GYRO);
        canBus.transmitScaled(CAN_ID_RUDDER_GYRO_Z, imuData.gyro_z, CAN_Scale::GYRO);
        
        uint8_t angleBytes[2];
        if (i2cBus.readRegister(0x36, 0x0E, angleBytes, 2)) {
            uint16_t rawAngle = (uint16_t)((angleBytes[0] << 8) | angleBytes[1]);
            rawRudderAngle = (float)rawAngle * (360.0f / 4096.0f);
        }

        canBus.transmitScaled(CAN_ID_RUDDER_ANGLE, rawRudderAngle, CAN_Scale::ANGLE);

        // Teleplot Output (10Hz)
        static uint32_t lastPlot = 0;
        if (millis() - lastPlot >= 100) {
            lastPlot = millis();
            Serial.printf(">rudder_acc_x:%.3f\n", imuData.accel_x);
            Serial.printf(">rudder_acc_y:%.3f\n", imuData.accel_y);
            Serial.printf(">rudder_acc_z:%.3f\n", imuData.accel_z);
            Serial.printf(">rudder_gyro_x:%.3f\n", imuData.gyro_x);
            Serial.printf(">rudder_gyro_y:%.3f\n", imuData.gyro_y);
            Serial.printf(">rudder_gyro_z:%.3f\n", imuData.gyro_z);
            Serial.printf(">rudder_angle:%.2f\n", rawRudderAngle);
        }
    }
}
