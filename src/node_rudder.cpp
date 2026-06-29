#include <Arduino.h>
#include "common_types.hpp"
#include "i2c_manager.hpp"
#include "sensor_manager.hpp"
#include "can_manager.hpp"

#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define CAN_TX_PIN   PA12
#define CAN_RX_PIN   PA11

I2CManager i2cBus(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
CANManager canBus;

ICM42688Sensor rudderIMU(i2cBus, 0x68);
float rawRudderAngle = 0.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("--- RTR2026 Avionics Initialization ---");
    Serial.println("Running: RTR_Rudder_Board Configuration (STM32)");

    if (i2cBus.begin()) {
        Serial.println("[OK] I2C Manager Active");
    }

    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus Driver Active");
    }

    rudderIMU.begin();

    Serial.println("--- Initialization Complete ---");
}

void loop() {
    static uint32_t lastRead = 0;
    if (millis() - lastRead >= 10) { // 100Hz Rate
        lastRead = millis();
        
        IMUPayload imuData;
        float pitch = 0.0f;
        float roll = 0.0f;
        if (rudderIMU.read(imuData)) {
            pitch = atan2(-imuData.accel_x, sqrt(imuData.accel_y * imuData.accel_y + imuData.accel_z * imuData.accel_z)) * 180.0f / PI;
            roll = atan2(imuData.accel_y, imuData.accel_z) * 180.0f / PI;
            canBus.transmitAttitude(pitch, roll);
        }
        
        uint8_t angleBytes[2];
        if (i2cBus.readRegister(0x36, 0x0E, angleBytes, 2)) {
            uint16_t rawAngle = (uint16_t)((angleBytes[0] << 8) | angleBytes[1]);
            rawRudderAngle = (float)rawAngle * (360.0f / 4096.0f);
        }

        canBus.transmitRudderAngle(rawRudderAngle);
    }
}
