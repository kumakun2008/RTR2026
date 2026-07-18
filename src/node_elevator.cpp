/**
 * @file node_elevator.cpp
 * @brief Elevator (horizontal stabilizer) angle sensor node.
 *        Hardware : ESP32-WROOM-32E + MPU6050 (I2C)
 *        CAN      : TWAI 1Mbps via CANManager
 *        I2C Pins : SDA=GPIO21, SCL=GPIO22  (ESP32 default)
 *        CAN Pins : TX=GPIO25, RX=GPIO26    (昨年プログラムと同設定)
 *
 *  [改修内容 vs 昨年]
 *   - CAN: arduino-CAN ライブラリ → プロジェクト共通の CANManager (TWAI) に統一
 *   - CAN ID / スケーリング: 独自バイナリフォーマット → can_protocol.hpp の統一定義を使用
 *   - ジャイロオフセットをキャリブ後に mpu オブジェクトへ書き戻し
 *   - 5点移動平均をピッチだけでなく全軸に適用
 *   - Teleplot 出力を毎ループ (100ms) で出力
 *
 * @author Team ЯTR
 * @date   2026-07-17
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include "can_manager.hpp"
#include "can_protocol.hpp"

// ======== Pin Assignments (ESP32-WROOM-32E) ========
// I2C: ESP32 default pins
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
// CAN: 昨年プログラム (CAN_TX=25, CAN_RX=26) と同一
#define CAN_TX_PIN    25
#define CAN_RX_PIN    26

// ======== Madgwick filter ========
constexpr float MADGWICK_FREQ = 100.0f;  // [Hz] ループ周期と一致させること

// ======== Moving-average filter length ========
constexpr int MA_LEN = 5;

// ======== Timing ========
constexpr uint32_t LOOP_INTERVAL_MS    = 10;    // 100 Hz
constexpr uint32_t SEND_INTERVAL_MS    = 10;    // 100 Hz
constexpr uint32_t DEBUG_INTERVAL_MS   = 100;   // 10 Hz Teleplot 出力
constexpr uint32_t DETAIL_LOG_MS       = 5000;  // 5秒毎の詳細ログ

// ======== Objects ========
MPU6050    mpu;
Madgwick   madgwick;
CANManager canBus;

// ======== Sensor data ========
volatile float elevPitch = 0.0f;
volatile float elevRoll  = 0.0f;
volatile float elevYaw   = 0.0f;
volatile float ax = 0.0f, ay = 0.0f, az = 0.0f;
volatile float gx = 0.0f, gy = 0.0f, gz = 0.0f;

// Gyro bias (calib)
float gyroOffX = 0.0f, gyroOffY = 0.0f, gyroOffZ = 0.0f;

// Moving-average buffers [pitch, roll, yaw]
float maBuf[3][MA_LEN] = {};
int   maIdx = 0;

bool imuReady  = false;

// --------------------------------------------------------------------------
// 移動平均フィルタ更新
// --------------------------------------------------------------------------
void maUpdate(float p, float r, float y) {
    maBuf[0][maIdx] = p;
    maBuf[1][maIdx] = r;
    maBuf[2][maIdx] = y;
    maIdx = (maIdx + 1) % MA_LEN;

    float sp = 0, sr = 0, sy = 0;
    for (int i = 0; i < MA_LEN; i++) {
        sp += maBuf[0][i];
        sr += maBuf[1][i];
        sy += maBuf[2][i];
    }
    elevPitch = sp / MA_LEN;
    elevRoll  = sr / MA_LEN;
    elevYaw   = sy / MA_LEN;
}

// --------------------------------------------------------------------------
// MPU6050 初期化 & キャリブレーション
// Returns false if sensor is not found
// --------------------------------------------------------------------------
bool initMPU6050() {
    mpu.initialize();

    // 接続確認 (リトライ 3回)
    for (int attempt = 0; attempt < 3; attempt++) {
        if (mpu.testConnection()) break;
        Serial.printf("MPU6050 connect attempt %d/3 failed, retrying...\n", attempt + 1);
        delay(1000);
        mpu.initialize();
    }
    if (!mpu.testConnection()) {
        Serial.println("[ERR] MPU6050 connection permanently failed!");
        return false;
    }
    Serial.println("[OK] MPU6050 connected");

    // レンジ設定
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250 °/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2 g

    // 低域通過フィルタ: DLPF 42Hz → ノイズ低減
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);

    // ジャイロオフセットキャリブレーション
    Serial.println("Calibrating MPU6050 gyro... Keep device still for 2s");
    delay(2000);

    constexpr int CALIB_SAMPLES = 200;
    double sumGx = 0, sumGy = 0, sumGz = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
        mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
        sumGx += gx_r / 131.0;
        sumGy += gy_r / 131.0;
        sumGz += gz_r / 131.0;
        delay(5);
    }
    gyroOffX = (float)(sumGx / CALIB_SAMPLES);
    gyroOffY = (float)(sumGy / CALIB_SAMPLES);
    gyroOffZ = (float)(sumGz / CALIB_SAMPLES);

    // オフセットを LSB 単位に変換してセンサへ書き戻し
    mpu.setXGyroOffset((int16_t)(-gyroOffX * 131.0f));
    mpu.setYGyroOffset((int16_t)(-gyroOffY * 131.0f));
    mpu.setZGyroOffset((int16_t)(-gyroOffZ * 131.0f));

    Serial.printf("[OK] Gyro offsets applied: X=%.3f Y=%.3f Z=%.3f dps\n",
                  gyroOffX, gyroOffY, gyroOffZ);
    Serial.println("[OK] MPU6050 calibration complete");
    return true;
}

// --------------------------------------------------------------------------
// IMU データ読み取り & Madgwick 更新
// --------------------------------------------------------------------------
void readIMU() {
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    // 物理量変換 (クランプで異常値を弾く)
    ax = constrain(ax_r / 16384.0f, -2.0f, 2.0f);    // g
    ay = constrain(ay_r / 16384.0f, -2.0f, 2.0f);
    az = constrain(az_r / 16384.0f, -2.0f, 2.0f);
    gx = constrain(gx_r / 131.0f, -250.0f, 250.0f);  // dps
    gy = constrain(gy_r / 131.0f, -250.0f, 250.0f);
    gz = constrain(gz_r / 131.0f, -250.0f, 250.0f);

    // Madgwick 6DOF 更新
    madgwick.updateIMU(gx, gy, gz, ax, ay, az);

    float rawPitch = madgwick.getPitch();
    float rawRoll  = madgwick.getRoll();
    float rawYaw   = madgwick.getYaw();

    // 異常値チェック
    if (abs(rawPitch) > 180.0f || abs(rawRoll) > 180.0f) {
        Serial.println("[WARN] Abnormal Euler angle – skipping update");
        return;
    }

    maUpdate(rawPitch, rawRoll, rawYaw);
}

// --------------------------------------------------------------------------
// CAN 送信 (全軸)
// --------------------------------------------------------------------------
void sendElevatorCAN() {
    if (!canBus.isInitialized()) return;

    // 角度 (Madgwick フィルタ後)
    canBus.transmitScaled(CAN_ID_ELEV_PITCH, elevPitch, CAN_Scale::ANGLE);
    canBus.transmitScaled(CAN_ID_ELEV_ROLL,  elevRoll,  CAN_Scale::ANGLE);
    canBus.transmitScaled(CAN_ID_ELEV_YAW,   elevYaw,   CAN_Scale::ANGLE);

    // 加速度 (生データ)
    canBus.transmitScaled(CAN_ID_ELEV_ACC_X, ax, CAN_Scale::ACCEL);
    canBus.transmitScaled(CAN_ID_ELEV_ACC_Y, ay, CAN_Scale::ACCEL);
    canBus.transmitScaled(CAN_ID_ELEV_ACC_Z, az, CAN_Scale::ACCEL);

    // ジャイロ (生データ)
    canBus.transmitScaled(CAN_ID_ELEV_GYRO_X, gx, CAN_Scale::GYRO);
    canBus.transmitScaled(CAN_ID_ELEV_GYRO_Y, gy, CAN_Scale::GYRO);
    canBus.transmitScaled(CAN_ID_ELEV_GYRO_Z, gz, CAN_Scale::GYRO);
}

// --------------------------------------------------------------------------
// setup
// --------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== Elevator Node (ESP32-WROOM-32E + MPU6050) Start ===");

    // I2C 初期化
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    Serial.printf("[OK] I2C @ SDA=%d SCL=%d 400kHz\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // Madgwick フィルタ初期化
    madgwick.begin(MADGWICK_FREQ);

    // MPU6050 初期化 & キャリブレーション
    imuReady = initMPU6050();
    if (!imuReady) {
        Serial.println("[ERR] MPU6050 FAILED – check connections (SDA/SCL/VCC/GND). Operating with offline logic (0 values).");
    }

    // CAN 初期化 (プロジェクト共通 CANManager)
    if (canBus.begin(CAN_TX_PIN, CAN_RX_PIN)) {
        Serial.println("[OK] CAN Bus (TWAI) 1Mbps started");
    } else {
        Serial.println("[WARN] CAN init failed – operating in Serial-only mode");
    }

    Serial.println("=== Elevator Node initialized ===");
}

// --------------------------------------------------------------------------
// loop
// --------------------------------------------------------------------------
void loop() {
    static uint32_t lastLoop   = 0;
    static uint32_t lastDebug  = 0;
    static uint32_t lastDetail = 0;
    uint32_t now = millis();

    // 100 Hz ループ
    if (now - lastLoop >= LOOP_INTERVAL_MS) {
        lastLoop = now;

        if (imuReady) {
            readIMU();
            sendElevatorCAN();
        } else {
            // MPU6050がオフラインの時も0データを送信する
            canBus.transmitScaled(CAN_ID_ELEV_PITCH, 0.0f, CAN_Scale::ANGLE);
            canBus.transmitScaled(CAN_ID_ELEV_ROLL,  0.0f,  CAN_Scale::ANGLE);
            canBus.transmitScaled(CAN_ID_ELEV_YAW,   0.0f,   CAN_Scale::ANGLE);

            canBus.transmitScaled(CAN_ID_ELEV_ACC_X, 0.0f, CAN_Scale::ACCEL);
            canBus.transmitScaled(CAN_ID_ELEV_ACC_Y, 0.0f, CAN_Scale::ACCEL);
            canBus.transmitScaled(CAN_ID_ELEV_ACC_Z, 0.0f, CAN_Scale::ACCEL);

            canBus.transmitScaled(CAN_ID_ELEV_GYRO_X, 0.0f, CAN_Scale::GYRO);
            canBus.transmitScaled(CAN_ID_ELEV_GYRO_Y, 0.0f, CAN_Scale::GYRO);
            canBus.transmitScaled(CAN_ID_ELEV_GYRO_Z, 0.0f, CAN_Scale::GYRO);
        }

        // 1Hz CAN Heartbeat
        static uint32_t lastHBelev = 0;
        if (now - lastHBelev >= 1000) {
            lastHBelev = now;
            uint8_t hbPayload = NODE_ID_ELEVATOR;
            canBus.transmitRaw(CAN_ID_HB_ELEVATOR, &hbPayload, 1);
        }
    }

    // 10 Hz : Teleplot 出力
    if (now - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = now;
        Serial.printf(">elev_pitch:%.2f\n", elevPitch);
        Serial.printf(">elev_roll:%.2f\n",  elevRoll);
        Serial.printf(">elev_yaw:%.2f\n",   elevYaw);
        Serial.printf(">elev_acc_x:%.3f\n", ax);
        Serial.printf(">elev_acc_y:%.3f\n", ay);
        Serial.printf(">elev_acc_z:%.3f\n", az);
        Serial.printf(">elev_gyro_x:%.3f\n", gx);
        Serial.printf(">elev_gyro_y:%.3f\n", gy);
        Serial.printf(">elev_gyro_z:%.3f\n", gz);
    }

    // 5秒毎の詳細ログ
    if (now - lastDetail >= DETAIL_LOG_MS) {
        lastDetail = now;
        Serial.printf("[INFO] Pitch:%.2f Roll:%.2f Yaw:%.2f | CAN:%s\n",
                      elevPitch, elevRoll, elevYaw,
                      canBus.isInitialized() ? "OK" : "FAIL");
        canBus.printStatus();
    }
}
