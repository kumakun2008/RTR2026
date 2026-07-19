#include <Arduino.h>
constexpr int URM_TRIG_PIN = D2;
constexpr int URM_ECHO_PIN = D1;

void setup()
{
    Serial.begin(115200);

    pinMode(URM_TRIG_PIN, OUTPUT);
    pinMode(URM_ECHO_PIN, INPUT);

    // URM37は通常時HIGH
    digitalWrite(URM_TRIG_PIN, HIGH);

    delay(1000);
    Serial.println("URM37 test start");
}

void loop()
{
    // LOWパルスで測距開始
    digitalWrite(URM_TRIG_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(URM_TRIG_PIN, HIGH);

    // URM37はECHOのLOW期間を測定する
    uint32_t pulseWidth =
        pulseIn(URM_ECHO_PIN, LOW, 60000UL);

    if (pulseWidth == 0) {
        Serial.println("timeout");
    } else {
        float distanceCm = pulseWidth / 50.0f;

        Serial.printf(
            "pulse = %lu us, distance = %.1f cm\n",
            static_cast<unsigned long>(pulseWidth),
            distanceCm
        );
    }

    delay(200);
}
