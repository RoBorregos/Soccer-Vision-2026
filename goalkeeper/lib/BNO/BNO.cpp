#include "Arduino.h"
#include "BNO.h"
#include "cmath"

// Adafruit BNO08x (BNO085) driver using game rotation vector for drift-free yaw

BNO085::BNO085()
{
    yaw_ = 0;
    raw_yaw_ = 0;
    target_angle_ = 0;
    difference_angle_ = 0;
    yaw_offset_ = 0;
}

bool BNO085::SetReports()
{
    if (!bno_.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
        Serial.println("ERROR: could not enable game rotation vector");
        return false;
    }
    return true;
}

void BNO085::InitializeBNO()
{
    Serial.println("Initializing BNO085...");
    Serial.println("BNO08x I2C Address: 0x4A");
    Serial.println("Teensy 4.1 SDA: pin 18, SCL: pin 19");

    if (!bno_.begin_I2C(0x4A, &Wire))
    {
        Serial.println("ERROR: BNO085 not detected at 0x4A!");
        Serial.println("Check:");
        Serial.println("  - I2C wiring (SDA/SCL)");
        Serial.println("  - Pull-up resistors on SDA/SCL");
        Serial.println("  - SA0 pin tied low for 0x4A");
        while (1);
    }
    delay(100);

    if (!SetReports()) {
        while (1);
    }
    delay(300);

    unsigned long t0 = millis();
    while (millis() - t0 < 500) {
        GetBNOData();
        if (raw_yaw_ != 0) break;
        delay(10);
    }

    setpoint = GetYaw();
    SetTarget(setpoint);
    delay(300);
    Serial.println("BNO085 initialized successfully!");
}

double BNO085::NormalizeAngle(double angle)
{
    while (angle > 180.0) {
        angle -= 360.0;
    }
    while (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

void BNO085::GetBNOData()
{
    if (bno_.wasReset()) {
        SetReports();
    }

    while (bno_.getSensorEvent(&sensor_value_)) {
        if (sensor_value_.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qr = sensor_value_.un.gameRotationVector.real;
            float qi = sensor_value_.un.gameRotationVector.i;
            float qj = sensor_value_.un.gameRotationVector.j;
            float qk = sensor_value_.un.gameRotationVector.k;

            float yaw_rad = atan2f(2.0f * (qr * qk + qi * qj),
                                   1.0f - 2.0f * (qj * qj + qk * qk));
            raw_yaw_ = yaw_rad * 180.0 / PI;
            yaw_ = NormalizeAngle(raw_yaw_);
        }
    }
}

double BNO085::GetYaw()
{
    return -yaw_;
}

void BNO085::SetTarget(double target) {
    target_angle_ = NormalizeAngle(target);
}

double BNO085::GetError() {
    double error = target_angle_ - yaw_;
    error = NormalizeAngle(error);
    return error;
}
