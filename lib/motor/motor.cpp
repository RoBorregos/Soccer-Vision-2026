#include "motor.h"
#include <Arduino.h>


Motor::Motor(uint8_t pwm_pin, uint8_t in1, uint8_t in2)
{
    pwm_pin_ = pwm_pin;
    in1_ = in1;
    in2_ = in2;
}

void Motor::InitializeMotor()
{
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    pinMode(pwm_pin_, OUTPUT);
}


void Motor::SetSpeed(float speed)
{
    if (speed >= 0) {
        MovePositive();
    } else {
        MoveNegative();
    }

    int pwm = constrain((int)abs(speed), 0, 255);
    Serial.print("Setting motor speed: ");
    Serial.println(pwm);
    // Cast only here, at the hardware boundary
    analogWrite(pwm_pin_, pwm);
}

void Motor::MovePositive()
{
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, LOW);
}

void Motor::MoveNegative()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, HIGH);
}

void Motor::StopMotor()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
}

uint8_t Motor::GetPwmPin()
{
    return pwm_pin_;
}