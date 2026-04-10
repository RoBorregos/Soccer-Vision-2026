#ifndef MOTOR_H
#define MOTOR_H
#pragma once

#include <Arduino.h>

class Motor
{
public:
    // Constructor: pin PWM + pines de dirección
    Motor(uint8_t pwm_pin, uint8_t in1, uint8_t in2);

    void InitializeMotor();

    // Control de movimiento
    void MovePositive();
    void MoveNegative();
    void StopMotor();

    // Control de velocidad (-255 to 255, sign controls direction)
    void SetSpeed(int16_t speed);

    // Getters útiles para debug
    uint8_t GetPwmPin();
    uint8_t GetIn1();
    uint8_t GetIn2();

private:
    uint8_t pwm_pin_;
    uint8_t in1_;
    uint8_t in2_;
};

#endif