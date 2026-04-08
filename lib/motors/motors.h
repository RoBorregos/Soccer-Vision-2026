#ifndef motors_h
#define motors_h

#include "Arduino.h"
#include "motor.h"
#include "constantes.h"

class Motors
{
public:
    Motor front_left;
    Motor front_right;
    Motor back_right;
    Motor back_left;

    Motors(uint8_t speed_front_left, uint8_t in1_front_left, uint8_t in2_front_left,
           uint8_t speed_front_right, uint8_t in1_front_right, uint8_t in2_front_right,
           uint8_t speed_back_right, uint8_t in1_back_right, uint8_t in2_back_right,
           uint8_t speed_back_left, uint8_t in1_back_left, uint8_t in2_back_left);

    void InitializeMotors();
    void SetSpeed(uint8_t pwm, uint8_t speed);
    void SetAllSpeeds(uint8_t speed);
    void StopMotors();
    void MoveForward();
    void MoveBackward();
    void MoveRight();
    void MoveLeft();
    void Movefront_left();
    void Movefront_right();
    void Moveback_right();
    void Moveback_left();
    void MoveMotors(int degree, uint8_t speed);
    void GetAllSpeeds();
    void MoveOmnidirectionalBase(double degree, uint8_t speed, double pid_output);
};

#endif