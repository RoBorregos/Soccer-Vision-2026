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
    void SetAllSpeeds(uint8_t speed);
    void GetAllSpeeds();
    void StopMotors();

    // Debug helpers — cardinal directions
    void MoveForward();
    void MoveBackward();
    void MoveRight();
    void MoveLeft();

    // Debug helpers — individual motors
    void Movefront_left();
    void Movefront_right();
    void Moveback_right();
    void Moveback_left();

    // Main omnidirectional movement function
    //   degree     : desired direction of travel in degrees (0 = forward)
    //   speed      : magnitude of motion (0–255)
    //   pid_output : heading correction from the heading PID
    void MoveOmnidirectionalBase(double degree, uint8_t speed, double pid_output);
};

#endif