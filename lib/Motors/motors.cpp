#include "motors.h"
#include <Arduino.h>
#include "constantes.h"
#include <cmath>

Motors::Motors(uint8_t pwm_pin1, uint8_t in1_1, uint8_t in2_1,
               uint8_t pwm_pin2, uint8_t in1_2, uint8_t in2_2,
               uint8_t pwm_pin3, uint8_t in1_3, uint8_t in2_3,
               uint8_t pwm_pin4, uint8_t in1_4, uint8_t in2_4)
: front_left(pwm_pin1, in1_1, in2_1),
  front_right(pwm_pin2, in1_2, in2_2),
  back_right(pwm_pin3, in1_3, in2_3),
  back_left(pwm_pin4, in1_4, in2_4)
{}

void Motors::InitializeMotors()
{
    Serial.println("Inicializando motores...");
    front_left.InitializeMotor();
    front_right.InitializeMotor();
    back_right.InitializeMotor();
    back_left.InitializeMotor();
}

void Motors::SetAllSpeeds(uint8_t speed)
{
    front_left.SetSpeed(speed);
    front_right.SetSpeed(speed);
    back_right.SetSpeed(speed);
    back_left.SetSpeed(speed);
}

void Motors::GetAllSpeeds()
{
    Serial.print("Motor 1 PWM pin: ");
    Serial.println(front_left.GetPwmPin());
    Serial.print("Motor 2 PWM pin: ");
    Serial.println(front_right.GetPwmPin());
    Serial.print("Motor 3 PWM pin: ");
    Serial.println(back_right.GetPwmPin());
    Serial.print("Motor 4 PWM pin: ");
    Serial.println(back_left.GetPwmPin());
}

void Motors::StopMotors()
{
    front_left.StopMotor();
    front_right.StopMotor();
    back_right.StopMotor();
    back_left.StopMotor();
}

// This functions are used just to debug in case of error or changes in connections, or electronics
// They will be deleted in the future, but for now they are useful to check if the motors are working and if the connections are correct
void Motors::MoveForward()
{
    front_left.MoveForward();
    front_right.MoveForward();
    back_right.MoveBackward();
    back_left.MoveBackward();
}

void Motors::MoveRight()
{
    front_left.MoveForward();
    front_right.MoveBackward();
    back_right.MoveBackward();
    back_left.MoveForward();
}

void Motors::MoveLeft()
{
    front_left.MoveBackward();
    front_right.MoveForward();
    back_right.MoveForward();
    back_left.MoveBackward();
}

void Motors::MoveBackward()
{
    front_left.MoveBackward();
    front_right.MoveBackward();
    back_right.MoveForward();
    back_left.MoveForward();
}

void Motors::Movefront_left() { front_left.MoveForward(); }
void Motors::Movefront_right() { front_right.MoveForward(); }
void Motors::Moveback_right() { back_right.MoveForward(); }
void Motors::Moveback_left() { back_left.MoveForward(); }

// This is the main function for moving the robot in any direction, it calculates the necessary speed and direction of each wheel and adds
// the PID for the robot to move in the desired direction while looking at 0 degrees
void Motors::MoveOmnidirectionalBase(double degree, uint8_t speed, double pid_output)
{
    // degree = degree + 90; // Ajuste para que 0° sea hacia adelante

    float rawSpeed1 = cos((322.5 + degree) * PI / 180) * speed;
    float rawSpeed2 = cos((37.5 + degree) * PI / 180) * speed;
    float rawSpeed3 = cos((142.5 + degree) * PI / 180) * speed;
    float rawSpeed4 = cos((217.5 + degree) * PI / 180) * speed;

    float speed1 = rawSpeed1 + pid_output;
    float speed2 = rawSpeed2 - pid_output;
    float speed3 = rawSpeed3 + pid_output;
    float speed4 = rawSpeed4 - pid_output;

    int pwm1 = constrain(abs((int)speed1), 0, 255);
    int pwm2 = constrain(abs((int)speed2), 0, 255);
    int pwm3 = constrain(abs((int)speed3), 0, 255);
    int pwm4 = constrain(abs((int)speed4), 0, 255);

    front_left.SetSpeed(pwm1);
    front_right.SetSpeed(pwm2);
    back_right.SetSpeed(pwm3);
    back_left.SetSpeed(pwm4);

    (speed1 >= 0) ? front_left.MoveForward() : front_left.MoveBackward();
    (speed2 >= 0) ? front_right.MoveForward() : front_right.MoveBackward();
    (speed3 >= 0) ? back_right.MoveForward() : back_right.MoveBackward();
    (speed4 >= 0) ? back_left.MoveForward() : back_left.MoveBackward();
}