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
    front_left.MovePositive();
    front_right.MovePositive();
    back_right.MovePositive();
    back_left.MovePositive();
}

void Motors::MoveRight()
{
    front_left.MovePositive();
    front_right.MoveNegative();
    back_right.MoveNegative();
    back_left.MovePositive();
}

void Motors::MoveLeft()
{
    front_left.MoveNegative();
    front_right.MovePositive();
    back_right.MovePositive();
    back_left.MoveNegative();
}

void Motors::MoveBackward()
{
    front_left.MoveNegative();
    front_right.MoveNegative();
    back_right.MovePositive();
    back_left.MovePositive();
}

void Motors::Movefront_left() { front_left.MovePositive(); }
void Motors::Movefront_right() { front_right.MovePositive(); }
void Motors::Moveback_right() { back_right.MovePositive(); }
void Motors::Moveback_left() { back_left.MovePositive(); }

// This is the main function for moving the robot in any direction, it calculates the necessary speed and direction of each wheel and adds
// the PID for the robot to move in the desired direction while looking at 0 degrees
void Motors::MoveOmnidirectionalBase(double degree, int16_t speed, double pid_output)
{
    // degree = degree + 90; // Ajuste para que 0° sea hacia adelante

    float Speed_Front_Left = (cos((322.5 + degree) * PI / 180) * speed) + pid_output;
    float Speed_Front_Right = (cos((37.5 + degree) * PI / 180) * speed) - pid_output;
    float Speed_Back_Right = (cos((142.5 + degree) * PI / 180) * speed) + pid_output;
    float Speed_Back_Left = (cos((217.5 + degree) * PI / 180) * speed) - pid_output;

    Serial.print("Calculated Speeds - FL: ");
    Serial.print(Speed_Front_Left);
    Serial.print(", FR: ");
    Serial.print(Speed_Front_Right);
    Serial.print(", BR: ");
    Serial.print(Speed_Back_Right);
    Serial.print(", BL: ");
    Serial.println(Speed_Back_Left);
    


    front_left.SetSpeed(Speed_Front_Left);
    front_right.SetSpeed(Speed_Front_Right);
    back_right.SetSpeed(Speed_Back_Right);
    back_left.SetSpeed(Speed_Back_Left);
}