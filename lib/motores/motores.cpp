#include "motores.h"
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
    // OJO: ahora ya no existe GetSpeed() (porque era confuso).
    // Si quieres imprimir algo útil, imprime el pin PWM:
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

//This functions are used just to debug in case of error or changes in connections, or electronics
//THey will be deleted in the future, but for now they are useful to check if the motors are working and if the connections are correct
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

void Motors::MoveMotor1() { front_left.MoveForward(); }
void Motors::MoveMotor2() { front_right.MoveForward(); }
void Motors::MoveMotor3() { back_right.MoveForward(); }
void Motors::MoveMotor4() { back_left.MoveForward(); }


//This is the main function for moving the robot in any direction, it calculates the necesary speed and direction of each wheel and adds
//the PID for the robot to move in the desired direction while looking at 0 degreees
void Motors::MoveOmnidirectionalBase(double degree, uint8_t speed, double pid_output)
{
    //degree = degree + 90; // Ajuste para que 0° sea hacia adelante

    float t1 = cos((322.5 + degree) * PI / 180) * speed; // +0.793  → Forward ✓
    float t2 = cos((37.5 + degree) * PI / 180) * speed;  // +0.793  → Forward ✓
    float t3 = cos((142.5 + degree) * PI / 180) * speed; // -0.793  → Backward ✓
    float t4 = cos((217.5 + degree) * PI / 180) * speed; // -0.793  → Backward ✓

    float m1 = t1 + pid_output;
    float m2 = t2 - pid_output;
    float m3 = t3 + pid_output;
    float m4 = t4 - pid_output;  
    
    


    int speedA = constrain(abs((int)m1), 0, 255);
    int speedB = constrain(abs((int)m2 ), 0, 255);
    int speedC = constrain(abs((int)m3), 0, 255);
    int speedD = constrain(abs((int)m4), 0, 255);

    front_left.SetSpeed(speedA);
    front_right.SetSpeed(speedB);
    back_right.SetSpeed(speedC);
    back_left.SetSpeed(speedD);

    (m1 >= 0) ? front_left.MoveForward() : front_left.MoveBackward();
    (m2 >= 0) ? front_right.MoveForward() : front_right.MoveBackward();
    (m3 >= 0) ? back_right.MoveForward() : back_right.MoveBackward();
    (m4 >= 0) ? back_left.MoveForward() : back_left.MoveBackward();
}
