#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

  Motors motorss(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);




const uint8_t Speed = 0; //Robot speedbase - reduced for testing

void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  motorss.InitializeMotors();
  Serial.println("Motors initialized");
  motorss.SetAllSpeeds(Speed);
  
}

// Buffers serial
void loop() {
  Serial.println("Testing Motors...");
  //motorss.MoveMotor1();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveMotor2();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveMotor3();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveMotor4();
  //delay(1000);
  //motorss.StopMotors();
  motorss.MoveForward();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveBackward();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveRight();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveLeft();
  //motorss.MoveMotors(0, Speed);
  //motorss.MoveMotorsImu(-90, Speed, 0);

}

