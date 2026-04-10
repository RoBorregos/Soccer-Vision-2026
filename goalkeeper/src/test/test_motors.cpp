#include <Arduino.h>
#include <cmath>
#include "motors.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

Motors motorss(
  FRONT_LEFT_PWM, FRONT_LEFT_IN1, FRONT_LEFT_IN2,
  FRONT_RIGHT_PWM, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,
  BACK_RIGHT_PWM, BACK_RIGHT_IN1, BACK_RIGHT_IN2,
  BACK_LEFT_PWM, BACK_LEFT_IN1, BACK_LEFT_IN2
);



<<<<<<<< HEAD:goalkeeper/src/test_motors.cpp

========
>>>>>>>> 277e46f7614aaeaed663df8a479c556cabdecd34:goalkeeper/src/test/test_motors.cpp
void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  motorss.InitializeMotors();
  Serial.println("Motors initialized");
  
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
  //motorss.MoveForward();
<<<<<<<< HEAD:goalkeeper/src/test_motors.cpp
  delay(1000);
========
  //delay(1000);
>>>>>>>> 277e46f7614aaeaed663df8a479c556cabdecd34:goalkeeper/src/test/test_motors.cpp
  //motorss.StopMotors();
  //motorss.MoveBackward();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveRight();
  //delay(1000);
  //motorss.StopMotors();
  //motorss.MoveLeft();
  //delay(1000);
  //motorss.MoveMotors(0, Speed);
  motorss.MoveOmnidirectionalBase(0, Speed, 0);

}
