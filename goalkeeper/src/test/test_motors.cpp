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

BNO085 bno;
PID pid(p, i, d, pid_max_output);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  motorss.InitializeMotors();
  bno.InitializeBNO();
  Serial.println("Motors initialized");
}

void loop() {
  Serial.println("Testing Motors...");

  bno.GetBNOData();
  double error   = bno.GetError();
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, PID_output_min, PID_output_max);

  motorss.MoveOmnidirectionalBase(-90, Speed, speed_w);
}
