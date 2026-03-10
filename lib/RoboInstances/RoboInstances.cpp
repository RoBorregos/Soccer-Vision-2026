#include "RoboInstances.h"

//PID constants definitions
float p = 1.6;
float i = 0.015;
float d = 0.15;
float pid_max_output = 120.0;

BNO055 bno;
PID pid(p, i, d, pid_max_output);

Motors motorss(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

PhotoMux photoMux(selectPins, muxPins);

camera frontCam(Serial1);
camera mirrorCam(Serial2, true);

PhotoMux::Sensor front[] = {
  {0, 39},
  {0, 40},
  {0, 41},
  {0, 20}
};
