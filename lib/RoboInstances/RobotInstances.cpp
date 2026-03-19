#include "RobotInstances.h"

BNO055 bno;
PID pid(p, i, d, pid_max_output);

Motors motorss(
  FRONT_LEFT_PWM, FRONT_LEFT_IN1, FRONT_LEFT_IN2,
  FRONT_RIGHT_PWM, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,
  BACK_RIGHT_PWM, BACK_RIGHT_IN1, BACK_RIGHT_IN2,
  BACK_LEFT_PWM, BACK_LEFT_IN1, BACK_LEFT_IN2
);

camera frontCam(Serial1);
camera mirrorCam(Serial2, true);

void initialize_robot() {
  Serial1.begin(115200);
  Serial2.begin(115200);
  bno.InitializeBNO();
  motorss.InitializeMotors();
  analogReadResolution(12);
  }