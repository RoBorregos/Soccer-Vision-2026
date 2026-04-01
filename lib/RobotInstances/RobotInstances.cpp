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

PhotoMux sensors(selectPins, muxPins);  // <- AGREGA ESTA LÍNEA

Kicker kicker(KICKER_PIN, Kick_ball_distance_very_close, Kicker_pulse_ms, Kicker_cooldown_ms);

PhotoMux::Sensor front[8] = {
  {3, 0}, 
  {3, 1}, 
  {3, 2}, 
  {3, 3}, 
  {3, 4}, 
  {3, 5}, 
  {3, 6},
  {3, 7}
};

PhotoMux::Sensor left[8] = {
  {1, 0}, 
  {1, 1}, 
  {1, 2}, 
  {1, 3}, 
  {1, 4}, 
  {1, 5}, 
  {1, 6},
  {1, 7}
};

PhotoMux::Sensor right[8] = {
  {0, 0}, 
  {0, 1}, 
  {0, 2}, 
  {0, 3}, 
  {0, 4}, 
  {0, 5}, 
  {0, 6},
  {0, 7}
};

PhotoMux::Sensor back[8] = {
  {2, 0}, 
  {2, 1}, 
  {2, 2}, 
  {2, 3}, 
  {2, 4}, 
  {2, 5}, 
  {2, 6},
  {2, 7}
};

void initialize_robot() {
  Serial1.begin(115200);
  Serial2.begin(115200);
  bno.InitializeBNO();
  motorss.InitializeMotors();
  pinMode(KICKER_PIN, OUTPUT);
  digitalWrite(KICKER_PIN, LOW);
  analogReadResolution(12);
  //Photosensors
  sensors.begin(); 
  analogReadResolution(12);
  sensors.configureSide(FRONT, front, 8);
  sensors.configureSide(BACK, back, 8);
  sensors.configureSide(LEFT, left, 8);
  sensors.configureSide(RIGHT, right, 8);

  sensors.setThreshold(FRONT, FRONT_THRESHOLD);
  sensors.setThreshold(LEFT,  LEFT_THRESHOLD);
  sensors.setThreshold(RIGHT, RIGHT_THRESHOLD);
  sensors.setThreshold(BACK,  BACK_THRESHOLD);
}