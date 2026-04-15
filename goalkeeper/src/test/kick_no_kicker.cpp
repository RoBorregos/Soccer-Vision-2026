#include <Arduino.h>
#include "RobotInstances.h"

bool ready_2_shoot = false;

// ─── Edit these to tune the kick ─────────────────────────────────────────────
float test_ang          = 0.0f;
int   test_speed        = Speed;
unsigned long kick_duration = 300; // ms
// ─────────────────────────────────────────────────────────────────────────────

// Kick state machine
enum KickState { KICK_IDLE, KICK_FORWARD, KICK_BACK };
KickState kickState   = KICK_IDLE;
unsigned long kickStartMs = 0;

void kick_without_kicker(float ang, int speed, double speed_w) {
  unsigned long now = millis();

  switch (kickState) {
    case KICK_IDLE:
      if (ready_2_shoot) {
        motorss.MoveOmnidirectionalBase(0, speed, speed_w);
        kickStartMs   = now;
        kickState     = KICK_FORWARD;
        ready_2_shoot = false;
      }
      break;

    case KICK_FORWARD:
      if (now - kickStartMs >= kick_duration) {
        motorss.MoveOmnidirectionalBase(180, speed, speed_w);
        kickStartMs = now;
        kickState   = KICK_BACK;
      }
      break;

    case KICK_BACK:
      if (now - kickStartMs >= kick_duration) {
        motorss.MoveOmnidirectionalBase(0, 0, speed_w);
        kickState = KICK_IDLE;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  initialize_robot();
  ready_2_shoot = true;
}

void loop() {
  bno.GetBNOData();
  double error   = bno.GetError();
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, PID_output_min, PID_output_max);

  kick_without_kicker(test_ang, test_speed, speed_w);
}