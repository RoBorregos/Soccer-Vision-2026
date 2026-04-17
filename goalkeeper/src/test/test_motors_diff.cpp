#include <Arduino.h>
#include "motors.h"
#include "constantes.h"

// Change this to set the speed of the test (-255 to 255, sign controls direction)
const int16_t TEST_SPEED = 50;

// Time each motor runs individually (ms)
const unsigned long RUN_MS  = 3000;
// Pause between motors (ms)
const unsigned long PAUSE_MS = 1500;

Motors motorss(
  FRONT_LEFT_PWM,  FRONT_LEFT_IN1,  FRONT_LEFT_IN2,
  FRONT_RIGHT_PWM, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,
  BACK_RIGHT_PWM,  BACK_RIGHT_IN1,  BACK_RIGHT_IN2,
  BACK_LEFT_PWM,   BACK_LEFT_IN1,   BACK_LEFT_IN2
);

void runOne(const char* name, Motor& m) {
  Serial.print("Running motor: ");
  Serial.print(name);
  Serial.print(" at speed ");
  Serial.println(TEST_SPEED);

  m.SetSpeed(TEST_SPEED);
  delay(RUN_MS);
  m.StopMotor();

  Serial.print("Stopped motor: ");
  Serial.println(name);
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(115200);
  motorss.InitializeMotors();
  motorss.StopMotors();
  Serial.println("test_motors_diff ready");
  delay(1000);
}

void loop() {
  Serial.println("---- Cycle start ----");
  runOne("FRONT_LEFT",  motorss.front_left);
  runOne("FRONT_RIGHT", motorss.front_right);
  runOne("BACK_RIGHT",  motorss.back_right);
  runOne("BACK_LEFT",   motorss.back_left);
  Serial.println("---- Cycle end ----");
  delay(2000);
}
