#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

BNO055 bno;
PID pid(1.6, 0.015, 0.15, 120);

float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle
double last_speed_w = 0.0; //No use for now
const uint8_t Speed = 80; //Robot speedbase

// Front camera data
float f_ball_distance = 0, f_ball_angle = 0;
float f_goal_distance = 0, f_goal_angle = 0;
float f_own_distance  = 0, f_own_angle  = 0;
bool  f_open_ball_seen = false, f_goal_seen = false, f_own_seen = false;

// Mirror camera data
float m_ball_distance = 0, m_ball_angle = 0;
float m_goal_distance = 0, m_goal_angle = 0;
float m_own_distance  = 0, m_own_angle  = 0;
bool  m_open_ball_seen = false, m_goal_seen = false, m_own_seen = false;

Motors motorss(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

// Buffers serial
String serial1_line;
String serial2_line;

//Processing serial data from front camera
void process_serialF(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    f_ball_distance = dist;   f_ball_angle = ang;
    f_goal_distance = g_dist; f_goal_angle = g_ang;
    f_own_distance  = o_dist; f_own_angle  = o_ang;

    f_open_ball_seen = (fabsf(dist)   > 1e-3f);
    f_goal_seen      = (fabsf(g_dist) > 1e-3f);
    f_own_seen       = (fabsf(o_dist) > 1e-3f);
  }
}

//Processing serial data from mirror camera
void process_serialM(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    m_ball_distance = dist;   m_ball_angle = ang;
    m_goal_distance = g_dist; m_goal_angle = g_ang;
    m_own_distance  = o_dist; m_own_angle  = o_ang;

    m_open_ball_seen = (fabsf(ang)    > 45.0f); //Ajuste para cámara espejo
    m_goal_seen      = (fabsf(g_dist) > 1e-3f);
    m_own_seen       = (fabsf(o_dist) > 1e-3f);
  }
}

//reading serial lines
void readSerialLines() {
  //front camera por Serial1
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      process_serialF(serial1_line);
      serial1_line = "";
    } else {
      serial1_line += c;
      if (serial1_line.length() > 120) serial1_line = "";
    }
  }

  //mirror camera por Serial2
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      process_serialM(serial2_line);
      serial2_line = "";
    } else {
      serial2_line += c;
      if (serial2_line.length() > 80) serial2_line = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  bno.InitializeBNO();
  motorss.InitializeMotors();

  delay(300);
  bno.GetBNOData();
  setpoint = (float)bno.GetYaw();
  bno.SetTarget(setpoint);
  delay(300);
}

void loop() {
  //Read serial lines from both cameras
  readSerialLines();
  
  //Get current yaw from BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  Serial.println(current_yaw);

  if (fabs(current_yaw) < 0.001 && fabs(last_valid_yaw) > 1.0) {
    current_yaw = last_valid_yaw;
    Serial.println(current_yaw);
  } else {
    last_valid_yaw = current_yaw;
  }

  //Obtain error for PID
  double error = bno.GetError();

  //Calculate PID output for angular correction
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, -180, 180);

  //if the ball is seen with FCamera robot moves with PID
  if (f_open_ball_seen) {
    Serial.print("Ball seen front: ");
    Serial.print(-f_ball_angle);
    float ang = -f_ball_angle;
    if (fabsf(ang) < 7.0f) ang = 0.0f;
    ang = constrain(ang, -90.0f, 90.0f);
    Serial.println(ang);
    motorss.MoveMotorsImu((int)ang, Speed, 0);

  //If ball not seen with FCamera Mcamera is used
  } else if (m_open_ball_seen) {
    Serial.print("Ball seen in mirror");
    Serial.println(m_ball_angle);
    if (m_ball_angle > 45.0f) {
      double m_ang = m_ball_angle + 15.0;
      motorss.MoveMotorsImu((int)m_ang, 90, speed_w);
      Serial.println("Moving right");
    } else if (m_ball_angle < -40.0f) {
      double m_ang = m_ball_angle - 15.0;
      motorss.MoveMotorsImu((int)m_ang, 90, speed_w);
      Serial.println("Moving left");
    }

  } else {
    motorss.MoveMotorsImu(0, 0, speed_w);
  }

  delay(20);
}