#include <Arduino.h>
#include "RoboInstances.h"

//Variables for time management and PID
unsigned long current_time;
unsigned long last_time = 0;
float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle
double last_speed_w = 0.0; //No use for now

//variables for moving side to side when ball is not seen
bool side = true; //true for right, false for left

//Variables for Motor control
const uint8_t Speed = 120; //Robot speedbase

//Variables for avoiding the lines
enum LineSide { LINE_NONE, LINE_FRONT, LINE_LEFT, LINE_RIGHT, LINE_BOTH_SIDES, LINE_FRONT_LEFT, LINE_FRONT_RIGHT, LINE_ALL_SIDES };

unsigned long lineDetectedTime = 0;
bool isAvoidingLine          = false;
LineSide detectedLineSide    = LINE_NONE;

//Variable for angle control in different game situations
float temp_ang = 0;

// Camera boolean states (front)
float f_ball_distance = 0, f_ball_angle = 0;
float f_goal_distance = 0, f_goal_angle = 0;
float f_own_distance  = 0, f_own_angle  = 0;
bool  f_ball_seen = false, f_goal_seen = false, f_own_seen = false;



// Camera boolean states (mirror)
float m_ball_distance = 0, m_ball_angle = 0;
float m_goal_distance = 0, m_goal_angle = 0;
float m_own_distance  = 0, m_own_angle  = 0;
bool  m_ball_seen = false, m_goal_seen = false, m_own_seen = false;

String serial1_line;
String serial2_line;

// ── Serial parsing ─────────────────────────────────────────────────────────────
void process_serialF(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    f_ball_distance = dist;   f_ball_angle = ang;
    f_goal_distance = g_dist; f_goal_angle = -g_ang;
    f_own_distance  = o_dist; f_own_angle  = o_ang;
    f_ball_seen = (fabsf(dist)   > 1e-3f);
    f_goal_seen = (fabsf(g_dist) > 1e-3f);
    f_own_seen  = (fabsf(o_dist) > 1e-3f);
  }
}

void process_serialM(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    m_ball_distance = dist;   m_ball_angle = ang;
    m_goal_distance = g_dist; m_goal_angle = g_ang;
    m_own_distance  = o_dist; m_own_angle  = o_ang;
    m_goal_angle += 25.0f;
    m_ball_seen = (fabsf(ang)    > 1e-3f);
    m_goal_seen = (fabsf(g_dist) > 1e-3f);
    m_own_seen  = (fabsf(o_dist) > 1e-3f);
  }
}

void readSerialLines() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') { process_serialF(serial1_line); serial1_line = ""; }
    else { serial1_line += c; if (serial1_line.length() > 120) serial1_line = ""; }
  }
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') { process_serialM(serial2_line); serial2_line = ""; }
    else { serial2_line += c; if (serial2_line.length() > 80) serial2_line = ""; }
  }
}

//This function reads the phototransistor values directly and determines if a line is detected based on the average of the squared readings compared to a threshold. This method is more efficient than using the PhotoMux class and provides a more direct way to check for line detection.
bool isLineDetectedDirect(const uint8_t* pins, uint8_t count, int threshold) {
  long sum = 0;
  for (uint8_t i = 0; i < count; i++) {
    int v = analogRead(pins[i]);
    sum += (long)v * v;
  }
  float avg = sum / float(count);
  return avg > threshold;
}

//This function checks the line sensors and updates the robot's state accordingly. If a line is detected, it sets the appropriate flags and determines which side the line is on. It also handles the logic for exiting line avoidance mode after a certain amount of time has passed.
void checkLineSensors() {
  bool frontDetected = isLineDetectedDirect(FRONT_PINS, FRONT_COUNT, FRONT_THRESHOLD);
  bool leftDetected  = isLineDetectedDirect(LEFT_PINS,  LEFT_COUNT,  LEFT_THRESHOLD);
  bool rightDetected = isLineDetectedDirect(RIGHT_PINS, RIGHT_COUNT, RIGHT_THRESHOLD);

  if (frontDetected || leftDetected || rightDetected) {
    lineDetectedTime = millis();
    isAvoidingLine   = true;

    if (frontDetected && leftDetected && rightDetected) {
      detectedLineSide = LINE_ALL_SIDES;
      Serial.println("LINE DETECTED: ALL SIDES -> Moving backward");
    } else if (leftDetected && rightDetected) {
      detectedLineSide = LINE_BOTH_SIDES;
      Serial.println("LINE DETECTED: BOTH SIDES -> Moving backward");
    } else if (frontDetected) {
      detectedLineSide = LINE_FRONT;
      Serial.println("LINE DETECTED: FRONT -> Moving backward");
    } else if (frontDetected && leftDetected) {
      detectedLineSide = LINE_FRONT_LEFT;
      Serial.println("LINE DETECTED: FRONT LEFT -> Moving backward and right");
    } else if (frontDetected && rightDetected) {
      detectedLineSide = LINE_FRONT_RIGHT;
      Serial.println("LINE DETECTED: FRONT RIGHT -> Moving backward and left");
    } else if (leftDetected) {
      detectedLineSide = LINE_LEFT;
      Serial.println("LINE DETECTED: LEFT -> Moving right");
    } else if (rightDetected) {
      detectedLineSide = LINE_RIGHT;
      Serial.println("LINE DETECTED: RIGHT -> Moving left");
    }
  } else if (isAvoidingLine) {
    if (millis() - lineDetectedTime >= correctionTime) {
      isAvoidingLine   = false;
      detectedLineSide = LINE_NONE;
      Serial.println("LINE ESCAPE END");
    }
  }
}

//This function checks if the ball is close and infront of the robot, to start different strategies
//It uses the aproximate distance and the angle to determine this
bool isBallFront() {
  return f_ball_seen && f_ball_distance < 125 && fabsf(f_ball_angle) < 30;
}

//This function is used, to check if the ball and the goal are aligned, if they are not it tries to align them
//After the alignement, it uses temp_ang to move the robot towards the ball while multyplying to make a slight curve
void desired_ang_goal(float g_ang, float b_ang) {
  //If ang goal is positive (right)
  if (g_ang > 0) {
    if (b_ang < 10) { //Check if goal angle and ball angle are aligned
      temp_ang = b_ang - 30; //If not, adjust temp_ang based on ball distanc
    } else {
      temp_ang = b_ang ; //If aligned, adjust temp_ang based on ball distance
    }
  }
  //If ang goal is negative (left)
  if (g_ang < 0) {
    if (b_ang > 10) {
      temp_ang = b_ang + 80;
    } else {
      temp_ang = b_ang ;

    }
  }
  Serial.print("Temp ang is: ");
  Serial.println(temp_ang);
}

//This function is DEADCODE (for now)
//It was intended to move the robot using the mirror

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
  analogReadResolution(12);
}

void loop() {
  // Read serial lines from both cameras
  readSerialLines();

  // Get current yaw from BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  Serial.println(current_yaw);

  if (fabs(current_yaw) < 0.001 && fabs(last_valid_yaw) > 1.0) {
    current_yaw = last_valid_yaw;
  } else {
    last_valid_yaw = current_yaw;
  }

  // Obtain error for PID
  double error = bno.GetError();

  // Calculate PID output for angular correction
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, -180, 180);

  // Check line sensors — maximum priority
  checkLineSensors();

  if (isAvoidingLine) {
    // LINE AVOIDANCE — overrides everything
    motorss.SetAllSpeeds(120);
    
    switch (detectedLineSide) {
      case LINE_FRONT:
      case LINE_ALL_SIDES:

      case LINE_BOTH_SIDES:
        motorss.MoveBackward();
        break;
      case LINE_FRONT_LEFT:
        motorss.MoveOmnidirectionalBase(45, 120, speed_w);
        break;
      case LINE_FRONT_RIGHT:
        motorss.MoveOmnidirectionalBase(-45, 120, speed_w);
        break;
      default:
      case LINE_LEFT:
        motorss.MoveOmnidirectionalBase(90, 120, speed_w);
        break;
      case LINE_RIGHT:
        motorss.MoveOmnidirectionalBase(-90, 120, speed_w);
        break;
        motorss.MoveBackward();
        break;
    }

  } else {

    motorss.SetAllSpeeds(Speed);

    if (false) {
      // Ball is close and centered: aim toward goal
      desired_ang_goal(f_goal_angle, f_ball_angle);
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      Serial.print("Ball in front, moving with temp_ang: ");
      Serial.println(temp_ang);
      Serial.print("Goal angle: ");
      Serial.println(f_goal_angle);

    }
    else if (f_ball_seen) {
      // Ball visible from front camera
      Serial.print("Ball seen front: ");
      float ang = -f_ball_angle;
      if (fabsf(ang) < 6.0f) ang = 0.0f;
      ang = constrain(ang, -90.0f, 90.0f);
      Serial.println(ang);
      Serial.print("Goal angle: ");
      Serial.println(f_goal_angle);

      Serial.print("Ball distance: ");
      Serial.println(f_ball_distance);
      float x = constrain(f_ball_angle / 90.0, -1.0, 1.0);
      float curved = x * abs(x);
      temp_ang = curved * 90.0;
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);

    } else if (m_ball_seen) {
      // Ball visible from mirror camera
      Serial.print("Ball seen in mirror: ");
      Serial.println(m_ball_angle);

      if ((m_ball_angle > 45.0f) && (m_ball_angle < 135.0f)) {
        motorss.MoveOmnidirectionalBase((int)(m_ball_angle + 38.0), Speed, speed_w);
        Serial.println("Moving right");
      } else if ((m_ball_angle < -45.0f) && (m_ball_angle > -135.0f)) {
        motorss.MoveOmnidirectionalBase((int)(m_ball_angle - 38.0), Speed, speed_w);
        Serial.println("Moving left");
      } else {
        Serial.println("Ball is behind");
        motorss.MoveOmnidirectionalBase(145, Speed, speed_w);
      }

    } else {
      // Ball not seen anywhere — sweep side to side
      if (millis() - last_time >= 1000) {
        last_time = millis();
        side = !side;
      }

      if (side) {
        motorss.MoveOmnidirectionalBase(-90, Speed, speed_w);
      } else {
        motorss.MoveOmnidirectionalBase(90, Speed, speed_w);
      }
    }
  }

  delay(20);
}