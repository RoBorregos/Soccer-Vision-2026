#include <Arduino.h>
#include "RobotInstances.h"

//Variables for time management and PID
unsigned long current_time;
unsigned long last_time = 0;
float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle
double last_speed_w = 0.0; //No use for now

//variables for moving side to side when ball is not seen
bool right = true; //true for right, false for left

//Variables for Motor control
const uint8_t Speed = 115; //Robot speedbase

//Variables for avoiding the lines
enum LineSide { LINE_NONE, LINE_FRONT, LINE_LEFT, LINE_RIGHT, LINE_BOTH_SIDES, LINE_FRONT_LEFT, LINE_FRONT_RIGHT, LINE_ALL_SIDES };

unsigned long lineDetectedTime = 0;
bool isAvoidingLine          = false;
LineSide detectedLineSide    = LINE_NONE;

//Variable for angle control in different game situations
float temp_ang = 0;

String serial1_line;
String serial2_line;

//New Variables
float Ball_distance_threshold = 125.0f; // Distance threshold to consider the ball is in front of the robot
float Ball_infront_ang_threshold = 25.0f; // Angle threshold to consider the ball is in front of the robot
float Deadband_4_ballgoalangle = 20.0f; // Deadband for ball-goal angle when the ball is in front
bool isLineDetectedDirect(const uint8_t* pins, uint8_t count, int threshold) {
  long sum = 0;
  for (uint8_t i = 0; i < count; i++) {
    int v = analogRead(pins[i]);
    sum += (long)v * v;
  }
  float avg = sum / float(count);
  return avg > threshold;
}

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
    } else if (frontDetected && leftDetected) {
      detectedLineSide = LINE_FRONT_LEFT;
      Serial.println("LINE DETECTED: FRONT LEFT -> Moving backward and right");
    } else if (frontDetected && rightDetected) {
      detectedLineSide = LINE_FRONT_RIGHT;
      Serial.println("LINE DETECTED: FRONT RIGHT -> Moving backward and left");
    } else if (frontDetected) {
      detectedLineSide = LINE_FRONT;
      Serial.println("LINE DETECTED: FRONT -> Moving backward");
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

bool isBallFront() {
  return frontCam.ball_seen && frontCam.ball_distance < Ball_distance_threshold && fabsf(frontCam.ball_angle) < Ball_infront_ang_threshold;
}

void desired_ang_goal(float goal_ang, float ball_ang) {
  if (goal_ang > 0) {// Goal is on the right
    if (ball_ang < -10.0f) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        temp_ang = ball_ang - 80; //This makes the robot move to the left to align the ball
      }
      else {
        bno.SetTarget(goal_ang + 10.0f);
        temp_ang = goal_ang;
      }
    } else {
      bno.SetTarget(goal_ang);
      temp_ang = goal_ang;
    }
  }
  if (goal_ang < 0) { // Goal is on the left
    if (ball_ang > 10.0f) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        temp_ang = ball_ang + 80; //This makes the robot move to the right to align the ball
      }
    } else {
      bno.SetTarget(goal_ang - 10.0f);
      temp_ang = goal_ang;
    }
  }
  Serial.print("Temp ang is: ");
  Serial.println(temp_ang);
  Serial.print("Ball angle is: ");
  Serial.println(ball_ang);
}

void setup() {
  Serial.begin(115200);
  initialize_robot();
  delay(300);
  bno.GetBNOData();
  setpoint = (float)bno.GetYaw();
  bno.SetTarget(setpoint);
  delay(300);
}

void loop() {
  // Read serial lines from both cameras
  frontCam.read();
  mirrorCam.read();

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

  if (false) {
    motorss.SetAllSpeeds(120);

    switch (detectedLineSide) {
      case LINE_FRONT:
      temp_ang = 180;
      motorss.MoveOmnidirectionalBase((int)temp_ang, 120, speed_w);
      break;

      case LINE_ALL_SIDES:

      case LINE_BOTH_SIDES:
        motorss.MoveBackward();
        break;
      case LINE_FRONT_LEFT:
      temp_ang = 45.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, 120, speed_w);
        break;
      case LINE_FRONT_RIGHT:
      temp_ang = -45.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, 120, speed_w);
        break;
      case LINE_LEFT:
          temp_ang = 90.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, 120, speed_w);
        break;
      case LINE_RIGHT:
          temp_ang = -90.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, 120, speed_w);
        break;
      default:
        motorss.MoveBackward();
        break;
    }

  } else {

    motorss.SetAllSpeeds(Speed);

    if (isBallFront()) {
      desired_ang_goal(frontCam.goal_angle, frontCam.ball_angle);
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      Serial.print("Ball in front, moving with temp_ang: ");
      Serial.println(temp_ang);
      Serial.print("Goal angle: ");
      Serial.println(frontCam.goal_angle);

    } else if (frontCam.ball_seen) {
      bno.SetTarget(0.0f);
      Serial.print("Ball seen front: ");
      float ang = -frontCam.ball_angle;
      if (fabsf(ang) < 6.0f) ang = 0.0f;
      ang = constrain(ang, -90.0f, 90.0f);
      Serial.println(ang);
      Serial.print("Goal angle: ");
      Serial.println(frontCam.goal_angle);
      Serial.print("Ball distance: ");
      Serial.println(frontCam.ball_distance);
      float x = constrain(ang / 90.0, -1.0, 1.0);
      float curved = x * abs(x);
      temp_ang = curved * 90.0;
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);

    } else if (mirrorCam.ball_seen) {
      Serial.print("Ball seen in mirror: ");
      Serial.println(mirrorCam.ball_angle);

      if ((mirrorCam.ball_angle > 45.0f) && (mirrorCam.ball_angle < 135.0f)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle + 30.0), Speed, speed_w);
        Serial.println("Moving right");
      } else if ((mirrorCam.ball_angle < -45.0f) && (mirrorCam.ball_angle > -135.0f)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle - 30.0), Speed, speed_w);
        Serial.println("Moving left");
      } else {
        Serial.println("Ball is behind");
        motorss.MoveOmnidirectionalBase(125, Speed, speed_w);
      }

    } else {
      if (millis() - last_time >= 1000) {
        last_time = millis();
        right = !right;
      }

      if (right) {
        temp_ang = -90.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      } else {
        temp_ang = 90.0f;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      }
    }
  }

  delay(20);
}