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
float Ball_distance_threshold   = 125.0f; // Distance threshold to consider the ball is in front of the robot
float Ball_infront_ang_threshold = 25.0f; // Angle threshold to consider the ball is in front of the robot
float Deadband_4_ballgoalangle  = 20.0f;  // Deadband for ball-goal angle when the ball is in front

// PID and yaw
const float PID_output_max = 180.0f;
const float PID_output_min = -180.0f;
const float Yaw_zero_glitch_threshold  = 0.001; // Minimum yaw magnitude to consider a real reading
const float Yaw_last_valid_min_change  = 1.0;   // Minimum last yaw magnitude that triggers glitch detection
//Variables for front
const float Ball_front_angle_deadband = 6.0f;
const float Ball_front_angle_clamp = 90.0f;
const float Goal_heading_offset_right =  10.0f; // Added when goal is on the right
const float Goal_heading_offset_left  = -10.0f; // Added when goal is on the left
const float Ball_orbit_offset = 80.0f;
const float Ball_front_min_lateral_angle = 10.0f;

// Mirror camera — angular window where the ball is considered to be on the right or left flank
const float Mirror_ball_right_ang_min =  45.0f;
const float Mirror_ball_right_ang_max = 135.0f;
const float Mirror_ball_left_ang_min  = -45.0f;
const float Mirror_ball_left_ang_max  = -135.0f;

const float Mirror_ball_flank_offset = 30.0f;
const float Mirror_ball_behind_ang = 125.0f;
const unsigned long Search_sweep_interval_ms = 1000;

// Line avoidance movement angles
const float Line_avoid_ang_front      = 180.0f;  // Ball is behind robot, push backward
const float Line_avoid_ang_front_left =  45.0f;  // Diagonal right-backward
const float Line_avoid_ang_front_right = -45.0f; // Diagonal left-backward
const float Line_avoid_ang_left       =  90.0f;  // Strafe right
const float Line_avoid_ang_right      = -90.0f;  // Strafe left

// Line avoidance speed override
const uint8_t Line_avoid_speed = 120;

// Search sweep lateral angles
const float Search_sweep_ang_right = -90.0f;
const float Search_sweep_ang_left  =  90.0f;

// BNO setup delay (ms)
const uint16_t BNO_setup_delay_ms = 300;


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
  return frontCam.ball_seen
      && frontCam.ball_distance < Ball_distance_threshold
      && fabsf(frontCam.ball_angle) < Ball_infront_ang_threshold;
}

void desired_ang_goal(float goal_ang, float ball_ang) {
  if (goal_ang > 0) { // Goal is on the right
    if (ball_ang < -Ball_front_min_lateral_angle) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        temp_ang = ball_ang - Ball_orbit_offset; // Move left to align the ball
      } else {
        bno.SetTarget(goal_ang + Goal_heading_offset_right);
        temp_ang = goal_ang;
      }
    } else {
      bno.SetTarget(goal_ang);
      temp_ang = goal_ang;
    }
  }
  if (goal_ang < 0) { // Goal is on the left
    if (ball_ang > Ball_front_min_lateral_angle) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        temp_ang = ball_ang + Ball_orbit_offset; // Move right to align the ball
      }
    } else {
      bno.SetTarget(goal_ang + Goal_heading_offset_left);
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
  delay(BNO_setup_delay_ms);
  bno.GetBNOData();
  setpoint = (float)bno.GetYaw();
  bno.SetTarget(setpoint);
  delay(BNO_setup_delay_ms);
}

void loop() {
  // Read serial lines from both cameras
  frontCam.read();
  mirrorCam.read();

  // Get current yaw from BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  Serial.println(current_yaw);

  if (fabs(current_yaw) < Yaw_zero_glitch_threshold && fabs(last_valid_yaw) > Yaw_last_valid_min_change) {
    current_yaw = last_valid_yaw;
  } else {
    last_valid_yaw = current_yaw;
  }

  // Obtain error for PID
  double error = bno.GetError();

  // Calculate PID output for angular correction
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, PID_output_min, PID_output_max);

  // Check line sensors — maximum priority
  //checkLineSensors();

  if (false) {
    motorss.SetAllSpeeds(Line_avoid_speed);

    switch (detectedLineSide) {
      case LINE_FRONT:
        temp_ang = Line_avoid_ang_front;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Line_avoid_speed, speed_w);
        break;

      case LINE_ALL_SIDES:
      case LINE_BOTH_SIDES:
        motorss.MoveBackward();
        break;

      case LINE_FRONT_LEFT:
        temp_ang = Line_avoid_ang_front_left;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Line_avoid_speed, speed_w);
        break;

      case LINE_FRONT_RIGHT:
        temp_ang = Line_avoid_ang_front_right;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Line_avoid_speed, speed_w);
        break;

      case LINE_LEFT:
        temp_ang = Line_avoid_ang_left;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Line_avoid_speed, speed_w);
        break;

      case LINE_RIGHT:
        temp_ang = Line_avoid_ang_right;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Line_avoid_speed, speed_w);
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
      if (fabsf(ang) < Ball_front_angle_deadband) ang = 0.0f;
      ang = constrain(ang, -Ball_front_angle_clamp, Ball_front_angle_clamp);
      Serial.println(ang);
      Serial.print("Goal angle: ");
      Serial.println(frontCam.goal_angle);
      Serial.print("Ball distance: ");
      Serial.println(frontCam.ball_distance);
      float x = constrain(ang / Ball_front_angle_clamp, -1.0, 1.0);
      float curved = x * abs(x);
      temp_ang = curved * Ball_front_angle_clamp;
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);

    } else if (mirrorCam.ball_seen) {
      Serial.print("Ball seen in mirror: ");
      Serial.println(mirrorCam.ball_angle);

      if ((mirrorCam.ball_angle > Mirror_ball_right_ang_min) && (mirrorCam.ball_angle < Mirror_ball_right_ang_max)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle + Mirror_ball_flank_offset), Speed, speed_w);
        Serial.println("Moving right");
      } else if ((mirrorCam.ball_angle < Mirror_ball_left_ang_min) && (mirrorCam.ball_angle > Mirror_ball_left_ang_max)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle - Mirror_ball_flank_offset), Speed, speed_w);
        Serial.println("Moving left");
      } else {
        Serial.println("Ball is behind");
        motorss.MoveOmnidirectionalBase(Mirror_ball_behind_ang, Speed, speed_w);
      }

    } else {
      if (millis() - last_time >= Search_sweep_interval_ms) {
        last_time = millis();
        right = !right;
      }

      if (right) {
        temp_ang = Search_sweep_ang_right;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      } else {
        temp_ang = Search_sweep_ang_left;
        motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      }
    }
  }

  delay(20);
}