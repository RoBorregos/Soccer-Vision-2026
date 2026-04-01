#include <Arduino.h>
#include "RobotInstances.h"

//Variables for time management and PID
unsigned long current_time;
unsigned long last_time = 0;
float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle


//variables for moving side to side when ball is not seen
bool sweepRight = true; //true for right, false for left



unsigned long lineDetectedTime = 0; //ms that line is detected
bool isAvoidingLine          = false; //Boolean value for detecting line
LineSide detectedLineSide    = LINE_NONE;

//Variable for angle control in different game situations
float temp_ang = 0;

String serial1_line;
String serial2_line;

//New Variables



//Kicker variables
bool kicker_active = false; //Boolean value to activate the kicker
unsigned long kicker_pulse_start = 0; //ms that the kicker was activated
unsigned long last_kick_time = 0; //ms that the last kick was performed, used for cooldown management

//Function that calls a boolean method of class sensors, stores it in variable, possible cases for line detection and time management for line avoidance
void checkLineSensors() {
  bool frontDetected = sensors.isLineDetected(FRONT);
  bool leftDetected  = sensors.isLineDetected(LEFT);
  bool rightDetected = false;
  bool backDetected  = sensors.isLineDetected(BACK);

  if (frontDetected || leftDetected || rightDetected || backDetected) {

    lineDetectedTime = millis();
    isAvoidingLine   = true;

    if (frontDetected && leftDetected && rightDetected) {
      detectedLineSide = LINE_ALL_SIDES;
      Serial.println("All Lines Detected");
    } else if (leftDetected && rightDetected) {
      detectedLineSide = LINE_BOTH_SIDES;
      Serial.println("Left and Right Lines Detected");
    } else if (frontDetected && leftDetected) {
      detectedLineSide = LINE_FRONT_LEFT;
      Serial.println("Front and Left Lines Detected");
    } else if (frontDetected && rightDetected) {
      detectedLineSide = LINE_FRONT_RIGHT;
      Serial.println("Front and Right Lines Detected");
    } else if (frontDetected) {
      detectedLineSide = LINE_FRONT;
      Serial.println("Front Line Detected");
    } else if (leftDetected) {
      detectedLineSide = LINE_LEFT;
      Serial.println("Left Line Detected");
    } else if (rightDetected) {
      detectedLineSide = LINE_RIGHT;
      Serial.println("Right Line Detected");
    } else if (backDetected) {
      detectedLineSide = LINE_BACK;
      Serial.println("Back Line Detected");
    }
  } else if (isAvoidingLine) {
    if (millis() - lineDetectedTime >= correctionTime) {
      isAvoidingLine   = false;
      detectedLineSide = LINE_NONE;
    }
  }
}

//Function that checks if the ball is in front of the robot using the front camera, distance and angle.
bool isBallFront() {
  return frontCam.ball_seen
      && frontCam.ball_distance < Ball_distance_threshold
      && fabsf(frontCam.ball_angle) < Ball_infront_ang_threshold;
}

//Checks if the ball is close enough to kick
bool canKickNow() {
  return frontCam.ball_seen
  && frontCam.ball_distance < Kick_ball_distance_very_close;
}

//Manages kicker pulse timing and cooldown to fire the kicker when the ball is in range
void updateKicker() {
  unsigned long now = millis();

  if (kicker_active) {
    if (now - kicker_pulse_start >= Kicker_pulse_ms) {
      digitalWrite(KICKER_PIN, LOW);
      kicker_active = false;
      last_kick_time = now;
      Serial.println("Kicker OFF");
    }
    return;
  }

  if (canKickNow() && (now - last_kick_time >= Kicker_cooldown_ms)) {
    digitalWrite(KICKER_PIN, HIGH);
    kicker_active = true;
    kicker_pulse_start = now;
    Serial.println("Kicker ON");
  }
}

//Function to determine the desired angle based on the goal angle and ball angle, with different logic depending on whether the goal is on the right or left. It also includes an orbiting behavior around the ball when it's in front of the robot but not aligned with the goal.
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
  updateKicker();

  // Get current yaw from BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  Serial.print("Yaw: ");
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
  checkLineSensors();

  if (isAvoidingLine) {
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

      case LINE_BACK:
        temp_ang = Line_avoid_ang_back;
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
        sweepRight = !sweepRight;
      }

      if (sweepRight) {
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