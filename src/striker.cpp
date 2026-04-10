#include <Arduino.h>
#include "RobotInstances.h"

//Variables for time management and PID
unsigned long current_time;
unsigned long last_time = 0;
float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle

//Movement Variables
bool sweepRight = true; //true for right, false for left

//PhotoMux Variables
unsigned long lineDetectedTime = 0; //ms that line is detected
bool isAvoidingLine          = false; //Boolean value for detecting line
LineSide detectedLineSide    = LINE_NONE; //Variable to store which line is detected, possible values defined in constantes.h

//Variable for angle control in different game situations
float temp_ang = 0;

//Variables for communication with cameras
String serial1_line;
String serial2_line;

bool ready_2_shoot = false;

//Function that calls a boolean method of class sensors, stores it in variable, possible cases for line detection and time management for line avoidance
void checkLineSensors() {
  bool frontDetected = phototransistors.isLineDetected(FRONT);
  bool leftDetected  = phototransistors.isLineDetected(LEFT);
  bool rightDetected = phototransistors.isLineDetected(RIGHT);
  bool backDetected  = phototransistors.isLineDetected(BACK);

  if (frontDetected || leftDetected || rightDetected || backDetected) {

    lineDetectedTime = millis();
    isAvoidingLine   = true;

    if (frontDetected && leftDetected && rightDetected) {
      detectedLineSide = LINE_ALL_SIDES;

    } else if (leftDetected && rightDetected) {
      detectedLineSide = LINE_BOTH_SIDES;
    } else if (frontDetected && leftDetected) {
      detectedLineSide = LINE_FRONT_LEFT;
    } else if (frontDetected && rightDetected) {
      detectedLineSide = LINE_FRONT_RIGHT;
    } else if (frontDetected) {
      detectedLineSide = LINE_FRONT;
    } else if (leftDetected) {
      detectedLineSide = LINE_LEFT;
    } else if (rightDetected) {
      detectedLineSide = LINE_RIGHT;
    } else if (backDetected) {
      detectedLineSide = LINE_BACK;
    }

  } else if (isAvoidingLine) {
    if (millis() - lineDetectedTime >= correctionTime) {
      isAvoidingLine   = false;
      detectedLineSide = LINE_NONE;
    }
  }
  if (debug_line_sensors) {
    Serial.println("=== Line Detection ====");
    Serial.print("Line sensors - Front: ");
    Serial.print(frontDetected);
    Serial.print(", Left: ");
    Serial.print(leftDetected);
    Serial.print(", Right: ");
    Serial.print(rightDetected);
    Serial.print(", Back: ");
    Serial.print(backDetected);
    Serial.print(" | Detected Line Side: ");
    Serial.println(detectedLineSide);
  }
}

//Function that checks if the ball is in front of the robot using the front camera, distance and angle.
bool isBallFront() {
  return frontCam.ball_seen
      && frontCam.ball_distance < Ball_distance_threshold
      && fabsf(frontCam.ball_angle) < Ball_infront_ang_threshold;
}


//Function to determine the desired angle based on the goal angle and ball angle, with different logic depending on whether the goal is on the right or left. It also includes an orbiting behavior around the ball when it's in front of the robot but not aligned with the goal.
void desired_ang_goal(float goal_ang, float ball_ang) {
  Robot_Mode_Infront currentMode; 
  if (goal_ang > 0) { // Goal is on the right
    if (ball_ang < -Ball_front_min_lateral_angle) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        currentMode = Aligning_with_goal_right;
        temp_ang = ball_ang - Ball_orbit_offset; // Move left to align the ball
      } else {
        
        bno.SetTarget(goal_ang + Goal_heading_offset_right);
        temp_ang = goal_ang;
        currentMode = Moving_towards_goal;
      }
    } else {
      bno.SetTarget(goal_ang);
      temp_ang = goal_ang;
      currentMode = Moving_towards_goal;
    }
  }
  if (goal_ang < 0) { // Goal is on the left
    if (ball_ang > Ball_front_min_lateral_angle) {
      if (fabsf(goal_ang - ball_ang) > Deadband_4_ballgoalangle) {
        temp_ang = ball_ang + Ball_orbit_offset; // Move right to align the ball
        currentMode = Aligning_with_goal_left;
      } else {
        bno.SetTarget(goal_ang + Goal_heading_offset_left);
        temp_ang = goal_ang;
        currentMode = Moving_towards_goal;
      }
    } else {
      bno.SetTarget(goal_ang + Goal_heading_offset_left);
      temp_ang = goal_ang;
      currentMode = Moving_towards_goal;
    }
  }

  
  if (debug_ball_infront) {

    Serial.print(F("==== Mode: ===="));
    switch (currentMode) {
      case Aligning_with_goal_right: Serial.println(("Aligning with goal right")); break;
      case Aligning_with_goal_left:  Serial.println(("Aligning with goal left"));  break;
      case Moving_towards_goal:      Serial.println(("Moving towards goal"));       break;
    }
    }
}

void setup() {
  initialize_robot();
}

void loop() {

  // Read serial lines from both cameras
  frontCam.read();
  mirrorCam.read();
  
  kicker.update(frontCam.ball_seen, frontCam.ball_distance);

  // Get current yaw from BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();

  if (fabs(current_yaw) < Yaw_zero_glitch_threshold && fabs(last_valid_yaw) > Yaw_last_valid_min_change) {
    current_yaw = last_valid_yaw;
  } else {
    last_valid_yaw = current_yaw;
  }

  // Obtain error for PID
  double error = bno.GetError();

  if (debug_bno) {
    Serial.println("=== BNO Data ===");
    Serial.print("Current Yaw: ");
    Serial.print(current_yaw);
    Serial.print(" | Error: ");
    Serial.println(error);
  }

  // Calculate PID output for angular correction
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, PID_output_min, PID_output_max);

  // Check line sensors — maximum priority
  checkLineSensors();

  if (isAvoidingLine) {
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

    if (isBallFront()) {
      desired_ang_goal(frontCam.goal_angle, frontCam.ball_angle);
      motorss.MoveOmnidirectionalBase((int)temp_ang, Speed, speed_w);
      if (debug_ball_infront) {
        Serial.println("==== Data infront ====");
        Serial.println("Ball detected infront of robot");
        Serial.print("Calculated temp_ang: ");
        Serial.println(temp_ang);
        Serial.print("Goal angle: ");
        Serial.println(frontCam.goal_angle);
        Serial.print("Ball angle: ");
        Serial.println(frontCam.ball_angle);
      }


    } else if (frontCam.ball_seen) {
      ; // When ball is seen but not in front, orient towards the ball
      float ang = -frontCam.ball_angle;
      if (fabsf(ang) < Ball_front_angle_deadband) ang = 0.0f;
      ang = constrain(ang, -Ball_front_angle_clamp, Ball_front_angle_clamp);
      float x = constrain(ang / Ball_front_angle_clamp, -1.0, 1.0);
      float curved = powf(fabs(x), 3.0f) * (x >= 0 ? 1 : -1);
      temp_ang = curved * Ball_front_angle_clamp;
      motorss.MoveOmnidirectionalBase((int)ang, Speed, speed_w);
      
      
      if (debug_frontal_camera){
      Serial.println("=== Frontal Camera Data ====");
      Serial.print("Ball angle : ");
      Serial.println(ang);
      Serial.print("Goal angle: ");
      Serial.println(frontCam.goal_angle);
      Serial.print("Ball distance: ");
      Serial.println(frontCam.ball_distance);
      }

    } else if (mirrorCam.ball_seen) {
      bno.SetTarget(0.0f);

      Robot_Mode_Mirror currentMode;


      if ((mirrorCam.ball_angle > Mirror_ball_right_ang_min) && (mirrorCam.ball_angle < Mirror_ball_right_ang_max)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle + Mirror_ball_flank_offset), Speed_lateral_movement, speed_w);
        currentMode = Ball_right;

      } else if ((mirrorCam.ball_angle < Mirror_ball_left_ang_min) && (mirrorCam.ball_angle > Mirror_ball_left_ang_max)) {
        motorss.MoveOmnidirectionalBase((int)(mirrorCam.ball_angle - Mirror_ball_flank_offset), Speed_lateral_movement, speed_w);
        currentMode = Ball_left;
      } else {
        motorss.MoveOmnidirectionalBase(Mirror_ball_behind_ang, Speed_lateral_movement, speed_w);
        currentMode = Ball_behind;
      }

      if (debug_mirror_camera) {
        Serial.println("=== Mirrror_cam_section === ");
        Serial.print("Ball seen in mirror: ");
        Serial.println(mirrorCam.ball_angle);
        switch(currentMode){
          case Ball_right: Serial.println("Ball detected in right"); break;
          case Ball_left: Serial.println("Ball detected left"); break;
          case Ball_behind: Serial.println("Ball detected behind"); break;
        }

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
      if (debug_movement){
        Serial.println("=== Robot Looking for Ball ===");
        if (sweepRight) {
          Serial.println("Moving right");
        }
        else{
          Serial.println("Moving left");
        }
      }
    }
  }



  delay(20);
}