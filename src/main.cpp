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

// ── Phototransistor pin definitions 
const uint8_t FRONT_PINS[] = {39, 40, 41, 20};
const uint8_t RIGHT_PINS[] = {22, 23, 17};
const uint8_t FRONT_COUNT  = 4;
const uint8_t LEFT_COUNT   = 3;
const uint8_t RIGHT_COUNT  = 3;

//Variables for avoiding the lines
const int FRONT_THRESHOLD = 2900; 
const int LEFT_THRESHOLD  = 2100;
const int RIGHT_THRESHOLD = 6000;

enum LineSide { LINE_NONE, LINE_FRONT, LINE_LEFT, LINE_RIGHT, LINE_BOTH_SIDES };

unsigned long lineDetectedTime = 0;

bool isAvoidingLine = false;
LineSide detectedLineSide = LINE_NONE;




//Variable for angle control in different game situations 
float temp_ang = 0;

// Camera boolean states
bool front_ball_seen = false;
bool front_goal_seen = false;
bool mirror_ball_seen = false;
bool mirror_goal_seen = false;


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

//This function is DEADCODE (for now) 
//It was intended to move the robot using the mirror
void mirror_angle_section(double ang, uint8_t speed, double speed_w) { //Deadcpe for now, but might be useful for future improvements
  if (abs(ang) > 145.0f) {
    if (ang > 0) {
      motorss.MoveOmnidirectionalBase(180, speed, speed_w);
      Serial.println("Moving right");
    } else {
      motorss.MoveOmnidirectionalBase(-180, speed, speed_w);
      Serial.println("Moving left");
    }
  } else {
    ang = ang * 1.2;
    motorss.MoveOmnidirectionalBase(0, speed, speed_w);
    Serial.println("Moving forward");
  }
}

//This function checks if the ball is close and infront of the robot, to start different strategies
//It uses the aproximate distance and the angle to determine this
bool isBallFront() {
  if (front_ball_seen) {
    if (frontCam.ball_distance < 42 && abs(frontCam.ball_angle) < 35) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

//This function is used, to check if the ball and the goal are aligned, if they are not it tries to align them
//After the alignement, it uses temp_ang to move the robot towards the ball while multyplying to make a slight curve
void desired_ang_goal(float g_ang, float b_ang, uint8_t speed, double speed_w, double b_distance) {
  //If ang goal is positive (right)
  if (g_ang > 0) {
    if (b_ang < 0) { //Check if goal angle and ball angle are aligned
      temp_ang = b_ang - (b_distance * 0.2); //If not, adjust temp_ang based on ball distance
    } else {
      temp_ang = b_ang +  (b_distance * 0.1); //If aligned, adjust temp_ang based on ball distance
      bno.SetTarget(15);
    }
  }
  //If ang goal is negative (left)
  if (g_ang < 0) {
    if (b_ang > 0) {
      temp_ang = b_ang + (b_distance * 0.2);
    } else {
      temp_ang = b_ang - (b_distance * 0.1);
      bno.SetTarget(-15);
    }
  }
}

//This function checks the line sensors and updates the robot's state accordingly. If a line is detected, it sets the appropriate flags and determines which side the line is on. It also handles the logic for exiting line avoidance mode after a certain amount of time has passed.  

void checkLineSensors() {
  bool frontDetected = isLineDetectedDirect(FRONT_PINS, FRONT_COUNT, FRONT_THRESHOLD); // CHANGED: replaced photoMux call
  bool leftDetected  = isLineDetectedDirect(LEFT_PINS,  LEFT_COUNT,  LEFT_THRESHOLD);
  bool rightDetected = isLineDetectedDirect(RIGHT_PINS, RIGHT_COUNT, RIGHT_THRESHOLD);

  if (frontDetected || leftDetected || rightDetected) {
    lineDetectedTime = millis();
    isAvoidingLine   = true;

    if (leftDetected && rightDetected) {
      detectedLineSide = LINE_BOTH_SIDES;
      Serial.println("LINE DETECTED: BOTH SIDES -> Moving backward");
    } else if (frontDetected) {
      detectedLineSide = LINE_FRONT;
      Serial.println("LINE DETECTED: FRONT -> Moving backward");
      Serial.println(photoMux.getAverage(FRONT));
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

  // Initialize PhotoMux
  photoMux.begin();
  photoMux.configureSide(FRONT, front, FRONT_COUNT);
  photoMux.setThreshold(FRONT, FRONT_THRESHOLD);
}

void loop() {
  // Read serial lines from both cameras
  frontCam.read();
  mirrorCam.read();

  // Update camera boolean states
  front_ball_seen  = frontCam.ball_seen;
  front_goal_seen  = frontCam.goal_seen;
  mirror_ball_seen = mirrorCam.ball_seen;
  mirror_goal_seen = mirrorCam.goal_seen;

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
      case LINE_BOTH_SIDES:
        motorss.MoveBackward();
        break;
      case LINE_LEFT:
        motorss.MoveOmnidirectionalBase(90, 120, speed_w);
        break;
      case LINE_RIGHT:
        motorss.MoveOmnidirectionalBase(-90, 120, speed_w);
        break;
      default:
        motorss.MoveBackward();
        break;
    }

  } else {

    motorss.SetAllSpeeds(Speed);

    if (isBallFront()) {
      // Ball is close and centered: aim toward goal
      desired_ang_goal(frontCam.goal_angle, frontCam.ball_angle, Speed, speed_w);
      motorss.MoveOmnidirectionalBase(temp_ang, Speed, speed_w);
      Serial.print("Ball in front, moving with temp_ang: ");
      Serial.println(temp_ang);

    } else {


      Serial.println(frontCam.ball_seen);

      if (front_ball_seen) {
        // Ball visible from front camera
        Serial.print("Ball seen front: ");
        float ang = -frontCam.ball_angle;
        if (fabsf(ang) < 7.0f) ang = 0.0f;
        ang = constrain(ang, -90.0f, 90.0f);
        Serial.print(ang);
        Serial.println(frontCam.ball_distance);
        motorss.MoveOmnidirectionalBase((int)ang, Speed, speed_w);

      } else if (mirror_ball_seen) {
        // Ball visible from mirror camera
        Serial.print("Ball seen in mirror: ");
        Serial.println(mirrorCam.ball_angle);

        if (mirrorCam.ball_angle > 45.0f) {
          double m_ang = mirrorCam.ball_angle + 30.0;
          motorss.MoveOmnidirectionalBase((int)m_ang, 90, speed_w);
          Serial.println("Moving right");
        } else if (mirrorCam.ball_angle < -40.0f) {
          double m_ang = mirrorCam.ball_angle - 30.0;
          motorss.MoveOmnidirectionalBase((int)m_ang, 90, speed_w);
          Serial.println("Moving left");
        } else {
          Serial.println("Ball is behind");
          motorss.MoveOmnidirectionalBase(120, Speed, speed_w);
        }

      } else {
        // Ball not seen anywhere sweep side to side
        if (millis() - last_time >= 1000) {
          last_time += 1000;
          side = !side;
        }

        if (side) {
          motorss.MoveOmnidirectionalBase(-90, Speed, speed_w);
        } else {
          motorss.MoveOmnidirectionalBase(90, Speed, speed_w);
        }
      }
    }
  }

  delay(20);
}