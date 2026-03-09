#include <Arduino.h>
#include "RoboInstances.h"

unsigned long current_time;
unsigned long last_time = 0;
bool side = true;

float setpoint = 0.0f; //Target angle for PID
double last_valid_yaw = 0.0;
double current_yaw = 0.0; //Current angle
double last_speed_w = 0.0; //No use for now
const uint8_t Speed = 120; //Robot speedbase



unsigned long lineDetectedTime = 0;

bool isAvoidingLine = false;

//For Ball control
float temp_ang = 0;

// Camera boolean states
bool front_ball_seen = false;
bool front_goal_seen = false;
bool mirror_ball_seen = false;
bool mirror_goal_seen = false;

void mirror_angle_section(double ang, uint8_t speed, double speed_w) {
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

bool isBallFront() {
  if (front_ball_seen) {
    if (frontCam.ball_distance < 42 && abs(frontCam.ball_angle) < 30) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void desired_ang_goal(float g_ang, float b_ang, uint8_t speed, double speed_w) {
  //If ang goal is positive (right)
  if (g_ang > 0) {
    if (b_ang < 0) { //Check if goal angle and ball angle are aligned
      temp_ang = b_ang - 40;
      motorss.MoveOmnidirectionalBase(temp_ang, speed, speed_w);
    } else {
      temp_ang = b_ang * 1.15;
      bno.SetTarget(15);
    }
  }
  //If ang goal is negative (left)
  if (g_ang < 0) {
    if (b_ang > 0) {
      temp_ang = b_ang + 30;
      motorss.MoveOmnidirectionalBase(temp_ang, speed, speed_w);
    } else {
      temp_ang = b_ang * 1.15;
      bno.SetTarget(-15);
    }
  }
}

void checkLineSensors() {
  bool frontDetected = photoMux.isLineDetected(FRONT);

  if (frontDetected) {
    lineDetectedTime = millis();
    isAvoidingLine = true;
    Serial.println("LINE DETECTED");
  } else if (isAvoidingLine) {
    if (millis() - lineDetectedTime >= correctionTime) {
      isAvoidingLine = false;
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
    motorss.SetAllSpeeds(Speed);
    motorss.MoveBackward();

  } else {
    // ── Normal movement logic ─────────────────────────────────────────────
    motorss.SetAllSpeeds(Speed);

    if (isBallFront()) {
      // Ball is close and centered: aim toward goal
      desired_ang_goal(frontCam.goal_angle, frontCam.ball_angle, Speed, speed_w);
      motorss.MoveOmnidirectionalBase(temp_ang, Speed, speed_w);
      Serial.print("Ball in front, moving with temp_ang: ");
      Serial.println(temp_ang);

    } else {
      // Ball NOT in front range — search / approach

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
          Serial.println("Ball out of bounds");
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