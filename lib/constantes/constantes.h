#ifndef constantes_h
#define constantes_h
#include <stdint.h>

//|||PID Constants|||

//PID values
const float p = 1.6;
const float i = 0.015;
const float d = 0.15;
const float pid_max_output = 120.0;
const double DeadEnd = 2.0;
const double I_DEADBAND = 1.5;

// PID limits and thresholds
const float PID_output_max = 180.0f;
const float PID_output_min = -180.0f;
const float Yaw_zero_glitch_threshold  = 0.001; // Minimum yaw magnitude to consider a real reading
const float Yaw_last_valid_min_change  = 1.0;   // Minimum last yaw magnitude that triggers glitch detection

//||||| Robot Movement||||||||


//Robot speedbase
const uint8_t Speed = 100; //Robot speedbase

// Motor pins
const int BACK_RIGHT_IN1 = 10;
const int BACK_RIGHT_IN2 = 11;
const int BACK_RIGHT_PWM = 12;

const int BACK_LEFT_IN1  = 31;
const int BACK_LEFT_IN2  = 30;
const int BACK_LEFT_PWM  = 3;

const int FRONT_RIGHT_IN1 = 35;
const int FRONT_RIGHT_IN2 = 36;
const int FRONT_RIGHT_PWM = 5;

const int FRONT_LEFT_IN1  = 28;
const int FRONT_LEFT_IN2  = 29;
const int FRONT_LEFT_PWM  = 2;

// Search sweep lateral angles
const float Search_sweep_ang_right = -90.0f;
const float Search_sweep_ang_left  =  90.0f;

//  |||||UTILITIES |||||


// Kicker
const int KICKER_PIN = 32;

// Servo
const int servo_min = 1000;
const int servo_mid = 1300;
const int servo_max = 1600;

//Vision Tresholds
const float Ball_distance_threshold   = 125.0f; // Distance threshold to consider the ball is in front of the robot
const float Ball_infront_ang_threshold = 25.0f; // Angle threshold to consider the ball is in front of the robot
const float Deadband_4_ballgoalangle  = 20.0f;  // Deadband for ball-goal angle when the ball is in front

// PhotoMux pins
const uint8_t selectPins[3] = {16, 15, 14};       // S0, S1, S2
const uint8_t muxPins[4]    = {A6, A7, A9, A14};  // Un pin analógico por chip mux

// Thresholds para detección de línea
const int FRONT_THRESHOLD = 20000;
const int LEFT_THRESHOLD  = 45000;
const int RIGHT_THRESHOLD = 130000;
const int BACK_THRESHOLD  = 80000;

// Tiempo de corrección tras detectar línea
const unsigned long correctionTime = 250;


//Variables for front
const float Ball_front_angle_deadband = 6.0f;
const float Ball_front_angle_clamp = 90.0f;
const float Kick_ball_distance_very_close = 55.0f;


//Kicker timing
const unsigned long Kicker_pulse_ms = 70;
const unsigned long Kicker_cooldown_ms = 1300;

//Variables for aligning robot towards goal when ball is infront
const float Goal_heading_offset_right =  10.0f; // Added when goal is on the right
const float Goal_heading_offset_left  = -10.0f; // Added when goal is on the left

//Variables for alining ball with goal
const float Ball_orbit_offset = 80.0f;
const float Ball_front_min_lateral_angle = 10.0f;


//What is this
// Mirror camera — angular window where the ball is considered to be on the right or left flank
const float Mirror_ball_right_ang_min =  45.0f;
const float Mirror_ball_right_ang_max = 135.0f;
const float Mirror_ball_left_ang_min  = -45.0f;
const float Mirror_ball_left_ang_max  = -135.0f;

const float Mirror_ball_flank_offset = 30.0f;
const float Mirror_ball_behind_ang = 125.0f;
const unsigned long Search_sweep_interval_ms = 1000;


//||||| LINE AVOIDANCE VARIABLES ||||


enum LineSide { LINE_NONE, LINE_FRONT, LINE_LEFT, LINE_RIGHT, LINE_BACK, LINE_BOTH_SIDES, LINE_FRONT_LEFT, LINE_FRONT_RIGHT, LINE_ALL_SIDES };
const float Line_avoid_ang_front      = 180.0f;  // Ball is behind robot, push backward
const float Line_avoid_ang_front_left =  45.0f;  // Diagonal right-backward
const float Line_avoid_ang_front_right = -45.0f; // Diagonal left-backward
const float Line_avoid_ang_left       =  90.0f;  // Strafe right
const float Line_avoid_ang_right      = -90.0f;  // Strafe left
const float Line_avoid_ang_back       =   0.0f;  // Move forward to escape rear line

// Line avoidance speed override
const uint8_t Line_avoid_speed = 120;



// |||||| BNO VARIABLES |||||
const uint16_t BNO_setup_delay_ms = 300;

#endif