#ifndef constantes_h
#define constantes_h
#include <stdint.h>

// PID constants
const float p = 1.6;
const float i = 0.015;
const float d = 0.15;
const float pid_max_output = 120.0;
const double DeadEnd = 2.0;
const double I_DEADBAND = 1.5;

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

// Kicker
const int KICKER_PIN = 32;

// Servo
const int servo_min = 1000;
const int servo_mid = 1300;
const int servo_max = 1600;

// PhotoMux pins
const uint8_t selectPins[3] = {16, 15, 14};       // S0, S1, S2
const uint8_t muxPins[4]    = {A6, A7, A9, A14};  // Un pin analógico por chip mux

// Thresholds para detección de línea
const int FRONT_THRESHOLD = 20000;
const int LEFT_THRESHOLD  = 19000;
const int RIGHT_THRESHOLD = 30000;
const int BACK_THRESHOLD  = 80000;

// Tiempo de corrección tras detectar línea
const unsigned long correctionTime = 250;

#endif