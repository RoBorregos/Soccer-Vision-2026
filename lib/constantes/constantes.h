#ifndef constantes_h
#define constantes_h
#include <stdint.h>

//PID constants

//PID constants definitions
const float p = 1.6;
const float i = 0.015;
const float d = 0.15;
const float pid_max_output = 120.0;

const double DeadEnd = 2.0; // Deadband for yaw error
const double I_DEADBAND = 1.5; // Degrees
const int BACK_RIGHT_IN1 = 11; //34
const int BACK_RIGHT_IN2 = 10; //33
const int BACK_RIGHT_PWM = 12;  //4

const int FRONT_RIGHT_IN1 = 30;  // 30  //Debería ser el 4
const int FRONT_RIGHT_IN2 = 31;  // 31
const int FRONT_RIGHT_PWM = 3;  // 3

const int BACK_LEFT_IN1 = 35;  // 36   //Deberia ser el 2 //Motor 4 para electronica
const int BACK_LEFT_IN2 = 36;  // 35
const int BACK_LEFT_PWM = 5;  // 5

const int FRONT_LEFT_IN1 = 29;  // 29   //Esta Bien
const int FRONT_LEFT_IN2 = 28;  // 28
const int FRONT_LEFT_PWM = 2;  // 2

const int KICKER_PIN =  32; //Ping del Kicker

// PhotoMux pins
const uint8_t FRONT_PINS[] = {39, 40, 41, 20};
const uint8_t LEFT_PINS[]  = {27, 26, 38};
const uint8_t RIGHT_PINS[] = {22, 23, 17};

const uint8_t FRONT_COUNT  = 4;
const uint8_t LEFT_COUNT   = 3;
const uint8_t RIGHT_COUNT  = 3;

//Treshdolds for photos
const int FRONT_THRESHOLD = 7000;
const int LEFT_THRESHOLD  = 60000;
const int RIGHT_THRESHOLD = 600000;


//Constantes para velocidades del dribbler
const int servo_min = 1000;
const int servo_mid = 1300;
const int servo_max = 1600;

const unsigned long correctionTime = 250;

#endif
