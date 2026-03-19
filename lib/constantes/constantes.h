#ifndef constantes_h
#define constantes_h
#include <stdint.h>

//PID constants
extern float p;
extern float i;
extern float d;
extern float pid_max_output;

const double DeadEnd = 2.0; // Deadband for yaw error
const double I_DEADBAND = 1.5; // Degrees
const int MOTOR3_IN1 = 11; //34
const int MOTOR3_IN2 = 10; //33
const int MOTOR3_PWM = 12;  //4

const int MOTOR2_IN1 = 30;  // 30  //Debería ser el 4
const int MOTOR2_IN2 = 31;  // 31
const int MOTOR2_PWM = 3;  // 3

const int MOTOR4_IN1 = 35;  // 36   //Deberia ser el 2 //Motor 4 para electronica
const int MOTOR4_IN2 = 36;  // 35
const int MOTOR4_PWM = 5;  // 5

const int MOTOR1_IN1 = 29;  // 29   //Esta Bien
const int MOTOR1_IN2 = 28;  // 28
const int MOTOR1_PWM = 2;  // 2

const int KICKER_PIN =  32; //Ping del Kicker

// PhotoMux pins
const uint8_t FRONT_PINS[] = {39, 40, 41, 20};
const uint8_t LEFT_PINS[]  = {27, 26, 38};
const uint8_t RIGHT_PINS[] = {22, 23, 17};

const uint8_t FRONT_COUNT  = 4;
const uint8_t LEFT_COUNT   = 3;
const uint8_t RIGHT_COUNT  = 3;

//Treshdolds for photos
const int FRONT_THRESHOLD = 8500;
const int LEFT_THRESHOLD  = 6000;
const int RIGHT_THRESHOLD = 6000;


//Constantes para velocidades del dribbler
const int servo_min = 1000;
const int servo_mid = 1300;
const int servo_max = 1600;

const unsigned long correctionTime = 250;

#endif
