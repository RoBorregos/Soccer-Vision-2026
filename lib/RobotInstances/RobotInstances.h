#pragma once
#include "BNO.h"
#include "PID.h"
#include "PhotoMux.h"
#include "camera.h"
#include "motors.h"
#include "constantes.h"
#include "Kicker.h"

#include "constantes.h"

extern BNO055 bno;
extern PID pid;
extern Motors motorss;
extern PhotoMux sensors;

extern PhotoMux::Sensor front[];
extern const uint8_t FRONT_COUNT;
extern Kicker kicker;

extern camera frontCam;
extern camera mirrorCam;
void initialize_robot();