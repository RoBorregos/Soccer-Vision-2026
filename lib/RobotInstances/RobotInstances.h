#pragma once
#include "motors.h"
#include "BNO.h"
#include "PID.h"
#include "PhotoMux.h"
#include "camera.h"

#include "constantes.h"

extern BNO055 bno;
extern PID pid;
extern Motors motorss;
extern PhotoMux photoMux;

extern PhotoMux::Sensor front[];
extern const uint8_t FRONT_COUNT;

extern camera frontCam;
extern camera mirrorCam;
void initialize_robot();
