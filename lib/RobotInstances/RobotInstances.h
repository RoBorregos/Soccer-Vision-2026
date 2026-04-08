#pragma once
#include "BNO.h"
#include "PID.h"
#include "PhotoMux.h"
#include "camera.h"
#include "motors.h"
#include "constantes.h"
#include "kicker.h"

#include "constantes.h"

extern BNO055 bno;
extern PID pid;
extern Motors motorss;
extern PhotoMux phototransistors;

extern PhotoMux::Sensor front[];
extern PhotoMux::Sensor back[];
extern PhotoMux::Sensor left[];
extern PhotoMux::Sensor right[];
extern const uint8_t FRONT_COUNT;
extern Kicker kicker;

extern camera frontCam;
extern camera mirrorCam;
void initialize_robot();
