#ifndef Bno_h
#define Bno_h
#include "Arduino.h"
#include "Wire.h"
#include <Adafruit_BNO08x.h>

class BNO085
{
private:
    double yaw_;
    double raw_yaw_;
    double target_angle_;
    double difference_angle_;
    double setpoint;
    Adafruit_BNO08x bno_{-1};
    sh2_SensorValue_t sensor_value_;

public:
    BNO085();
    void InitializeBNO();
    double NormalizeAngle(double angle);
    void GetBNOData();
    double GetYaw();

    // Funciones de setpoint y error
    void SetTarget(double target);
    double GetError();
};

#endif
