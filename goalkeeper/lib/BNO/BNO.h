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
    double roll_;
    double pitch_;
    double target_angle_;
    double difference_angle_;
    double setpoint;
    double yaw_offset_;
    Adafruit_BNO08x bno_{-1};
    sh2_SensorValue_t sensor_value_;
    bool SetReports();

public:
    BNO085();
    void InitializeBNO();
    double NormalizeAngle(double angle);
    void GetBNOData();
    double GetYaw();
    double GetRoll();
    double GetPitch();
    void SetYaw(double yaw);

    // Funciones de setpoint y error
    void SetTarget(double target);
    double GetError();

    // NUEVAS FUNCIONES PARA TELEMETRÍA
    double GetTarget();
    double GetRawYaw();
    void PlotAngles();  // Graficar ángulos del BNO
};

#endif
