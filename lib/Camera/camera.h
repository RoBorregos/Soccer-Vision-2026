#pragma once
#include <Arduino.h>

class camera {
    public:
    float ball_distance, ball_angle;
    float goal_distance, goal_angle;
    float own_distance,  own_angle;
    bool  ball_seen = false, goal_seen = false, own_seen = false;

    camera(HardwareSerial& serial, bool isMirror = false, bool enemy_yellow = true);
    void read();
    
    private:
    HardwareSerial& _serial;
    String _buffer;
    bool _isMirror;
    bool _enemy_yellow;

    void process(const String& line);
};