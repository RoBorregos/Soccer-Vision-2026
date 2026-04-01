#pragma once
#include <Arduino.h>

class Kicker {
    public:
    Kicker(uint8_t pin, float distanceThreshold, unsigned long pulsems, unsigned long cooldownms);
    void update(bool ballSeen, float ballDistance);

    private:
    uint8_t _pin;
    float _distanceThreshold;
    unsigned long _pulse_ms;
    unsigned long _cooldown_ms;
    bool _active;
    unsigned long _pulseStart;
    unsigned long _lastKickTime;

    bool canKick(bool ballSeen, float ballDistance);


};