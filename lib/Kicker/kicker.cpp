#include "Kicker.h"

Kicker::Kicker(uint8_t pin, float distanceThreshold, unsigned long pulsems, unsigned long cooldownms)
    : _pin(pin), _distanceThreshold(distanceThreshold), _pulse_ms(pulsems), _cooldown_ms(cooldownms),
      _active(false), _pulseStart(0), _lastKickTime(0) {
    pinMode(_pin, OUTPUT);
}

bool Kicker::canKick(bool ballSeen, float ballDistance) {
    return ballSeen && ballDistance < _distanceThreshold;
}

void Kicker::update(bool ballSeen, float ballDistance) {
    unsigned long now = millis();
    if (_active) {
        if (now - _pulseStart >= _pulse_ms) {
            digitalWrite(_pin, LOW);
            _active = false;
            _lastKickTime = now;
        }
        return;
    }
    if (canKick(ballSeen, ballDistance) && (now - _lastKickTime >= _cooldown_ms)) {
        digitalWrite(_pin, HIGH);
        _active = true;
        _pulseStart = now;
    }
}