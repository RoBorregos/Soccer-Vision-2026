#include "camera.h"

camera::camera(HardwareSerial& serial, bool isMirror)
: _serial(serial), _isMirror(isMirror),
    ball_distance(0), ball_angle(0),
    goal_distance(0), goal_angle(0),
    own_distance(0),  own_angle(0),
    ball_seen(false), goal_seen(false), own_seen(false),
    _buffer("")
{
}

void camera::read() {
  while (_serial.available()) {
    char c = (char)_serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      process(_buffer);
      _buffer = "";
    } else {
      _buffer += c;
      if (_buffer.length() > 120) _buffer = "";
    }
  }
}

void camera::process(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    ball_distance = dist;   ball_angle = ang;
    goal_distance = g_dist; goal_angle = g_ang;
    own_distance  = o_dist; own_angle  = o_ang;

    if (_isMirror) {
      goal_angle += 25.0f;
      ball_seen = (fabsf(ang)    > 1e-3f);
      goal_seen = (fabsf(g_dist) > 1e-3f);
      own_seen  = (fabsf(o_dist) > 1e-3f);
    } else {
      goal_angle = -goal_angle;
      ball_seen = (fabsf(dist)   > 1e-3f);
      goal_seen = (fabsf(g_dist) > 1e-3f);
      own_seen  = (fabsf(o_dist) > 1e-3f);
    }
  }
}