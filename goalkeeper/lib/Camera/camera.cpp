#include "camera.h"

camera::camera(HardwareSerial& serial, bool isMirror, bool enemy_yellow)
: _serial(serial), _isMirror(isMirror), _enemy_yellow(enemy_yellow),
    ball_distance(0), ball_angle(0),
    goal_distance(0), goal_angle(0),
    own_distance(0),  own_angle(0),
    ball_seen(false), goal_seen(false), own_seen(false),
    _buffer{0}, _bufferIndex(0)
{
}

void camera::read() {
  while (_serial.available()) {
    char c = (char)_serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      _buffer[_bufferIndex] = '\0';
      process(_buffer);
      _bufferIndex = 0;
    } else {
      if (_bufferIndex < (BUFFER_SIZE - 1)) {
        _buffer[_bufferIndex++] = c;
      } else {
        _bufferIndex = 0;
      }
    }
  }
}

void camera::process(const char* line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line, "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    ball_distance = dist;   ball_angle = ang;
    if (_enemy_yellow){
    goal_distance = g_dist; goal_angle = g_ang;
    own_distance  = o_dist; own_angle  = o_ang;
    }
  else {
    goal_distance = o_dist; goal_angle = o_ang;
    own_distance  = g_dist; own_angle  = g_ang;

  }


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