#include <Arduino.h>
#include "RobotInstances.h"

PhotoMux::Sensor front[8] = {
  {3,0},{3,1},{3,2},{3,3},{3,4},{3,5},{3,6},{3,7}
};
PhotoMux::Sensor back[8] = {
  {2,0},{2,1},{2,2},{2,3},{2,4},{2,5},{2,6},{2,7}
};
PhotoMux::Sensor left[8] = {
  {1,0},{1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}
};
PhotoMux::Sensor right[8] = {
  {0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7}
};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  sensors.begin();
  sensors.configureSide(FRONT, front, 8);
  sensors.configureSide(BACK,  back,  8);
  sensors.configureSide(LEFT,  left,  8);
  sensors.configureSide(RIGHT, right, 8);

  sensors.setThreshold(FRONT, FRONT_THRESHOLD);
  sensors.setThreshold(BACK,  BACK_THRESHOLD);
  sensors.setThreshold(LEFT,  LEFT_THRESHOLD);
  sensors.setThreshold(RIGHT, RIGHT_THRESHOLD);
}

void loop() {
  Serial.println("========================================");
  Serial.print("FRONT (MUX 0) avg: "); Serial.println(sensors.getAverage(FRONT));
  Serial.print("LEFT  (MUX 1) avg: "); Serial.println(sensors.getAverage(LEFT));
  Serial.print("BACK  (MUX 2) avg: "); Serial.println(sensors.getAverage(BACK));
  Serial.print("RIGHT (MUX 3) avg: "); Serial.println(sensors.getAverage(RIGHT));

  delay(500);
}