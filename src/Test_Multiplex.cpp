#include <Arduino.h>
#include "RobotInstances.h"


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