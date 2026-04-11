#include <Arduino.h>
#include "RobotInstances.h"


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  phototransistors.begin();
  phototransistors.configureSide(FRONT, front, 8);
  phototransistors.configureSide(BACK,  back,  8);
  phototransistors.configureSide(LEFT,  left,  8);
  phototransistors.configureSide(RIGHT, right, 5);

  phototransistors.setThreshold(FRONT, FRONT_THRESHOLD);
  phototransistors.setThreshold(BACK,  BACK_THRESHOLD);
  phototransistors.setThreshold(LEFT,  LEFT_THRESHOLD);
  phototransistors.setThreshold(RIGHT, RIGHT_THRESHOLD);
}

void loop() {
  Serial.println("========================================");
  Serial.print("FRONT (MUX 0) avg: "); Serial.println(phototransistors.getAverage(FRONT));
  Serial.print("LEFT  (MUX 1) avg: "); Serial.println(phototransistors.getAverage(LEFT));
  Serial.print("BACK  (MUX 2) avg: "); Serial.println(phototransistors.getAverage(BACK));
  Serial.print("RIGHT (MUX 3) avg: "); Serial.println(phototransistors.getAverage(RIGHT));

  delay(500);
}