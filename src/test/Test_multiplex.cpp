#include <Arduino.h>
#include "RobotInstances.h"


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  phototransistors.begin();
  phototransistors.configureSide(FRONT, front, 8);
  phototransistors.configureSide(BACK,  back,  8);
  phototransistors.configureSide(LEFT,  left,  8);
  phototransistors.configureSide(RIGHT, right, 6);

  phototransistors.setThresholdRange(FRONT, FRONT_THRESHOLD_MIN, FRONT_THRESHOLD_MAX);
  phototransistors.setThresholdRange(BACK,  BACK_THRESHOLD_MIN, BACK_THRESHOLD_MAX);//400000
  phototransistors.setThresholdRange(LEFT,  LEFT_THRESHOLD_MIN, LEFT_THRESHOLD_MAX);//20000
  phototransistors.setThresholdRange(RIGHT, RIGHT_THRESHOLD_MIN, RIGHT_THRESHOLD_MAX);}

void loop() {
  Serial.println("========================================");
  Serial.print("FRONT (MUX 0) avg: "); Serial.println(phototransistors.getAverage(FRONT));
  Serial.print("LEFT  (MUX 1) avg: "); Serial.println(phototransistors.getAverage(LEFT));
  Serial.print("BACK  (MUX 2) avg: "); Serial.println(phototransistors.getAverage(BACK));
  Serial.print("RIGHT (MUX 3) avg: "); Serial.println(phototransistors.getAverage(RIGHT));

  delay(500);
}