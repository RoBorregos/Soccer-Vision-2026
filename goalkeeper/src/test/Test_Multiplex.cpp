#include <Arduino.h>
#include "RobotInstances.h"

void printSideValues(const char* label, const PhotoMux::Sensor* sideSensors, uint8_t count, Side side) {
  Serial.print(label);
  Serial.print(" avg: ");
  Serial.println(phototransistors.getAverage(side));

  Serial.print("  values: ");
  for (uint8_t i = 0; i < count; i++) {
    int value = phototransistors.readSensor(sideSensors[i].muxIndex, sideSensors[i].channel);
    Serial.print(value);
    if (i < count - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  phototransistors.begin();
  phototransistors.configureSide(FRONT, front, 8);
  phototransistors.configureSide(BACK,  back,  8);
  phototransistors.configureSide(LEFT,  left,  8);
  phototransistors.configureSide(RIGHT, right, 5);

  phototransistors.setThreshold(FRONT, FRONT_THRESHOLD);
  phototransistors.setThreshold(BACK,  BACK_THRESHOLD);//400000
  phototransistors.setThreshold(LEFT,  LEFT_THRESHOLD);//20000
  phototransistors.setThreshold(RIGHT, RIGHT_THRESHOLD);
}

void loop() {
  Serial.println("========================================");
  printSideValues("BACK  (MUX 2)", back, 8, BACK);//43000
  //printSideValues("FRONT (MUX 0)", front, 8, FRONT);//60000
  //printSideValues("LEFT  (MUX 1)", left, 8, LEFT);//20000
  //printSideValues("RIGHT (MUX 3)", right, 5, RIGHT);//100000
  delay(500);

}