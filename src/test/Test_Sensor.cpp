#include <Arduino.h>
#include "RobotInstances.h"


void printSideReadings(const char* sideName, PhotoMux::Sensor* sensorArray, uint8_t count) {
  Serial.println("========================================");
  Serial.print("MUX ");
  Serial.print(sensorArray[0].muxIndex);
  Serial.print(" | ");
  Serial.println(sideName);
  Serial.println("----------------------------------------");

  for (uint8_t i = 0; i < count; i++) {
    int val = phototransistors.readSensor(sensorArray[i].muxIndex, sensorArray[i].channel);
    Serial.print("  CH");
    Serial.print(sensorArray[i].channel);
    Serial.print(": ");
    Serial.println(val);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  phototransistors.begin();
  phototransistors.configureSide(FRONT, front, 8);
  phototransistors.configureSide(BACK,  back,  8);
  phototransistors.configureSide(LEFT,  left,  8);
  phototransistors.configureSide(RIGHT, right, 8);

  phototransistors.setThreshold(FRONT, FRONT_THRESHOLD);
  phototransistors.setThreshold(BACK,  BACK_THRESHOLD);
  phototransistors.setThreshold(LEFT,  LEFT_THRESHOLD);
  phototransistors.setThreshold(RIGHT, RIGHT_THRESHOLD);

  Serial.println("PhotoMux individual sensor test ready.");
}

void loop() {
  printSideReadings("FRONT", front, 8);
  printSideReadings("LEFT",  left,  8);
  printSideReadings("BACK",  back,  8);
  printSideReadings("RIGHT", right, 8);

  delay(500);
}