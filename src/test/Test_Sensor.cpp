#include <Arduino.h>
#include "RobotInstances.h"


void printSideReadings(const char* sideName, PhotoMux::Sensor* sensorArray, uint8_t count, Side side) {
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
  
  Serial.print("  AVG: ");
  Serial.println(phototransistors.getAverage(side));
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  phototransistors.begin();
  phototransistors.configureSide(FRONT, front, 8);
  phototransistors.configureSide(BACK,  back,  7);
  phototransistors.configureSide(LEFT,  left,  7);
  phototransistors.configureSide(RIGHT, right, 6);

  phototransistors.setThresholdRange(FRONT, FRONT_THRESHOLD_MIN, FRONT_THRESHOLD_MAX);
  phototransistors.setThresholdRange(BACK,  BACK_THRESHOLD_MIN, BACK_THRESHOLD_MAX);
  phototransistors.setThresholdRange(LEFT,  LEFT_THRESHOLD_MIN, LEFT_THRESHOLD_MAX);
  phototransistors.setThresholdRange(RIGHT, RIGHT_THRESHOLD_MIN, RIGHT_THRESHOLD_MAX);

  Serial.println("PhotoMux individual sensor test ready.");
}

void loop() {
  printSideReadings("FRONT", front, 8, FRONT);
  printSideReadings("LEFT",  left,  7, LEFT);
  printSideReadings("BACK",  back,  7, BACK);
  printSideReadings("RIGHT", right, 6, RIGHT);

  delay(500);
}