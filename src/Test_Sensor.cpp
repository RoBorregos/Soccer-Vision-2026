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
    int val = sensors.readSensor(sensorArray[i].muxIndex, sensorArray[i].channel);
    Serial.print("  CH");
    Serial.print(sensorArray[i].channel);
    Serial.print(": ");
    Serial.println(val);
  }
}

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

  Serial.println("PhotoMux individual sensor test ready.");
}

void loop() {
  printSideReadings("FRONT", front, 8);
  printSideReadings("LEFT",  left,  8);
  printSideReadings("BACK",  back,  8);
  printSideReadings("RIGHT", right, 8);

  delay(500);
}