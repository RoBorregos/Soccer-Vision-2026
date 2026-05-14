#include <Arduino.h>

const int MODULE_PIN = 2;

void setup() {
  Serial.begin(9600);
  pinMode(MODULE_PIN, INPUT);
}

void loop() {
  int state = digitalRead(MODULE_PIN);
  Serial.println(state);
  delay(100);
}
