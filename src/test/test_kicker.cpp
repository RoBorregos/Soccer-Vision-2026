#include <Arduino.h>

const uint8_t Output_pin = 32;

void setup() {
  pinMode(Output_pin, OUTPUT);
  digitalWrite(Output_pin, LOW);
}

void loop() {
  digitalWrite(Output_pin, HIGH);
  delay(100);
  digitalWrite(Output_pin, LOW);
  delay(5000);
}