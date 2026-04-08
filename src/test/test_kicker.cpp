#include <Arduino.h>

const uint8_t Output_pin = 32;

void setup() {
  pinMode(Output_pin, OUTPUT);
  digitalWrite(Output_pin, HIGH);
}

void loop() {
}