#include <Arduino.h>

const uint8_t Output_pin = 32;

void setup() {
  pinMode(Output_pin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(Output_pin,  LOW);
  delay(5000);
  digitalWrite(Output_pin,  HIGH);
  Serial.println("KIcker HIGH");
  delay(100);

}