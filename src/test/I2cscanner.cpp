#include <Wire.h>
#include <Arduino.h>

void setup() {
    Wire.begin();
    Serial.begin
    
    (115200);
    while (!Serial);
    Serial.println("\n=== I2C Scanner for Teensy 4.1 ===");
    Serial.println("SDA: pin 18, SCL: pin 19");
    Serial.println("Expected: BNO055 at 0x28\n");
}

void loop() {
    Serial.println("Scanning...");
    int nDevices = 0;

    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("  Device found at 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);

            if (address == 0x28 || address == 0x29)
                Serial.print("  <-- BNO055");

            Serial.println();
            nDevices++;
        }
    }

    if (nDevices == 0)
        Serial.println("  No devices found! Check wiring.\n");
    else {
        Serial.print(nDevices);
        Serial.println(" device(s) found.\n");
    }

    delay(5000);
}