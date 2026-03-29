#include <Arduino.h>
#include "RobotInstances.h"

// Definición de pines para los 8 sensores por placa
PhotoMux::Sensor front[8] = {{0,0},{0,1},{0,2},{0,3},{0,4},{0,5},{0,6},{0,7}};
PhotoMux::Sensor left[8]  = {{1,0},{1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}};
PhotoMux::Sensor back[8]  = {{2,0},{2,1},{2,2},{2,3},{2,4},{2,5},{2,6},{2,7}};
PhotoMux::Sensor right[8] = {{3,0},{3,1},{3,2},{3,3},{3,4},{3,5},{3,6},{3,7}};

// Mux Control Pins (Ajusta estos a tus pines reales de S0, S1, S2)
const int S0 = 24; 
const int S1 = 25;
const int S2 = 26;
const int muxPins[] = {A17, A16, A15, A14}; // Pines analógicos de las 4 placas

void setup() {
  Serial.begin(115200);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  analogReadResolution(12);

  Serial.println("--- DIAGNÓSTICO DIRECTO DE HARDWARE ---");
}

void loop() {
  Serial.println("\nPLACA: \tCH0\tCH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7");
  Serial.println("-------------------------------------------------------------------");

  const char* labels[] = {"FRONT", "LEFT ", "BACK ", "RIGHT"};

  for (int m = 0; m < 4; m++) {
    Serial.print(labels[m]);
    Serial.print(":\t");
    
    for (int ch = 0; ch < 8; ch++) {
      // Control manual de los pines del multiplexor para saltarnos la librería
      digitalWrite(S0, (ch & 0x01));
      digitalWrite(S1, (ch & 0x02));
      digitalWrite(S2, (ch & 0x04));
      
      delayMicroseconds(20); // Tiempo para que el switch sea estable
      int val = analogRead(muxPins[m]);
      
      Serial.print(val);
      Serial.print("\t");
    }
    Serial.println();
  }

  Serial.println("-------------------------------------------------------------------");
  delay(500);
}