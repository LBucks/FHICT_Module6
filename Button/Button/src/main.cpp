#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(2);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN5, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(PIN5)) {
    // Button released
    digitalWrite(LED_BUILTIN, false);
  }
  else {
    // Button pressed
    digitalWrite(LED_BUILTIN, true);
  }
}