#include <Arduino.h>
#include "LenBucUart.h"

void InitPins(void);
void PrintDigitalPins(void);
void PrintAnalogPins(void);
void PrintMenu(void);
void ClearScreen(void);
void ErasePreviousSixLines(void);

bool menuActive;

char previousCommand = '\n';

void setup() {
  Serial.begin(9600);
  LBBegin(9600);
  InitPins();
}
void loop() {
    if (LBAvailable()) {
      LBWrite(LBRead());
    }
    delay(1);
    return;
  if (Serial.available() > 0) {
    switch ((char)Serial.read())
    {
    case 'T':
      break;
    case 'S':
      ClearScreen();
      PrintMenu();
      menuActive = true;
      previousCommand = 'S';
      break;
    case 'D':
      if (menuActive) {
        if (previousCommand == 'D' || previousCommand == 'A') {
          ErasePreviousSixLines();
        }
        PrintDigitalPins();
        previousCommand = 'D';
      }
      break;
    case 'A':
      if (menuActive) {
        if (previousCommand == 'D' || previousCommand == 'A') {
          ErasePreviousSixLines();
        }
        PrintAnalogPins();
        previousCommand = 'A';
      }
      break;
    case 'C':
      if (menuActive) {
        // Clear the screen
        menuActive = false;
        ClearScreen();
        previousCommand = 'C';
      }
    default:
      // Invalid character received. Do nothing.
      break;
    }    
  }
}
void InitPins(void) {
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(PIN_A0, INPUT);
  pinMode(PIN_A1, INPUT);
  pinMode(PIN_A2, INPUT);
  pinMode(PIN_A3, INPUT);
  pinMode(PIN_A4, INPUT);
  pinMode(PIN_A5, INPUT);
}
void PrintDigitalPins(void) {
  Serial.print("DI08: ");
  Serial.println(digitalRead(8));
  Serial.print("DI09: ");
  Serial.println(digitalRead(9));
  Serial.print("DI10: ");
  Serial.println(digitalRead(10));
  Serial.print("DI11: ");
  Serial.println(digitalRead(11));
  Serial.print("DI12: ");
  Serial.println(digitalRead(12));
  Serial.print("DI13: ");
  Serial.println(digitalRead(13));
}
void PrintAnalogPins(void) {
  Serial.print("A0: ");
  Serial.println(analogRead(PIN_A0));
  Serial.print("A1: ");
  Serial.println(analogRead(PIN_A1));
  Serial.print("A2: ");
  Serial.println(analogRead(PIN_A2));
  Serial.print("A3: ");
  Serial.println(analogRead(PIN_A3));
  Serial.print("A4: ");
  Serial.println(analogRead(PIN_A4));
  Serial.print("A5: ");
  Serial.println(analogRead(PIN_A5));
}
void PrintMenu(void) {
  Serial.println("View current Input Levels:");
  Serial.println("- D: Digital Inputs DI08 - DI13");
  Serial.println("- A: Analog Inputs A0 - A5");
  Serial.println("- C: Clear Screen");
}
void ClearScreen(void) {
  Serial.write(27); // ESC command
  Serial.print("[2J"); // CLS command
  Serial.write(27); // ESC command
  Serial.print("[H"); // Reset cursor
}
void ErasePreviousSixLines() {
  for (int x = 1; x <= 6; x++) {
    Serial.write(27);
    Serial.print("[6A"); // Move cursor up one line
    Serial.write(27);
    Serial.print("[K"); // Erase to end of line
  }
}