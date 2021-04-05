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
  LBBegin(9600);
  InitPins();
}
void loop() {
  if (LBAvailable()) {
    switch ((char)LBRead())
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
  LBPrint("DI08: ");
  LBPrintln((boolean)digitalRead(8));
  LBPrint("DI09: ");
  LBPrintln((boolean)digitalRead(9));
  LBPrint("DI10: ");
  LBPrintln((boolean)digitalRead(10));
  LBPrint("DI11: ");
  LBPrintln((boolean)digitalRead(11));
  LBPrint("DI12: ");
  LBPrintln((boolean)digitalRead(12));
  LBPrint("DI13: ");
  LBPrintln((boolean)digitalRead(13));
}
void PrintAnalogPins(void) {
  LBPrint("A0: ");
  LBPrintln(analogRead(PIN_A0));
  LBPrint("A1: ");
  LBPrintln(analogRead(PIN_A1));
  LBPrint("A2: ");
  LBPrintln(analogRead(PIN_A2));
  LBPrint("A3: ");
  LBPrintln(analogRead(PIN_A3));
  LBPrint("A4: ");
  LBPrintln(analogRead(PIN_A4));
  LBPrint("A5: ");
  LBPrintln(analogRead(PIN_A5));
}
void PrintMenu(void) {
  LBPrintln("View current Input Levels:");
  LBPrintln("- D: Digital Inputs DI08 - DI13");
  LBPrintln("- A: Analog Inputs A0 - A5");
  LBPrintln("- C: Clear Screen");
}
void ClearScreen(void) {
  return; // Currently broken with my own version, but out of time to debug.
  Serial.write(27); // ESC command
  LBPrint("[2J"); // CLS command
  Serial.write(27); // ESC command
  LBPrint("[H"); // Reset cursor
}
void ErasePreviousSixLines() {
  return; // Currently broken with my own version, but out of time to debug.
  for (int x = 1; x <= 6; x++) {
    Serial.write(27);
    LBPrint("[6A"); // Move cursor up one line
    Serial.write(27);
    LBPrint("[K"); // Erase to end of line
  }
}