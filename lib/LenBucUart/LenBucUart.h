#pragma once
#include <Arduino.h>

/*
    Public functions
*/
void LBBegin(int baudrate);
void LBWrite(byte data);
void LBPrint(const char *str);
void LBPrintln(boolean);
void LBPrintln(int);
void LBPrintln(const char *str);
boolean LBAvailable(void);
byte LBRead(void);