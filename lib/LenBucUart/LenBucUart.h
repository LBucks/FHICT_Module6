#pragma once
#include <Arduino.h>

/*
    Public functions
*/
void LBBegin(int baudrate);
void LBWrite(byte data);
boolean LBAvailable(void);
byte LBRead(void);