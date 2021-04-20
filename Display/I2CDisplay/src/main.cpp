#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// Defines and constants
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define OUT_TEMP 0x26
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
const int L3G4200D_Address = 105; //I2C address of the L3G4200D

// Prototypes
void InitializeGyro();
void WriteGyroRegister(byte address, byte data);
byte ReadGyroRegister(byte address);

U8G2_SSD1306_64X32_1F_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
uint8_t flipColor = 0;
uint8_t drawColor = 1;


void setup() {
  Serial.begin(9600);
  display.begin();
}

void loop() {
  int shortZ = ((ReadGyroRegister(OUT_Z_H) << 8) | ReadGyroRegister(OUT_Z_L));
  // Convert to dps
  shortZ = shortZ * 0.070; // Use 0.070 when dps is set to 2000
  if (shortZ < 7 && shortZ > -7) {
    // Remove noise when not moving.
    shortZ = 0;
  }
  char rotationZInString[7];
  itoa(shortZ, rotationZInString, 10);
  char tempInString[2];
  itoa((int)ReadGyroRegister(OUT_TEMP), tempInString, 10);
  // Use pages due to limited uno memory.
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x13B_tf);
    // Draw Z rotation.
    display.drawStr(0, 13, "rZ: ");
    display.drawStr(24, 13, rotationZInString);
    // Draw temperature.
    display.drawStr(0, 26, "Temp: ");
    display.drawStr(36, 26, tempInString);
    // Draw button box.
    if (false) {
      display.drawBox(56, 0, 8, 13);
    }
    else {
      display.drawFrame(56, 0, 8, 13);
    }
  } while ( display.nextPage() );
  delay(100);
}

void InitializeGyro() {
  // Registers as defined by the the L3G4200D datasheet.
  // Only enable X axis
  WriteGyroRegister(CTRL_REG1, 0b1001);
  // Do nothing with the filtering
  WriteGyroRegister(CTRL_REG2, 0b0);
  // Do nothing with the interrupts but enable DRDY.
  WriteGyroRegister(CTRL_REG3, 0b1000);
  // Set full scale to 2000dps and enable block data update.
  WriteGyroRegister(CTRL_REG4, 0b10100000);
  // Do nothing with high pass filtering.
  WriteGyroRegister(CTRL_REG4, 0b0);
}

void WriteGyroRegister(byte address, byte data) {
  Wire.beginTransmission(L3G4200D_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

byte ReadGyroRegister(byte address) {
  Wire.beginTransmission(L3G4200D_Address);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(L3G4200D_Address, 1);
  while(!Wire.available()) {
    // Wait
  }
  return Wire.read();
}