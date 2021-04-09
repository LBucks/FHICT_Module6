#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_64X32_1F_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
uint8_t flipColor = 0;
uint8_t drawColor = 1;

void setup() {
  Serial.begin(9600);
  // display.setI2CAddress(0x3C);
  display.begin();
}

void loop() {
  // Use pages due to limited uno memory.
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x13B_tf);
    display.drawStr(0, 13, "Tijd voor");
    display.drawStr(0, 26, "weekend");
  } while ( display.nextPage() );
  delay(1000);
}