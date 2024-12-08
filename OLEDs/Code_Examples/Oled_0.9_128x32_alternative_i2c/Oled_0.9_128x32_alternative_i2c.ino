#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

// Use software I²C directly with U8G2
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/8, /* data=*/9, /* reset=*/U8X8_PIN_NONE);

void setup(void) {
  u8g2.begin(); // Initialize the display
}

void loop(void) {
  u8g2.clearBuffer(); // Clear the internal memory
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(8, 29, "MYBOTIC"); // Write to internal memory
  u8g2.sendBuffer(); // Transfer internal memory to the display
  delay(3000);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(31, 24, "your");
  u8g2.sendBuffer();
  delay(800);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(10, 29, "robotic");
  u8g2.sendBuffer();
  delay(800);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr(4, 29, "solution");
  u8g2.sendBuffer();
  delay(2000);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setCursor(0, 15);
  u8g2.print("Hello World!");
  u8g2.sendBuffer();
  delay(2000);
}
