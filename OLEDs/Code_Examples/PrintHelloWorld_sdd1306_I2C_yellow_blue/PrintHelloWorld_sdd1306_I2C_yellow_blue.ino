
#include <Arduino.h>
#include <U8g2lib.h>


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);





const int numMessages = 4; // Number of messages in the array
int currentMessageIndex = 0; // Keeps track of the current message

void printMessage(int i)
{
  const __FlashStringHelper* messages[] = {
  F("Hello World"),
  F("Goodbye World"),
  F("Arduino Rocks!"),
  F("U8g2 Library")
  };
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print(messages[i]);
  } while ( u8g2.nextPage() );
}

void loop(void) {
  printMessage(currentMessageIndex);  // Display current message
  delay(2000);                                   // Delay to observe the message

  // Move to the next message
  currentMessageIndex++;
  if (currentMessageIndex >= numMessages) {
    currentMessageIndex = 0; // Reset to the first message after the last one
  }
}

void setup() {
  u8g2.begin();  // Initialize the display
}


// void printMessage()
// {
//   u8g2.clearBuffer();                // Clear the internal buffer
//   u8g2.setFont(u8g2_font_ncenB14_tr); // Set the font (try a known working font)
//   u8g2.drawStr(0, 30, "Goodbye World"); // Print "Hello World"
//   u8g2.sendBuffer();                 // Send the buffer content to the display
// }


// void loop() {
//   printMessage();
//   delay(1000); // Delay to observe the display
// }
