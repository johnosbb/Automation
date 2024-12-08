#include <Arduino.h>
#include <U8g2lib.h>

// Define the structure for the content
typedef struct {
  const char *header;
  const char *content_1;
  const char *content_2;
  int x_header;
  int y_header;
  int x_content_1;
  int y_content_1;
  int x_content_2;
  int y_content_2;
} DisplayItem;

char gHeader[25] = "Status";
char gContent1[25] = "Program One";
char gContent2[25] = "Running";

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void draw_line(unsigned int a)
{
  u8g2.drawLine(0, a, 127, a);

}

// Function to print an array of DisplayItem
void printContent(const DisplayItem items[], int itemCount) {
  u8g2.firstPage();
  do {
    // Iterate through each item in the array
    for (int i = 0; i < itemCount; i++) {
      // Set cursor and print the header
      u8g2.setCursor(items[i].x_header, items[i].y_header);
      u8g2.print(items[i].header);
      // Set cursor and print the first content
      u8g2.setCursor(items[i].x_content_1, items[i].y_content_1);
      u8g2.print(items[i].content_1);
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_2, items[i].y_content_2);
      u8g2.print(items[i].content_2);
      draw_line(14);
    }
  } while (u8g2.nextPage());
}

void loop(void) {
  DisplayItem displayItems[] = {
    {gHeader, gContent1, gContent2, 0, 0, 0, 20, 0, 40}
  };
  // Call the function to print the content
  printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
  delay(2000);  // Delay to observe the message
}

void setup() {
  u8g2.begin();  // Initialize the display
  u8g2_prepare();
}
