/* The true ESP32 chip ID is essentially its MAC address.
This sketch provides an alternate chip ID that matches
the output of the ESP.getChipId() function on ESP8266
(i.e. a 32-bit integer matching the last 3 bytes of
the MAC address. This is less unique than the
MAC address chip ID, but is helpful when you need
an identifier that can be no more than a 32-bit integer
(like for switch...case).

created 2020-06-07 by cweinhofer
with help from Cicicok */

#include <U8g2lib.h>
#include <debug.h>
#include "WiFi.h"
#include <config.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Define the structure for the content
typedef struct {
  const char *header;
  const char *content_1;
  const char *content_2;
  const char *content_3;
  int x_header;
  int y_header;
  int x_content_1;
  int y_content_1;
  int x_content_2;
  int y_content_2;
  int x_content_3;
  int y_content_3;
} DisplayItem;

char gHeader[25] = "Status";
char gContent1[25] = "Program: ";
char gContent2[25] = "No Program";
char gContent3[25] = "";

DisplayItem displayItems[] = {
{
  gHeader, gContent1, gContent2,gContent3, 0,0,0,20,80,20,0,40}
};

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define PROGRAM_ONE 1
#define PROGRAM_TWO 2
#define PROGRAM_THREE 3
#define PROGRAM_FOUR 4
#define PROGRAM_IDLE 0
unsigned int active_program = PROGRAM_IDLE;


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
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_3, items[i].y_content_3);
      u8g2.print(items[i].content_3);
      draw_line(14);
    }
  } while (u8g2.nextPage());
}

void updateDisplay()
{
    switch(active_program) {
        case 0:
            strcpy(gContent2,"Idle");
            break;
        case 1: 
            strcpy(gContent2,"One");
            break;
        case 2: 
            strcpy(gContent2,"Two");
            break;
        case 3: 
            strcpy(gContent2,"Three");
            break;
        case 4: 
            strcpy(gContent2,"Four");
            break;
        default:
            strcpy(gContent2,"Error");
            break; 
    }
    printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
}

void setupWifi()
{
  WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
}


void scanWifiNetworks()
{
    DEBUG_PRINT_INFO(F("scan start"));

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    DEBUG_PRINT_INFO(F("scan done"));
    if (n == 0) {
        DEBUG_PRINT_INFO(F("no networks found"));
    } else {
        DEBUG_PRINT_INFO(String(n).c_str() + String(F(" networks found")));
        for (int i = 0; i < n; ++i) {
            // Create the string with SSID and RSSI for each network found
            String networkInfo = String(i + 1) + String(F(": ")) + WiFi.SSID(i) + String(F(" (")) + String(WiFi.RSSI(i)) + String(F(")"));

            // Append encryption type information
            if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) {
                networkInfo += F(" *");
            }

            DEBUG_PRINT_INFO(networkInfo.c_str());
            delay(10);
        }
    }

    DEBUG_PRINT_INFO(F(""));

    // Wait a bit before scanning again
    delay(5000);
}



uint32_t chipId = 0;

void setup() {
  Serial.begin(115200);
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  DEBUG_PRINT_INFO(F("Setting up OLED Display"));
  u8g2_prepare();
  u8g2.begin();
  DEBUG_PRINT_INFO(F("Updating Display"));
  updateDisplay();
  setupWifi();
  WiFi.config(device_ip,  gateway_ip, subnet_mask,dns_ip_1,dns_ip_2);
  WiFi.begin(ssid, pass);
  Serial.println("WiFi connecting.");
  while (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINT_INFO(F("-"));
      delay(500);
      DEBUG_PRINT_INFO(F("."));
  }
}

void loop() {
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: ");
  Serial.println(chipId);

  delay(3000);
}
