/*
  Elements.ino, Example for the AutoConnect library.
  Copyright (c) 2020, Hieromon Ikasamo
  https://github.com/Hieromon/AutoConnect
  This software is released under the MIT License.
  https://opensource.org/licenses/MIT

  This example demonstrates the typical behavior of AutoConnectElement.
  It also represents a basic structural frame for saving and reusing
  values entered in a custom web page into flash.
*/

// To properly include the suitable header files to the target platform.
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#define FORMAT_ON_FAIL
using WebServerClass = ESP8266WebServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#define FORMAT_ON_FAIL  true
using WebServerClass = WebServer;
#endif
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "time.h"
#include <StensTimer.h>
#include <AutoConnect.h>

#ifdef AUTOCONNECT_USE_LITTLEFS
#include <LittleFS.h>
#if defined(ARDUINO_ARCH_ESP8266)
FS& FlashFS = LittleFS;
#elif defined(ARDUINO_ARCH_ESP32)
fs::LittleFSFS& FlashFS = LittleFS;
#endif
#else
#include <FS.h>
#include <SPIFFS.h>
fs::SPIFFSFS& FlashFS = SPIFFS;
#endif

/* define some custom Action codes */
#define DO_THING_ONE 1
#define DO_THING_TWO 2
#define DO_THING_THREE 3

#define PARAM_FILE      "/elements.json"
#define USERNAME        "admin"   // For HTTP authentication
#define PASSWORD        "1234"   // For HTTP authentication


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI_D1   9 // D1 data wire on OLED SSD1306 SPI also refered to as Master Out Slave In - SDO – Serial Data Out.
#define OLED_CLK_D0   10 // D0 on OLED SSD1306 SPI also refered to as SS Slave Select
#define OLED_DC    11 // data/command DC on OLED SSD1306 SPI also refered to as COPI
#define OLED_CS    12  // chip select CS on OLED SSD1306 SPI
#define OLED_RESET 13 // reset wire RST on OLED SSD1306 SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI_D1, OLED_CLK_D0, OLED_DC, OLED_RESET, OLED_CS);

/* stensTimer variable to be used later in the code */
StensTimer* stensTimer;

const char* ntpServer = "europe.pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 0;

char baseMacChr[18] = {0};


static const char PAGE_ELEMENTS[] PROGMEM = R"(
{
  "uri": "/elements",
  "title": "Bregenz",
  "menu": true,
  "element": [
    {
      "name": "tablecss",
      "type": "ACStyle",
      "value": "table{font-family:arial,sans-serif;border-collapse:collapse;width:100%;color:black;}td,th{border:1px solid #dddddd;text-align:center;padding:8px;}tr:nth-child(even){background-color:#dddddd;}"
    },
    {
      "name": "text",
      "type": "ACText",
      "value": "AutoConnect element behaviors collection",
      "style": "font-family:Arial;font-size:18px;font-weight:400;color:#191970",
      "posterior": "div"
    },
    {
      "name": "check",
      "type": "ACCheckbox",
      "value": "check",
      "label": "Check",
      "labelposition": "infront",
      "checked": true
    },
    {
      "name": "input",
      "type": "ACInput",
      "label": "Text input",
      "placeholder": "This area accepts hostname patterns",
      "pattern": "^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\\-]*[a-zA-Z0-9])\\.)*([A-Za-z0-9]|[A-Za-z0-9][A-Za-z0-9\\-]*[A-Za-z0-9])$"
    },
    {
      "name": "pass",
      "type": "ACInput",
      "label": "Password",
      "apply": "password"
    },
    {
      "name": "number",
      "type": "ACInput",
      "label": "Number",
      "value": "3",
      "apply": "number",
      "pattern": "\\d*"
    },
    {
      "name": "hr",
      "type": "ACElement",
      "value": "<hr style=\"height:1px;border-width:0;color:gray;background-color:#52a6ed\">",
      "posterior": "par"
    },
    {
      "name": "radio",
      "type": "ACRadio",
      "value": [
        "Button-1",
        "Button-2",
        "Button-3"
      ],
      "label": "Radio buttons",
      "arrange": "vertical",
      "checked": 1
    },
    {
      "name": "select",
      "type": "ACSelect",
      "option": [
        "Option-1",
        "Option-2",
        "Option-3"
      ],
      "label": "Select",
      "selected": 2
    },
    {
      "name": "table",
      "type": "ACElement",
      "value": "<table><tr><th>Board</th><th>Platform</th></tr><tr><td>NodeMCU</td><td>Espressif8266</td></tr><tr><td>ESP32-DevKitC</td><td>Espressif32</td></tr></table>",
      "posterior": "par"
    },
    {
      "name": "upload",
      "type": "ACFile",
      "label": "Upload:",
      "store": "fs"
    },
    {
      "name": "load",
      "type": "ACSubmit",
      "value": "Load",
      "uri": "/elements"
    },
    {
      "name": "save",
      "type": "ACSubmit",
      "value": "Save",
      "uri": "/save"
    },
    {
      "name": "adjust_width",
      "type": "ACElement",
      "value": "<script type=\"text/javascript\">window.onload=function(){var t=document.querySelectorAll(\"input[type='text']\");for(i=0;i<t.length;i++){var e=t[i].getAttribute(\"placeholder\");e&&t[i].setAttribute(\"size\",e.length*.8)}};</script>"
    }
  ]
}
)";

static const char PAGE_SAVE[] PROGMEM = R"(
{
  "uri": "/save",
  "title": "Elements",
  "menu": false,
  "element": [
    {
      "name": "caption",
      "type": "ACText",
      "format": "Elements have been saved to %s",
      "style": "font-family:Arial;font-size:18px;font-weight:400;color:#191970"
    },
    {
      "name": "validated",
      "type": "ACText",
      "style": "color:red",
      "posterior": "div"
    },
    {
      "name": "echo",
      "type": "ACText",
      "style": "font-family:monospace;font-size:small;white-space:pre;",
      "posterior": "div"
    },
    {
      "name": "ok",
      "type": "ACSubmit",
      "value": "OK",
      "uri": "/elements"
    }
  ]
}
)";

WebServerClass  server;
AutoConnect portal(server);
AutoConnectConfig config;
AutoConnectAux  elementsAux;
AutoConnectAux  saveAux;



int16_t DrawCentreString(const char *buf, int x, int y)
{
    // char buff[255];
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, x, y, &x1, &y1, &w, &h); //calc width of new string
    int dw = display.width();
    int dh = display.height();
    int x_pos = (dw - w) / 2;
    // sprintf(buff,"x1=%d,y1=%d,w=%d,h=%d, dw=%d dh=%d,c= %d",x1, y1, w, h,dw,dh,x_pos);
    // Serial.println(buff);
    if(x_pos < 0)
      x_pos = 0;
    display.setCursor(x_pos, y);
    display.print(buf);
    return h;
}
int16_t DisplayMessageOLED(const char * message,int x, int y)
{
  // display.setCursor(x, y);
  // display.println(message);
  int16_t line_height = DrawCentreString(message,x,y);
  display.display();      // Show initial text
  delay(100);
  return line_height;
}


void DisplayMessage(char * message,int text_size)
{
  int x = 0;
  int y = 0;
  int16_t line_height = 7;
  // Start index for the current word
  int startIndex = 0;
  String strMessage = String(message);
  // Find the index of the first space character
  int spaceIndex = strMessage.indexOf('|');
  display.clearDisplay();
  display.setTextSize(text_size); // Draw 1X-scale text was 2
  display.setTextColor(SSD1306_WHITE);
  // Tokenize the message
  while (spaceIndex != -1) {
    // Extract the current word using substring
    String word = strMessage.substring(startIndex, spaceIndex);
    line_height = DisplayMessageOLED(word.c_str(),x,y);
    // Update the start index for the next word
    startIndex = spaceIndex + 1;
    // Find the index of the next space character
    spaceIndex = strMessage.indexOf(' ', startIndex);
    y = y + line_height;
    // Serial.print("Sending to display: ");
    // Serial.print(y);
    // Serial.print(" : ");
    // Serial.println(word);
  }

  // Print the last word
  String lastWord = strMessage.substring(startIndex);
  Serial.println(lastWord);
  DisplayMessageOLED(lastWord.c_str(),x,y);
}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}


void displayLocalTime()
{
  char message[255];
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    DisplayMessage("Failed to|obtain time",2);
    return;
  }
    // Format the time information into a string
  strftime(message, 255,"%A| %B %d %Y %H:%M:%S", &timeinfo);;
  DisplayMessage(message,1);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}


void setup_display()
{

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  testdrawchar();      // Draw characters of the default font
}

/* this function will be called whenever a timer is due,
the set timer is passed as a parameter so you can read any property you need. */
void timerCallback(Timer* timer){
  char message[255];
  /* Print some values of this timer */
  Serial.print("Timer call -> Action: ");
  Serial.print(timer->getAction());
  Serial.print(", Current Time: ");
  Serial.println(millis());
  if(timer->getAction() == 3)
  {
    // sprintf(message,"I'm alive %u",millis());
    // DisplayMessage(message,2);
    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    // printLocalTime();
    displayLocalTime();
  }
}



void ShowChipDetails()
{
    unsigned int chip_id;
    chip_id = ESP.getEfuseMac();
    Serial.print("chip id: ");
    Serial.println(baseMacChr);
    Serial.print("chip IP address: ");
    Serial.println(WiFi.localIP());
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.print("chip revision: ");
    Serial.println(chip_info.revision);
    Serial.print("chip cores: ");
    Serial.println(chip_info.cores);
    Serial.print("chip feature BT: ");
    Serial.println((chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "");
    Serial.print("chip feature BLE: ");
    Serial.println((chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    Serial.print("flash size: ");
    unsigned int flash_size = ESP.getFlashChipSize();
    Serial.println(flash_size);
}


void setup_timers()
{
  /* Save instance of StensTimer to the tensTimer variable*/
  stensTimer = StensTimer::getInstance();
      /* Tell StensTimer which callback function to use */
  stensTimer->setStaticCallback(timerCallback);
    /* perform thing_one after 2 seconds */
  stensTimer->setTimer(DO_THING_ONE, 2000);
  /* perform thing_two two times with 4 seconds in between */
  stensTimer->setTimer(DO_THING_TWO, 4000, 2);

  /* perform thing_three every 6 seconds indefiniteley,
  you the timer creation functions return the timer's instance so you can save it */
  Timer* timerThree = stensTimer->setInterval(DO_THING_THREE, 60000);
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  setup_display();
  setup_timers();

  FlashFS.begin(FORMAT_ON_FAIL);

  // Responder of root page handled directly from WebServer class.
  server.on("/", []() {
    String content = "Place the root page with the sketch application.&ensp;";
    content += AUTOCONNECT_LINK(COG_24);
    server.send(200, "text/html", content);
  });

  // Load a custom web page described in JSON as PAGE_ELEMENT and
  // register a handler. This handler will be invoked from
  // AutoConnectSubmit named the Load defined on the same page.
  elementsAux.load(FPSTR(PAGE_ELEMENTS));
  elementsAux.on([] (AutoConnectAux& aux, PageArgument& arg) {
    if (portal.where() == "/elements") {
      // Use the AutoConnect::where function to identify the referer.
      // Since this handler only supports AutoConnectSubmit called the
      // Load, it uses the uri of the custom web page placed to
      // determine whether the Load was called me or not.
      File param = FlashFS.open(PARAM_FILE, "r");
      if (param) {
        aux.loadElement(param, { "text", "check", "input", "input", "pass", "number", "radio", "select" } );
        param.close();
      }
    }
    return String();
  });

  saveAux.load(FPSTR(PAGE_SAVE));
  saveAux.on([] (AutoConnectAux& aux, PageArgument& arg) {
    // You can validate input values before saving with
    // AutoConnectInput::isValid function.
    // Verification is using performed regular expression set in the
    // pattern attribute in advance.
    AutoConnectInput& input = elementsAux["input"].as<AutoConnectInput>();
    aux["validated"].value = input.isValid() ? String() : String("Input data pattern missmatched.");

    // The following line sets only the value, but it is HTMLified as
    // formatted text using the format attribute.
    aux["caption"].value = PARAM_FILE;

    File param = FlashFS.open(PARAM_FILE, "w");
    if (param) {
      // Save as a loadable set for parameters.
      elementsAux.saveElement(param, { "text", "check", "input", "pass", "number", "radio", "select" });
      param.close();
      // Read the saved elements again to display.
      param = FlashFS.open(PARAM_FILE, "r");
      Serial.print("reading param:");
      Serial.println(param.readString());
      aux["echo"].value = param.readString();
      param.close();
    }
    else {
      aux["echo"].value = "Filesystem failed to open.";
    }
    return String();
  });

  portal.join({ elementsAux, saveAux });
  config.auth = AC_AUTH_BASIC;
  config.authScope = AC_AUTHSCOPE_AUX;
  config.username = USERNAME;
  config.password = PASSWORD;
  config.ticker = true;
  config.autoReconnect = true;
  config.reconnectInterval = 1;
  portal.config(config);
  DisplayMessage("Starting|Web Portal",2);
  portal.begin();
  ShowChipDetails();
}

void loop() {
  portal.handleClient();
    /* let StensTimer do it's magic every time loop() is executed */
  stensTimer->run();
}
