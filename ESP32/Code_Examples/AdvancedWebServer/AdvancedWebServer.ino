/*
   Copyright (c) 2015, Majenko Technologies
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

 * * Neither the name of Majenko Technologies nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <TimerOne.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include <WebServer.h>
#include <ESPmDNS.h>




  
//#define ENABLE_WS_2828_8


#ifdef ENABLE_WS_2828_8
#include "FastLED.h"
// LED Related -----------------------------------------------------------------
// Define the Pins
#define LED_PIN 2
// How many leds are connected?
#define NUM_LEDS 8
CRGBArray<NUM_LEDS> leds;
CRGB ledReference;
CRGB availableColours[10] = { CRGB::White, CRGB::Blue, CRGB:: CornflowerBlue, CRGB:: DeepSkyBlue, CRGB::DodgerBlue, CRGB::LightBlue ,CRGB:: Cyan, CRGB::Red, CRGB::Orange, CRGB::Green };
unsigned int selectedColour = 0;
bool lamp_on = 0;
unsigned int on_time= 0;
// LED Related -----------------------------------------------------------------
#endif

#define VERBOSE 1
#define STATIC_IP_ADDRESS
//#define ENABLE_BLYNK

#ifdef ENABLE_BLYNK
#include <BlynkSimpleEsp32.h>
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#ifdef ENABLE_BLYNK
BlynkTimer timer;
#endif

WiFiClient wifiClient;
bool small_font = 0;
WebServer server(80);
/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient mqttClient(wifiClient);
bool mqtt_connected = 0;
bool blynk_connected = 0;
char baseMacChr[18] = {0};

void blink_board_led()
{
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
  delay(1000); 
}

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


void GetMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    
}


//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void MQTTConnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "";
    if (mqttClient.connect(mqttClientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected to MQTT");
      DisplayMessage("MQTT Connected",1);
      mqtt_connected = 1;
      if(blynk_connected && mqtt_connected)
        DisplayMessage("Blynk and Mqtt|Connected",1);
      mqttClient.subscribe("stat/#");
    } else {
      Serial.print("failed to connect to MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      delay(5000);
      mqtt_connected = 0;
    }
  }
}


void HomePage() {
  // Define the static HTML content as a constant string stored in program memory
  const char htmlContent[] PROGMEM = 
    "<html>\
      <head>\
        <meta http-equiv='refresh' content='5'/>\
        <title>ESP32 Demo</title>\
        <style>\
          body { background-color: #99FFFF; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
        </style>\
      </head>\
      <body>\
        <h1>ESP 32</h1>\
        <p>Uptime: %02d:%02d:%02d</p>\
        <p>Chip ID: %s</p>\
        <p>Number of Cores: %02u</p>\
        <p>Chip Revision: %02u</p>\
        <img src=\"/test.svg\" />\
      </body>\
    </html>";

  // Calculate dynamic content
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  // Create a temporary buffer to hold the formatted HTML content
  char temp[600];
  snprintf(temp, 600, htmlContent, hr, min % 60, sec % 60, baseMacChr, (unsigned short)chip_info.cores,(unsigned short)chip_info.revision);

  // Send the HTML content (static + dynamic) stored in the temporary buffer
  server.send(200, "text/html", temp);
}




void HandleRoot() {
  HomePage();
  blink_board_led();
  Serial.println("Webserver Accessed");
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


void HandleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

#ifdef ENABLE_WS_2828_8
void InitDisplay()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
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
}
#endif

void DisplayMessage(char * message,int size)
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
  display.setTextSize(size); // Draw 1X-scale text was 2
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

int16_t DisplayMessageOLED(const char * message,int x, int y)
{
  // display.setCursor(x, y);
  // display.println(message);
  int16_t line_height = DrawCentreString(message,x,y);
  display.display();      // Show initial text
  delay(100);
  return line_height;
}

//--------------------------------------
// function MQTTCallback called everytime 
// if a mqtt message arrives from the broker
//--------------------------------------
void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
    // Check if the last byte of the payload is a newline character
  if (length > 0 && payload[length - 1] == '\n') {
    Serial.println(" (ends with newline)");
  } else {
    Serial.println(" (does not end with newline)");
  }

  if(strcmp("stat/SENSORS_BACK/GARDEN_PIR", topic) == 0)
  {
    //Blynk.virtualWrite(V1, 1);
    DisplayMessage("Garden PIR",2);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_FRONT_GATE/FRONT_GATE_PIR",topic)== 0)
  {
    //Blynk.virtualWrite(V4, 1);
    DisplayMessage("Front Gate|PIR",2);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_PATIO/PIR1",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Patio PIR",2);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_FRONT/PIR_FRONTDOOR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Front|Door|PIR",1);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_FRONT/LANDING_PIR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Landing|PIR",2);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_ESP32/PIR1",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Backdoor|PIR",2);
    small_font = 0;
  }
  else if(strcmp("stat/SENSORS_FRONT/HALL_PIR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Hall PIR",2);
    small_font = 0;

  }
  else if(strcmp("stat/ESP32LAMP/ON",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Turning|On",2);
    small_font = 0;
    #ifdef ENABLE_WS_2828_8
    ledReference = CRGB::White;
    turn_on();
    #endif
  }
  else if(strcmp("stat/ESP32LAMP/BLUE",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Turning|On",2);
    small_font = 0;
    #ifdef ENABLE_WS_2828_8
    ledReference = CRGB::Blue;
    turn_on();
    #endif
  }
  else if(strcmp("stat/ESP32LAMP/OFF",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Turning|Off",2);
    small_font = 0;
    #ifdef ENABLE_WS_2828_8
    ledReference = CRGB::White;
    turn_off();
    #endif
  }
  else if(strcmp("stat/ESP32LAMP/BRIGHTEN",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    DisplayMessage("Brighten",2);
    small_font = 0;
    #ifdef ENABLE_WS_2828_8
    //brighten(10);
    #endif
  }
  else 
  {
    
 
  }

}

#ifdef ENABLE_BLYNK
// This function sends Arduino's uptime every second to Virtual Pin 2.
void BlynkTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);

}
#endif

// Function to convert IP address to char* array
void ipToString(IPAddress ip, char* output) {
  sprintf(output, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
}


#ifdef ENABLE_WS_2828_8



void turn_on()
{
  int i = 0;
  CRGB * glassLeds;

  glassLeds = leds;
  if(VERBOSE)
    Serial.print("Turning On Leds\n");
  //FastLED.setBrightness(brightness); 
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i].r = ledReference.r;
    glassLeds[i].g = ledReference.g;
    glassLeds[i].b = ledReference.b;
    FastLED.show();
  }

  lamp_on = 1;

}




void turn_off()
{
  int i = 0;
  CRGB * glassLeds;
  if(VERBOSE)
    Serial.print("Turning Off Leds\n"); 
  glassLeds = leds;
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i] = CRGB::Black;
    FastLED.show();
  }
  lamp_on = 0;
}


void led_setup()
{
    // LED Related -----------------------------------------------------------------
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<NEOPIXEL,LED_PIN>(leds, NUM_LEDS);
  ledReference = CRGB::White;
  turn_off();
  // LED Related -----------------------------------------------------------------
}

#endif


void setup_wifi()
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

void setup(void) {
  char  ip[16];

  
  // configure LEDC PWM
  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  #ifdef ENABLE_WS_2828_8
  led_setup();
  #endif
  pinMode(LED_BUILTIN, OUTPUT);      // set the LED pin mode for the onboard led
  Serial.begin(115200);
  setup_wifi();
#ifdef STATIC_IP_ADDRESS  
  // Setup WiFi network
  WiFi.config(device_ip,  gateway_ip, subnet_mask,dns_ip_1,dns_ip_2);
  WiFi.begin(ssid, pass);
#else
  WiFi.begin(ssid, pass);
  Serial.println("WiFi connecting.");
  while (WiFi.status() != WL_CONNECTED) {
      Serial.print("-");
      delay(500);
      Serial.print(".");
  }
#endif
  GetMacAddress();
  Serial.println("");
#ifdef ENABLE_WS_2828_8
  InitDisplay();
#endif  

  DisplayMessage("Wifi Connecting...",1);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  DisplayMessage("Wifi Connected",1);
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  ipToString(WiFi.localIP(), ip);
  DisplayMessage(ip,1);

  // initializes the MDNS service on the Arduino device, enabling it to advertise its presence on the local network and respond to MDNS queries from other devices.
  // if (MDNS.begin("esp32")) {
  //   Serial.println("MDNS responder started");
  // }
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(MQTTCallback);
  Serial.println("MQTT Client Started .... ");
  #ifdef ENABLE_BLYNK
  Serial.print("Starting Blynk .");
  // Setup Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  while (Blynk.connect() == false) {
    Serial.print(". ");
  }
 
  blynk_connected=1;
  Serial.println("Setting Blynk Timer.... ");
  // Setup a function to be called every second
  timer.setInterval(10000L, BlynkTimerEvent);
 #endif
  
  server.on("/", HandleRoot);
  server.on("/test.svg", DrawGraph);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(HandleNotFound);
  server.begin();
  Serial.println(F("HTTP server started"));
  DisplayMessage("HTTP server|started",1);


  ShowChipDetails();
}

void loop(void) {
  if (!mqttClient.connected()) {
      MQTTConnect();
    }
  server.handleClient();
  delay(2);//allow the cpu to switch to other tasks
  mqttClient.loop();
#ifdef ENABLE_BLYNK
  timer.run();

  Blynk.run();
#endif
}

void DrawGraph() {
  String out = "";
  char temp[100];
  out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
  out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
  out += "<g stroke=\"black\">\n";
  int y = rand() % 130;
  for (int x = 10; x < 390; x += 10) {
    int y2 = rand() % 130;
    sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
    out += temp;
    y = y2;
  }
  out += "</g>\n</svg>\n";

  server.send(200, "image/svg+xml", out);
}
