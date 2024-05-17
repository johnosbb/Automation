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

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include "config.h"


WiFiClient wifiClient;
WebServer server(80);
PubSubClient mqttClient(wifiClient);
bool mqtt_connected = 0;
const int led = 13;
char baseMacChr[18] = {0};
bool small_font = 0;

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
      if(mqtt_connected)
        DisplayMessage("Mqtt Connected",1);
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

void DisplayMessage(char * message,int size)
{
      Serial.println(message);
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


void blink_board_led()
{
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
  delay(1000); 
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

void handleNotFound() {
  digitalWrite(led, 1);
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
  digitalWrite(led, 0);
}


void setup_wifi()
{
  WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  WiFi.begin(ssid, password);
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
}
void GetMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    
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


void setup(void) {
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  setup_wifi();
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  GetMacAddress();
  ShowChipDetails();
  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(MQTTCallback);
  Serial.println("MQTT Client Started .... ");


  server.on("/", HandleRoot);
  server.on("/test.svg", drawGraph);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {

    if (!mqttClient.connected()) {
      MQTTConnect();
    }
  delay(2);//allow the cpu to switch to other tasks
  mqttClient.loop();


  server.handleClient();
  delay(2);//allow the cpu to switch to other tasks
}

void drawGraph() {
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
