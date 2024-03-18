
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include "../../../../Arduino/linksys_8266_config.h"
#include <BlynkSimpleEsp8266.h>
/* Start of Code */
#include "FastLED.h"
#include <ArduinoJson.h>
#define VERBOSE 1

// Define the Pins
#define LED_PIN 2
// How many leds are connected?
#define NUM_LEDS 9

CRGBArray<NUM_LEDS> leds;
CRGB ledReference;


bool blynk_connected = 0;
bool mqtt_connected = 0;

bool lamp_on = 0;
char message[255] = "MQTT Client Loading";
WiFiClient wifiClient;


/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient mqttClient(wifiClient);

void SendMQTTMessage(char * value)
{
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["POWER"] = value;
  jsonDocument["value"] = 25.5;

  // Convert JSON object to a string
  char messageBuffer[256];
  serializeJson(jsonDocument, messageBuffer);
  mqttClient.publish("stat/LIGHTHOUSE/RESULT", messageBuffer);
}
// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
  strcpy(message,"Connected to Blynk");
  blynk_connected = 1;
}

BLYNK_WRITE(0) 
{
    blink(); 
    if (param.asInt()) {
        //HIGH
        turnOn();
        SendMQTTMessage("ON");
    } else {
       //LOW
       turnOff();
       SendMQTTMessage("OFF");
       
    }
}

BLYNK_WRITE(1) 
{
    blink(); 
    int value = param.asInt();
    if (value) {
        //HIGH
           ledReference = CRGB::Blue;
          if(VERBOSE)
            Serial.print("Enabling Blue\n");
            turnOff();
            turnOn();
    } else {
       //LOW
          ledReference = CRGB::White;
          if(VERBOSE)
            Serial.print("Enabling White\n");
          turnOff();
          turnOn();
       
    }
}

void blink()
{
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
  delay(1000); 
}

//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void connect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "";
    if (mqttClient.connect(mqttClientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected to MQTT");
      strcpy(message,"MQTT Connected");
      mqtt_connected = 1;
      if(blynk_connected && mqtt_connected)
        strcpy(message,"Blynk and Mqtt connected");
      //mqttClient.subscribe("stat/SENSORS_BACK/GARDEN_PIR");
      mqttClient.subscribe("stat/#");
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED off (HIGH is the voltage level)
    } else {
      Serial.print("failed to connect to MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      delay(5000);
      blink();
      mqtt_connected = 0;
    }
  }
}


//--------------------------------------
// function mqtt_callback called everytime 
// if a mqtt message arrives from the broker
//--------------------------------------
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  blink();
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  int result = strcmp("stat/SENSORS_BACK/GARDEN_PIR", topic);
  if(strcmp("stat/SENSORS/LAMP_OFF",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    lamp_on = 0;
    turnOff();
  }
  else if(strcmp("stat/SENSORS/LAMP_ON",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    lamp_on = 1;
    ledReference = CRGB::White;
    turnOn();
  }
  else if(strcmp("stat/SENSORS/LAMP_COLOR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    ledReference = CRGB::Blue;
    turnOff();
    turnOn();
  }

}



void setup(void) {

  pinMode(LED_BUILTIN, OUTPUT);
  // Debug console
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<NEOPIXEL,LED_PIN>(leds, NUM_LEDS);
  Serial.print("WIFI status = ");
  Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("WIFI status = ");
  Serial.println(WiFi.getMode());
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
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(mqtt_callback);

    // Setup Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  while (Blynk.connect() == false) {
  }
  Serial.println(message);
  ledReference = CRGB::White;
  turnOff();
}

void turnOn()
{
  int i = 0;
  CRGB * glassLeds;
  glassLeds = leds;
  if(VERBOSE)
    Serial.print("Turning On Leds\n"); 
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i].r = ledReference.r;
    glassLeds[i].g = ledReference.g;
    glassLeds[i].b = ledReference.b;
    FastLED.show();
  }
}

void turnOff()
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
}


void loop(void) {

  if (!mqttClient.connected()) {
    connect();
  }
  mqttClient.loop();
  Blynk.run();

}

