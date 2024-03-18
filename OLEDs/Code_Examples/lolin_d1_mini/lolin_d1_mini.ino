
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <U8g2lib.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include "../../../../Arduino/linksys_8266_config.h"
#include <BlynkSimpleEsp8266.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

bool blynk_connected = 0;
bool mqtt_connected = 0;

BlynkTimer timer;

char message[255] = "MQTT Client Loading";
WiFiClient wifiClient;



/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient mqttClient(wifiClient);


U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C D1=SCL,D2=SDA



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
    } else {
      Serial.print("failed to connect to MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      delay(5000);
      mqtt_connected = 0;
    }
  }
}


//--------------------------------------
// function callback called everytime 
// if a mqtt message arrives from the broker
//--------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  int result = strcmp("stat/SENSORS_BACK/GARDEN_PIR", topic);
  if(strcmp("stat/SENSORS_BACK/GARDEN_PIR", topic) == 0)
  {
    Blynk.virtualWrite(V1, 1);
    strcpy(message,"Garden PIR");

  }
  else if(strcmp("stat/SENSORS_FRONT_GATE/FRONT_GATE_PIR",topic)== 0)
  {
    Blynk.virtualWrite(V4, 1);
    strcpy(message,"Front Gate PIR");
  }
  else if(strcmp("stat/SENSORS_PATIO/PIR1",topic) == 0)
  {
    Blynk.virtualWrite(V0, 1);
    strcpy(message,"Patio PIR");
  }

}
// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
  // Blynk.virtualWrite(V4, 0);
  // Blynk.virtualWrite(V1, 0);
}


void setup(void) {

  pinMode(LED_BUILTIN, OUTPUT);
  // Debug console
  Serial.begin(9600);
  delay(100);
  u8g2.begin();
  u8g2.clearBuffer();					// clear the internal memory  
  // Setup WiFi network
  WiFi.config(device_ip, dns_ip_1, gateway_ip, subnet_mask);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(callback);

    // Setup Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  while (Blynk.connect() == false) {
  }
  // Setup a function to be called every second
  timer.setInterval(10000L, myTimerEvent);

}

void loop(void) {

    if (!mqttClient.connected()) {
    connect();
  }
  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(0,32,message);



  } while ( u8g2.nextPage() );
  mqttClient.loop();
  timer.run();
  Blynk.run();
  //delay(1000);
}

