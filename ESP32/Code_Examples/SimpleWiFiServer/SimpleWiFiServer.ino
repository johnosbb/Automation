#include <Arduino.h>
#include <U8g2lib.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include "../../../../Arduino/config.h"
#include <BlynkSimpleEsp32.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
bool mqtt_connected = 0;
bool blynk_connected = 1;

char message[255] = "MQTT Client Loading";
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C D1=SCL,D2=SDA
WiFiClient wifiClient;
bool small_font = 0;

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient mqttClient(wifiClient);

WiFiServer server(80);
void blink()
{
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (LOW is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage HIGH
  delay(1000); 
}




void show_chip_details()
{
    unsigned int chip_id;
    #ifdef ESP8266
    chip_id = ESP.getChipId();
    #elif defined(ESP32)
    chip_id = ESP.getEfuseMac();
    #endif
    Serial.print("chip id: ");
    Serial.println(chip_id);
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
    Serial.print("silicon revision: ");
    Serial.println(chip_info.revision);    
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

  if(strcmp("stat/SENSORS_BACK/GARDEN_PIR", topic) == 0)
  {
    //Blynk.virtualWrite(V1, 1);
    // update_display("Garden");
    small_font = 0;
    strcpy(message,"Garden PIR");

  }
  else if(strcmp("stat/SENSORS_FRONT_GATE/FRONT_GATE_PIR",topic)== 0)
  {
    //Blynk.virtualWrite(V4, 1);
    // update_display("Gate");
    small_font = 0;
    strcpy(message,"Front Gate PIR");
  }
  else if(strcmp("stat/SENSORS_PATIO/PIR1",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Patio PIR");
  }
  else if(strcmp("stat/SENSORS_FRONT/PIR_FRONTDOOR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Front Door PIR");
  }
  else if(strcmp("stat/SENSORS_FRONT/LANDING_PIR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Landing PIR");
  }
  else if(strcmp("stat/SENSORS_ESP32/PIR1",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Backdoor PIR");
  }
  else if(strcmp("stat/SENSORS_FRONT/HALL_PIR",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Hall PIR");
  }
  else if(strcmp("stat/SENSORS/TEST",topic) == 0)
  {
    //Blynk.virtualWrite(V0, 1);
    // update_display("Patio");
    small_font = 0;
    strcpy(message,"Hello World!");
  }
  else 
  {
    
    // // update_display("Patio");
    // small_font = 1;
    // strncpy(message,topic,254);
  }

}




void setup()
{
   
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);      // set the LED pin mode

    u8g2.begin();
    u8g2.clearBuffer();	
    delay(10);
    Serial.println("WiFi Setup.");
    // We start by connecting to a WiFi network
    delay(7000);
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA); //Optional
    WiFi.disconnect();
    delay(100);
    //WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
    // // or
    // WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK); // Lower min security to WPA.
    WiFi.begin(ssid, pass);
    Serial.println("WiFi connecting.");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("-");
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();
    mqttClient.setServer(mqtt_server, mqtt_server_port);
    mqttClient.setCallback(callback);
    show_chip_details();

}

void loop(){
    blink();
    if (!mqttClient.connected()) {
      connect();
    }
    // this paging is required for MCU that have limited ram
    u8g2.firstPage();
    do {
      if(small_font)
        u8g2.setFont(u8g2_font_squeezed_b7_tr); 
      else
        u8g2.setFont(u8g2_font_ncenB14_tr ); 
      u8g2.drawStr(0,24,message);
    } while ( u8g2.nextPage() );
    mqttClient.loop();
    WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(5, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(5, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
