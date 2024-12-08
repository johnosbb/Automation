#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <PubSubClient.h>
#include "config.h" // Include your config.h file with WiFi and MQTT details

// Pin definitions
#define RELAY_PIN D0

// Global Variables
ESP8266WiFiMulti wifiMulti;
WiFiClient espClient;              // WiFi client instance
PubSubClient mqttClient(espClient); // Pass WiFiClient to PubSubClient
bool mqtt_connected = false;

// Function prototypes
void MQTTCallback(char* topic, byte* payload, unsigned int length);
void MQTTConnect();
void turn_on_relay();
void turn_off_relay();

// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 5000;

void setup() {
  // Initialize Serial Monitor
  WiFi.persistent(false);
  Serial.begin(9600);
  delay(100);
  Serial.println("\nESP8266 Multi WiFi example");

  // Set WiFi to station mode
  WiFi.mode(WIFI_STA);

  // Register multiple WiFi networks
  wifiMulti.addAP("bregenz1", "aa81990034ae");
  wifiMulti.addAP("hellobackpack", "aa81990034ae");
  wifiMulti.addAP("bregenz2", "aa81990034ae");

  // Configure Relay Pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ensure relay is off initially

  // Set up MQTT
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(MQTTCallback);
}

void loop() {
  // Maintain WiFi connection
  if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      MQTTConnect(); // Reconnect MQTT if necessary
    }
    mqttClient.loop(); // Handle incoming MQTT messages
  } else {
    Serial.println("WiFi not connected!");
  }
}

void MQTTConnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId = "ESP8266Client-smartsocket";
    if (mqttClient.connect(mqttClientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      mqtt_connected = true;
      mqttClient.subscribe("stat/#");
    } else {
      Serial.print("Failed to connect to MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" will try again in 5 seconds");
      delay(5000);
      mqtt_connected = false;
    }
  }
}

void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");

  char payloadStr[length + 1]; // +1 for null terminator
  for (unsigned int i = 0; i < length; i++) {
    payloadStr[i] = (char)payload[i];
  }
  payloadStr[length] = '\0'; // Null-terminate the string

  Serial.println(payloadStr);
  Serial.println();

  if (strcmp("stat/SMARTSOCKET_1/RELAY_1_ON", topic) == 0) {
    Serial.println("Turning on Relay");
    turn_on_relay();
  } else if (strcmp("stat/SMARTSOCKET_1/RELAY_1_OFF", topic) == 0) {
    Serial.println("Turning off Relay");
    turn_off_relay();
  }
}

void turn_on_relay() {
  digitalWrite(RELAY_PIN, LOW); // Relay ON
  Serial.println("Relay is ON");
}

void turn_off_relay() {
  digitalWrite(RELAY_PIN, HIGH); // Relay OFF
  Serial.println("Relay is OFF");
}
