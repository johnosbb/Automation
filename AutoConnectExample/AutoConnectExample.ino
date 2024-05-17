#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#endif
#include <time.h>
#include <AutoConnect.h>

WebServer Server;          // Replace with ESP8266WebServer for 8266
AutoConnect      portal(Server);
AutoConnectConfig Config;
uint8_t state;

void rootPage() {
  char content[] = "Hello, world V 1.0.0";
  Server.send(200, "text/plain", content);
}


void onConnect(IPAddress& ipaddr) {
  Serial.print("WiFi connected with ");
  Serial.print(WiFi.SSID());
  Serial.print(", IP:");
  Serial.println(ipaddr.toString());
}


void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Config.autoReset = false;     // Not reset the module even by intentional disconnection using AutoConnect menu.
  Config.autoReconnect = true;  // Reconnect to known access points.
  Config.reconnectInterval = 6; // Reconnection attempting interval is 3[min].
  Config.retainPortal = true;   // Keep the captive portal open.
  portal.config(Config);
  Server.on("/", rootPage);
  portal.onConnect(onConnect); 
  if (portal.begin()) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
  }
  else
    Serial.println("WiFi connection failed to complete.");
}

void loop() {
  portal.handleClient();
  uint8_t transition = portal.portalStatus();
  if (transition != state) {
    if (transition & AutoConnect::AC_CAPTIVEPORTAL)
      Serial.println("Captive portal activated");
    if (transition & AutoConnect::AC_AUTORECONNECT)
      Serial.println("Auto reconnection applied");
    if (!(transition & AutoConnect::AC_ESTABLISHED))
      Serial.println("WiFi connection lost");

    state = transition;
  }

}