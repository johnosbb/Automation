/*
  HandlePortal.ino, Example for the AutoConnect library.
  Copyright (c) 2018, Hieromon Ikasamo
  https://github.com/Hieromon/AutoConnect

  This software is released under the MIT License.
  https://opensource.org/licenses/MIT
*/
/*
  This is a way of not explicitly declaring ESP8266WebServer. It uses
  the ESP8266WebServer function without its declaration.
  I recommend that you consider this example compared to HandlePortalEX.ino.
  https://github.com/Hieromon/AutoConnect/blob/master/examples/HandlePortalEX/HandlePortalEX.ino
  It will help you understand AutoConnect usage.
*/

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
using WebServer = ESP8266WebServer;
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#endif
#include <AutoConnectCore.h>
#include "PageBuilder.h"

// This is for 8266
#ifndef BUILTIN_LED
#define LED_ACTIVELEVEL LOW
#define BUILTIN_LED  2  // backward compatibility
#endif

AutoConnect portal;

void handleRoot() {
  String page = PSTR(
"<html>"
"<head>"
  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
  "<style type=\"text/css\">"
    "body {"
    "-webkit-appearance:none;"
    "-moz-appearance:none;"
    "font-family:'Arial',sans-serif;"
    "text-align:center;"
    "}"
    ".main-container {"
    "height:100vh;"
    "display:flex;"
    "flex-direction:column;"
    "}"
    ".header, .footer {"
    "height:10%;"
    "background-color:black;"
    "color:white;"
    "display:flex;"
    "align-items:center;"
    "justify-content:center;"
    "font-size:2em;"
    "}"

    ".main-section {"
    "flex:1;"
    "background-color:white;"
    "color:black;"
    "display:flex;"
    "flex-direction:column;"
    "justify-content:flex-start;"
    "align-items:center;"
    "overflow-y:auto;"
    "}"

    ".row {"
    "display:flex;"
    "flex-direction:column;"
    "align-items:center;"
    "margin-top:10px;"
    "}"
    ".menu > a:link {"
    "position: absolute;"
    "display: inline-block;"
    "right: 12px;"
    "padding: 0 6px;"
    "text-decoration: none;"
    "}"
    ".label {"
    "background-color:white;"
    "color:black;"
    "padding:5px;"
    "border-radius:10px;"
    "font-size:1em;"
    "margin-top:1px;"
    "}"
    ".button {"
    "display:inline-block;"
    "border:none;"
    "border-radius:20px;"
    "background-color:black;"
    "color:white;"
    "padding:10px 20px;"
    "margin-bottom:5px;"
    "transition:background-color 0.3s ease;"
    "}"

    ".button:hover {"
    "background-color:red;"
    "}"
  "</style>"
"</head>"
"<body>"
  // "<div class=\"menu\">" AUTOCONNECT_LINK(BAR_32) "</div>"
  // "BUILT-IN LED<br>"
  // "GPIO(");
  // page += String(BUILTIN_LED);
  // page += String(F(") : <span style=\"font-weight:bold;color:"));
  // page += digitalRead(BUILTIN_LED) ? String("Tomato\">HIGH") : String("SlateBlue\">LOW");
  // page += String(F("</span>"));
  // page += String(F("<p><a class=\"button\" href=\"/io?main=low\">Main Lamp On</a><a class=\"button\" href=\"/io?main=high\">Main Lamp Off</a></p>"));
  // page += String(F("</body></html>"));
  "<div class=\"main-container\">"
  "<div class=\"header\">Lighting</div>"
  "<div class=\"main-section\">"
    "<div class=\"row\">"
    "<a class=\"button\" href=\"/io?main=low\">"
    "On</a>"
    "<div class=\"label\">Main Lamp</div>"
    "</div>"
    "<div class=\"row\">"
    "<a class=\"button\" href=\"/io?main=low\">On</a>"
    "<div class=\"label\">Third Floor</div>"
    "</div>"
    "<div class=\"row\">"
    "<a class=\"button\" href=\"/io?main=low\">On</a>"
    "<div class=\"label\">Second Floor</div>"
    "</div>"
    "<div class=\"row\">"
    "<a class=\"button\" href=\"/io?main=low\">On</a>"
    "<div class=\"label\">First Floor</div>"
    "</div>"
    "<div class=\"row\">"
    "<a class=\"button\" href=\"/io?main=low\">On</a>"
    "<div class=\"label\">All Lamps</div>"
    "</div>"
  "</div>"
  "<div class=\"footer\">Status</div>"
  "</div>"
  "</body>"
  "</body></html>");
  portal.host().send(200, "text/html", page);
}

void sendRedirect(String uri) {
  WebServer& server = portal.host();
  server.sendHeader("Location", uri, true);
  server.send(302, "text/plain", "");
  server.client().stop();
}
void toggleLed()
{
    int currentState = digitalRead(LED_BUILTIN);
    Serial.print("Current State: ");
    Serial.println(currentState);
    // Determine the new state.
    int newState;
    if (currentState == LED_ACTIVELEVEL) {
      // If the LED is currently in the active state, switch it to the inactive state.
      newState = !LED_ACTIVELEVEL;
    } else {
      // If the LED is currently in the inactive state, switch it to the active state.
      newState = LED_ACTIVELEVEL;
    }

    // Write the new state to the LED.
    digitalWrite(LED_BUILTIN, newState);
}


void handleGPIO() {
  WebServer& server = portal.host();
  if (server.arg("main") == "low")
    toggleLed();
  else if (server.arg("main") == "high")
    toggleLed();
  sendRedirect("/");
}

bool atDetect(IPAddress& softapIP) {
  Serial.println("Captive portal started, SoftAP IP:" + softapIP.toString());
  return true;
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println();
  pinMode(BUILTIN_LED, OUTPUT);

  // Put the home location of the web site.
  // But in usually, setting the home uri is not needed cause default location is "/".
  //portal.home("/");   

  // Starts user web site included the AutoConnect portal.
  portal.onDetect(atDetect);
  if (portal.begin()) {
    WebServer& server = portal.host();
    server.on("/", handleRoot);
    server.on("/io", handleGPIO);
    Serial.println("Started, IP:" + WiFi.localIP().toString() + " SSID:" + WiFi.SSID());
  }
  else {
    Serial.println("Connection failed.");
    while (true) { yield(); }
  }
}

void loop() {
  portal.handleClient();
  if (WiFi.status() == WL_IDLE_STATUS) {
#if defined(ARDUINO_ARCH_ESP8266)
    ESP.reset();
#elif defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#endif
    delay(1000);
  }
}
