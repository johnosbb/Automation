/*
  WebLED.ino, Example for the PageBuilder library.
  Copyright (c) 2017, 2020, Hieromon Ikasamo
  https://github.com/Hieromon/PageBuilder
  This software is released under the MIT License.
  https://opensource.org/licenses/MIT

  This example demonstrates the typical behavior of PageBuilder. It also
  represents a basic structural HTML definition with the PageElement.
*/

#define ACCELSTEPPER // https://github.com/adafruit/AccelStepper/blob/master/AccelStepper.h

#ifdef ACCELSTEPPER
// https://arduinoinfo.mywikis.net/wiki/SmallSteppers#Test_Sketch:_Rotate_1_turn_in_each_direction.2C_repeat
/*-----( Import needed libraries )-----*/
#include <AccelStepper.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define FULLSTEP 4
#define HALFSTEP 8
#define IN1 D1
#define IN2 D2
#define IN3 D5
#define IN4 D6
#endif



#define STATIC_IP_ADDRESS
#include "FastLED.h"
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#endif
#include "config.h"

// #include "WebLED.h"   // Only the LED lighting icon


#include <AutoConnectCore.h>
#include <PageBuilder.h>

AutoConnect portal;

// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;
const int numberOfSteps = 10;




#ifdef ACCELSTEPPER
AccelStepper stepper1(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

#endif


//#define VERBOSE
// LED Related -----------------------------------------------------------------
// WS2812
// Define the Pins
#define WS2818_MAIN D0 // red
#define FIRST_FLOOR_LAMP D3 // yellow
#define SECOND_FLOOR_LAMP D4 // green
#define THIRD_FLOOR_LAMP D5 // blue
// How many leds are connected?
#define NUM_LEDS 8
CRGBArray<NUM_LEDS> leds;
CRGB ledReference;
CRGB availableColours[10] = { CRGB::White, CRGB::Blue, CRGB:: CornflowerBlue, CRGB:: DeepSkyBlue, CRGB::DodgerBlue, CRGB::LightBlue ,CRGB:: Cyan, CRGB::Red, CRGB::Orange, CRGB::Green };
unsigned int selectedColour = 0;
bool first_lamp_on = false;
bool main_lamp_on = false;
bool second_lamp_on = false;
bool builtin_lamp_on  =false;

// LED Related -----------------------------------------------------------------



char * ssid  = ssid1;
char * pass = pass1;



static const char _NAV_BAR[] PROGMEM = R"(
<div class="nav-bar common-section">
  <a href="/lighting" class="nav-link">&#128161;</a> <!-- Light bulb icon for Lighting Control -->
  <a href="/motor" class="nav-link">&#9881;</a> <!-- Cogwheel icon for Motor Control -->
  <a href="/home" class="nav-link">&#8962;</a> <!-- House icon for Home Page -->
</div>
)";



static const char _PAGE_LED[] PROGMEM = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP8266 LED Control</title>
  <style type="text/css">
  {{STYLE}}
  </style>
</head>
<body>
<div class="main-container">
  {{NAV_BAR}}
  <div class="header common-section">Lighting Control</div>
  <div class="main-section">
    <div class="lrow">
      <a class="{{BUTTON_STYLE_MAIN}}" href="/lighting?led_main={{LEDIO_ARG_MAIN}}" onclick='setActive(this)'>{{LEDIO_LABEL_MAIN}}</a>
      <div class="label">Main Lamp</div>
    </div>
    <div class="lrow">
      <a class="{{BUTTON_STYLE_FIRST}}" href="/lighting?led_first={{LEDIO_ARG_FIRST}}" onclick='setActive(this)'>{{LEDIO_LABEL_FIRST}}</a>
      <div class="label">First Floor Lamp</div>
    </div>
  <div class="lrow">
      <a class="{{BUTTON_STYLE_SECOND}}" href="/lighting?led_second={{LEDIO_ARG_SECOND}}" onclick='setActive(this)'>{{LEDIO_LABEL_SECOND}}</a>
      <div class="label">Second Floor Lamp</div>
  </div>
  </div>
    <div class="footer common-section">ESP8266 Light House Control V1.0.1</div>
</div>
</body>
</html>
)";



static const char _PAGE_MOTOR[] PROGMEM = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP8266 Motor Control</title>
  <style type="text/css">
  {{STYLE}}
  </style>
</head>
<body>
<div class="main-container">
  {{NAV_BAR}}
  <div class="header common-section">Motor Control</div>
  <div class="main-section">
    <div class="mrow">
      <a class="button motor-button button-green" href="/motor?motor_ccw=On">Move Motor Counter Clockwise</a>
      <a class="button motor-button button-green" href="/motor?motor_center=On">Center Motor</a>
      <a class="button motor-button button-green" href="/motor?motor_cw=On">Move Motor Clockwise</a>
    </div>
  </div>
  <div class="footer common-section">ESP8266 Light House Control V1.0.1: {{MOTOR_LABEL_STATUS}}</div>
</div>
</body>
</html>
)";






static const char _STYLE_MAIN[] PROGMEM = R"(
body {
  -webkit-appearance: none;
}
p {
  font-family: 'Arial', sans-serif;
  font-weight: bold;
  text-align: center;
}
.main-container {
  height: 100vh;
  display: flex;
  flex-direction: column;
}
.common-section {
  height: 5%; 
  background-color: black;
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
  padding-bottom: 10px;
}
.header {
  font-size: 2em; 
}
.footer {
  font-size: 1em; 
}
.nav-bar {
  height: 5%; 
  background-color: black;
  display: flex;
  justify-content: space-around;
  align-items: center;
}
.nav-link {
  color: white;
  text-decoration: none;
  font-size: 1.5em; 
}
.nav-link:hover {
  color: lightgray; 
}
.main-section {
  flex: 1; 
  background-color: white;
  color: black;
  display: flex;
  flex-direction: column;
  justify-content: flex-start; 
  align-items: center;
  overflow-y: auto; 
  padding-top: 20px; 
}
.lrow {
  display: flex;
  flex-direction: column;
  border: 0px solid black;
  padding: 5px;
  margin: 5px 0;
}
.mrow {
  display: flex;
  justify-content: center; 
  align-items: center;
  gap: 10px;
  border: 0px solid black;
  padding: 5px;
  margin: 5px 0;
}
.button {
  border: none;
  align-items: center;
  justify-content: center;
  border-radius: 20px;
  background-color: black;
  color: white;
  padding: 10px 20px;
  margin-bottom: 2px;
  text-align: center;
  transition: background-color 0.3s ease; 
  width: 150px; 
  display: inline-block; 
  box-sizing: border-box; 
}
.motor-button {
  height: 100px;  
 
}
/* Specific styles for lamp buttons */
.lamp-button {
  height: 50px;  

}
.button-red {
  background-color: red;
}
.button-green {
  background-color: green;
}
.button:hover {
  border: 2px solid black; 
}
.button.active {
  background-color: green; 
}
.one a {
  text-decoration: none;
}
.label {
  background-color: white;
  color: black;
  padding: 5px;
  text-align: center;
  border-radius: 10px;
  font-size: 1em; /* Adjust font size to be proportional to the header */
  margin-top: 2px; /* Increased margin to create a gap between button and label */
}
)";

#define BAUDRATE 9600


// ONBOARD_LED is WiFi connection indicator.
// LED_BUILTIN is controlled by the web page.
#ifndef LED_BUILTIN
#define ONBOARD_LED 16    // Different pin assignment by each module
#elif defined(LED_BUILTIN_AUX)
#define ONBOARD_LED LED_BUILTIN_AUX
#else
#define ONBOARD_LED LED_BUILTIN
#endif

// Get an architecture of compiled
String getArch(PageArgument& args) {
#if defined(ARDUINO_ARCH_ESP8266)
  return "ESP8266";
#elif defined(ARDUINO_ARCH_ESP32)
  return "ESP32";
#endif
}




void printPageArguments(const PageArgument& args) {
  for (int i = 0; i < args.size(); ++i) {
    Serial.print("Argument ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(args.argName(i));
    Serial.print(" = ");
    Serial.println(args.arg(i));
  }
}

#ifdef ACCELSTEPPER
void setupAccelStepper()
{
  Serial.println("Testing motor");
  // set the speed and acceleration
  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(100);
  // set target position
  // stepper1.moveTo(stepsPerRevolution);
  stepper1.disableOutputs();
}


void checkAccelStepper()
{
  // check current stepper motor position to invert direction
  if (stepper1.distanceToGo() == 0){
    stepper1.moveTo(-stepper1.currentPosition());
    Serial.println("Changing direction");
  }
  // move the stepper motor (one step at a time)
  stepper1.run();
} 
#endif

void flashLed()
{
    for(int i =0; i++; i < 10)
    {
      digitalWrite(LED_BUILTIN, LOW);
      delay(10);
      digitalWrite(LED_BUILTIN, HIGH);
    }
}







// This function is the logic for LED_BUILTIN on/off.
// It is called from the occurrence of the 'LEDIO' token by PageElement
//  'LightingControl' declaration as following code.
String ArgsMainLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  Serial.print("ArgsMainLamp called: ");
  printPageArguments(args);
  #endif
  // processMainLedArgs(args);
  processLampArgs(args);
  // Feedback the lighting icon depending on actual port value which
  //  indepent from the http request parameter.
  if (main_lamp_on)
    value = "Off";
  else
    value = "On";
  return value;
}





String processMotorArgs(PageArgument& args)
{
    String value = "";
    if (args.hasArg("motor_ccw")) {
        if (args.arg("motor_ccw") == "On") {
              stepper1.enableOutputs();
            	// Rotate CCW slowly at 5 RPM
              Serial.println("Moving Motor Counter Clockwise for 5 steps.");
              #if defined(ACCELSTEPPER)
              stepper1.setSpeed(50);
              stepper1.move(-200);
              // move the stepper motor (one step at a time)
              stepper1.run();
              #endif
              value = "Moved Counter Clockwise " + String(numberOfSteps) + " Steps";
        } else {
            Serial.println("Received an unknown argument");
            flashLed();
        }
    }
    if (args.hasArg("motor_center")) {
        if (args.arg("motor_center") == "On") {  
           stepper1.enableOutputs();          
            value = "Centered Motor";
            #if defined(ACCELSTEPPER)
              stepper1.setSpeed(50);
              stepper1.moveTo(0);
              // move the stepper motor (one step at a time)
              stepper1.run();
              #endif
        }  else {
            Serial.println("Received an unknown argument");
            flashLed();
        }
    }
    if (args.hasArg("motor_cw")) {
        if (args.arg("motor_cw") == "On") {
              stepper1.enableOutputs();
            	// Rotate CW slowly at 5 RPM
              Serial.println("Moving Motor Clockwise for " + String(numberOfSteps) + " steps.");
              #if defined(ACCELSTEPPER)
              stepper1.setSpeed(20);
              stepper1.move(200);
              // move the stepper motor (one step at a time)
              stepper1.run();
              #endif
              value = "Moved Clockwise + String(numberOfSteps) + " Steps";
        } else {
            Serial.println("Received an unknown argument");
            flashLed();
        }
    }
    delay(1);
    return value;
}




void processLampArgs(PageArgument& args)
{
    // Blinks LED_BUILTIN according to value of the http request parameter 'led'.
  if (args.hasArg("led_builtin")) {
    if (args.arg("led_builtin") == "On")
    {
      digitalWrite(LED_BUILTIN, LOW);
      #ifdef VERBOSE
      Serial.println("Setting lamp on");
      #endif
      builtin_lamp_on = true;
    }
    else if (args.arg("led_builtin") == "Off")
    {
      #ifdef VERBOSE
      Serial.println("Setting lamp off");
      #endif
      digitalWrite(LED_BUILTIN, HIGH);
      builtin_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      flashLed();
    }  
  }
  if (args.hasArg("led_main")) {
    if (args.arg("led_main") == "On")
    {
        turn_on();
    }
    else if (args.arg("led_main") == "Off")
    {
      turn_off();
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      flashLed();
    }  
  }
  if (args.hasArg("led_first")) {
    if (args.arg("led_first") == "On")
    {
      digitalWrite(FIRST_FLOOR_LAMP, HIGH);
      first_lamp_on = true;
    }
    else if (args.arg("led_first") == "Off")
    {
      digitalWrite(FIRST_FLOOR_LAMP, LOW);
      first_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      flashLed();
    }  
  }
  if (args.hasArg("led_second")) {
    if (args.arg("led_second") == "On")
    {
      digitalWrite(SECOND_FLOOR_LAMP, HIGH);
      second_lamp_on = true;
    }
    else if (args.arg("led_second") == "Off")
    {
      #ifdef VERBOSE
      Serial.println("Setting second floor lamp off");
      #endif
      digitalWrite(SECOND_FLOOR_LAMP, LOW);
      second_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      flashLed();
    }  
  }
  delay(1);

}



String ArgsFirstFloorLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  Serial.print("ArgsFirstFloorLamp called: ");
  printPageArguments(args);
  #endif
  //processFirstFloorArgs(args);
  processLampArgs(args);
  // Feedback the lighting icon depending on actual port value which
  //  indepent from the http request parameter.
  if (first_lamp_on == true)
    value = "Off";
  else
    value = "On";
  #ifdef VERBOSE
  Serial.print("Returning value :");
  Serial.println(value);
  #endif
  return value;
}

String ArgsSecondFloorLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  Serial.print("ArgsSecondFloorLamp called: ");
  printPageArguments(args);
  #endif
  //processSecondFloorArgs(args);
  processLampArgs(args);
  // Feedback the lighting icon depending on actual port value which
  //  indepent from the http request parameter.
  if (second_lamp_on == true)
    value = "Off";
  else
    value = "On";
  #ifdef VERBOSE
  Serial.print("Returning value :");
  Serial.println(value);
  #endif
  return value;
}

String labelMainLamp(PageArgument& args) {
  #ifdef VERBOSE
  Serial.println("labelMainLamp called");
  printPageArguments(args);
  #endif
  // processMainLedArgs(args);
  processLampArgs(args);
  String value = "";
  // Feedback the lighting icon depending on actual port value which
  //  independent from the http request parameter.
  if (main_lamp_on)
    value = "On";
  else
    value = "Off";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  }
  #endif
  return value;
}

String labelFirstFloorLamp(PageArgument& args) {
  #ifdef VERBOSE
  Serial.println("labelFirstFloorLamp called");
  printPageArguments(args);
  #endif
  //processFirstFloorArgs(args);
  processLampArgs(args);
  String value = "";
  // Feedback the lighting icon depending on actual port value which
  //  independent from the http request parameter.
  if (first_lamp_on == true)
    value = "On";
  else
    value = "Off";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  }
  #endif
  return value;
}



String labelSecondFloorLamp(PageArgument& args) {
  #ifdef VERBOSE
  Serial.println("labelSecondFloorLamp called");
  printPageArguments(args);
  #endif
  //processSecondFloorArgs(args);
  processLampArgs(args);
  String value = "";
  // Feedback the lighting icon depending on actual port value which
  //  independent from the http request parameter.
  if (second_lamp_on == true)
    value = "On";
  else
    value = "Off";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  }
  #endif
  return value;
}


String buttonStyleMainLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  {
    Serial.println("buttonStyleMainLamp called");
    printPageArguments(args);
  }
  #endif
  // processMainLedArgs(args);
  processLampArgs(args);
  if (main_lamp_on)
    value = "button lamp-button button-green";
  else
    value = "button lamp-button button-red";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  } 
  #endif
  return value;
}

String buttonStyleFirstFloorLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  {
    Serial.println("buttonStyleFirstFloorLamp called");
    printPageArguments(args);
  }
  #endif
  processLampArgs(args);
  if (first_lamp_on == true)
    value = "button lamp-button button-green";
  else
    value = "button lamp-button button-red";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  }
  #endif
  return value;
}

String buttonStyleSecondFloorLamp(PageArgument& args) {
  String value = "";
  #ifdef VERBOSE
  {
    Serial.println("buttonStyleSecondFloorLamp called");
    printPageArguments(args);
  }
  #endif
  processLampArgs(args);
  if (second_lamp_on == true)
    value = "button lamp-button button-green";
  else
    value = "button lamp-button button-red";
  #ifdef VERBOSE
  {
    Serial.print("Returning value :");
    Serial.println(value);
  }
  #endif
  return value;
}


// Page construction
PageElement LightingControl(FPSTR(_PAGE_LED), {
  {"NAV_BAR", [](PageArgument& arg) { return String(FPSTR(_NAV_BAR)); } },
  {"STYLE", [](PageArgument& arg) { return String(FPSTR(_STYLE_MAIN)); }},
  {"ARCH", getArch },
  {"LEDIO_ARG_MAIN", ArgsMainLamp },
  {"LEDIO_LABEL_MAIN", labelMainLamp},
  {"BUTTON_STYLE_MAIN", buttonStyleMainLamp},
  {"LEDIO_ARG_FIRST", ArgsFirstFloorLamp },
  {"LEDIO_LABEL_FIRST", labelFirstFloorLamp},
  {"BUTTON_STYLE_FIRST", buttonStyleFirstFloorLamp},
  {"LEDIO_ARG_SECOND", ArgsSecondFloorLamp },
  {"LEDIO_LABEL_SECOND", labelSecondFloorLamp},
  {"BUTTON_STYLE_SECOND", buttonStyleSecondFloorLamp}
});
PageBuilder LEDPage("/lighting", {LightingControl});



// Page construction
PageElement MotorControl(FPSTR(_PAGE_MOTOR), {
  {"NAV_BAR", [](PageArgument& arg) { return String(FPSTR(_NAV_BAR)); } },
  {"STYLE", [](PageArgument& arg) { return String(FPSTR(_STYLE_MAIN)); }},
  {"MOTOR_LABEL_STATUS", processMotorArgs}
});
PageBuilder MotorPage("/motor", {MotorControl});


#if defined(ARDUINO_ARCH_ESP8266)
ESP8266WebServer  Server;
#elif defined(ARDUINO_ARCH_ESP32)
WebServer  Server;
#endif


bool atDetect(IPAddress& softapIP) {
  Serial.println("Captive portal started, SoftAP IP:" + softapIP.toString());
  return true;
}


bool setupAutoConnect()
{
  // Starts user web site included the AutoConnect portal.
  portal.onDetect(atDetect);
  if (portal.begin()) {
    WebServer& server = portal.host();
    Serial.println("Started, IP:" + WiFi.localIP().toString() + " SSID:" + WiFi.SSID());
    return true;
  }
  else {
    Serial.println("Connection failed.");
    while (true) { yield(); }
  }
  return false;
  
}

#ifdef VERBOSE
void show_led_states()
{
  int i = 0;
  CRGB * glassLeds;
  glassLeds = leds;
  #ifdef VERBOSE
    Serial.print("Showing led states\n"); 
  #endif
  for(int i = 0; i < NUM_LEDS; i++) {
    Serial.print(i + 1); // Print LED index (1-based)
    Serial.print(": r=");
    Serial.print(glassLeds[i].r);
    Serial.print(", g=");
    Serial.print(glassLeds[i].g);
    Serial.print(", b=");
    Serial.println(glassLeds[i].b);
  
  }
}
#endif


void turn_on()
{
  int i = 0;
  CRGB * glassLeds;
  glassLeds = leds;
  #ifdef VERBOSE
    Serial.print("Turning On Leds\n");
  #endif 
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i].r = ledReference.r;
    glassLeds[i].g = ledReference.g;
    glassLeds[i].b = ledReference.b;
    FastLED.show();
  }
  main_lamp_on = true;

  //show_led_states();
}

void turn_off()
{
  int i = 0;
  CRGB * glassLeds;
  #ifdef VERBOSE
    Serial.print("Turning Off Leds\n");
  #endif 
  glassLeds = leds;
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i] = CRGB::Black;
    FastLED.show();
    //show_led_states();
  }
  main_lamp_on = false;
}


void led_setup()
{
  // LED Related -----------------------------------------------------------------
  pinMode(WS2818_MAIN, OUTPUT);
  pinMode(FIRST_FLOOR_LAMP, OUTPUT);
  pinMode(SECOND_FLOOR_LAMP, OUTPUT);
  FastLED.addLeds<WS2812,WS2818_MAIN>(leds, NUM_LEDS); //NEOPIXEL
  turn_off();
  //turn_on();
  // LED Related -----------------------------------------------------------------
}


bool setupWifi()
{
  unsigned int numberOfAttempts = 0;
  WiFi.disconnect();
#ifdef STATIC_IP_ADDRESS
  // Setup WiFi network
  #if defined(ARDUINO_ARCH_ESP32)
    WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  #endif  
  WiFi.config(device_ip, gateway_ip, subnet_mask, dns_ip_1, dns_ip_2);
  WiFi.begin(ssid, pass);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
#endif

  do {
    delay(500);
    numberOfAttempts++;
  } while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (numberOfAttempts < 10));
  if(numberOfAttempts  == 10)
  {
    Serial.println("Failed to connect to " + String(ssid) + " after " + numberOfAttempts + " attempts.");
    return false;
  }
  Serial.println("Connected to " + String(ssid));
  return true;
}






void setup() {
  delay(1000);  // for stable the module.
  Serial.begin(BAUDRATE);

  // pinMode(ONBOARD_LED, OUTPUT);
  // digitalWrite(ONBOARD_LED, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Using AutoConnect Methods.");
  setupAutoConnect();
  LEDPage.transferEncoding(PageBuilder::TransferEncoding_t::ByteStream);
  LEDPage.insert(portal.host());
  MotorPage.transferEncoding(PageBuilder::TransferEncoding_t::ByteStream);
  MotorPage.insert(portal.host());
  ledReference = CRGB::White;
  led_setup();
#ifdef ACCELSTEPPER
 setupAccelStepper();

#endif
}

void loop() {
  // if (WiFi.status() == WL_CONNECTED)
  //   digitalWrite(ONBOARD_LED, LOW);
  // else
  //   digitalWrite(ONBOARD_LED, HIGH);
  portal.handleClient();
  stepper1.run();
  if (stepper1.distanceToGo() != 0)
  {
    Serial.print("Seeking targets: Position: ");
    Serial.println(stepper1.currentPosition());
  }
  else
    stepper1.disableOutputs();

  //checkAccelStepper();
  //test_rotation();
  //Server.handleClient();
}
