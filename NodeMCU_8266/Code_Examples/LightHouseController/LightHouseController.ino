/*  
 John O'Sullivan
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

//http://192.168.1.198/lighting

#define STATIC_IP_ADDRESS
#include "FastLED.h"
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#endif
#include "config.h"
#include <NTPClient.h>
// #include "WebLED.h"   // Only the LED lighting icon
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <TimeLib.h>
#include <AutoConnectCore.h>
#include <PageBuilder.h>

AutoConnect portal;

// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


#ifdef ACCELSTEPPER
AccelStepper stepper1(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
// Variables to store the motor control state
enum MotorState { IDLE, SWEEP, MOVE_TO_STEPS, MOVE_TO_NEG_STEPS, MOVE_TO_FINAL };
MotorState motorState = IDLE;
long targetSpeed = 20;
long targetSteps = 200;
#endif

String scheduledSweep="23:30";
String lampTurnOff="23:30";
String lampTurnOn="23:30";
String clearEvents="0";

bool enable_events = false;
//#define VERBOSE
// LED Related -----------------------------------------------------------------
// WS2812
// Define the Pins
#define WS2818_MAIN D0 // yellow
#define WS2812_FIRST_FLOOR_LAMP D4 // blue
#define WS2812_SECOND_FLOOR_LAMP D7 // green
#define THIRD_FLOOR_LAMP D5 // blue
// How many main_leds are connected?
#define NUM_LEDS_MAIN_LAMP 8
#define NUM_LEDS_FIRST_FLOOR_LAMP 1
#define NUM_LEDS_SECOND_FLOOR_LAMP 1
CRGBArray<NUM_LEDS_MAIN_LAMP> main_leds;
CRGBArray<NUM_LEDS_FIRST_FLOOR_LAMP> first_floor_led;
CRGBArray<NUM_LEDS_SECOND_FLOOR_LAMP> second_floor_led;
#define COLOR_ORDER GRB // Change this to the appropriate order for your LEDs
CRGB mainLampColourReference;
CRGB firstFloorLampColourReference;
CRGB secondFloorLampColourReference;
CRGB availableColours[10] = { CRGB::White, CRGB::Blue, CRGB:: CornflowerBlue, CRGB:: DeepSkyBlue, CRGB::DodgerBlue, CRGB::LightBlue ,CRGB:: Cyan, CRGB::Red, CRGB::Orange, CRGB::Green };
unsigned int selectedColour = 0;
bool first_lamp_on = false;
bool main_lamp_on = false;
bool second_lamp_on = false;
bool builtin_lamp_on  =false;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool mqtt_connected = 0;
long long INTERVAL_ELAPSED = 1000000;
long long timerupdate = 0;

String currentTime;
int currentHour = 0;
int currentMinute = 0;
int intervalSweep = 60;
// LED Related -----------------------------------------------------------------

String remainingSweepTime;

char * ssid  = ssid1;
char * pass = pass1;



static const char _NAV_BAR[] PROGMEM = R"(
<div class="nav-bar common-section">
  <a href="/lighting" class="nav-link">&#128161;</a> <!-- Light bulb icon for Lighting Control -->
  <a href="/motor" class="nav-link">&#9881;</a> <!-- Cogwheel icon for Motor Control -->
  <a href="/timers" class="nav-link">&#128339;</a> <!-- Clock icon for Timers Page -->
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
<script type="text/javascript">
  function setColour() {
    const selectedColor = document.getElementById('color-picker').value;
    //window.location.href = `/lighting?led_colour="${selectedColor}"&led_colour=${selectedColor}`;
    const hex = selectedColor.replace("#", "");
    const colour = parseInt(hex, 16);
    window.location.href = `/lighting?led_colour=${colour}`;
  }

</script>
<body>
<div class="main-container">
  {{NAV_BAR}}
  <div class="header common-section">Lighting Control</div>
  <div class="main-section">
    <div class="lrow">
      <a class="{{BUTTON_STYLE_MAIN}}" href="/lighting?led_main={{LEDIO_ARG_MAIN}}" >{{LEDIO_LABEL_MAIN}}</a>
      <div class="label">Main Lamp</div>
    </div>
    <div class="lrow">
      <a class="{{BUTTON_STYLE_FIRST}}" href="/lighting?led_first={{LEDIO_ARG_FIRST}}" >{{LEDIO_LABEL_FIRST}}</a>
      <div class="label">First Floor Lamp</div>
    </div>
    <div class="lrow">
        <a class="{{BUTTON_STYLE_SECOND}}" href="/lighting?led_second={{LEDIO_ARG_SECOND}}" >{{LEDIO_LABEL_SECOND}}</a>
        <div class="label">Second Floor Lamp</div>
    </div>
    <div class="lrow">
        <div class="control-pair">
          <label class="control-section-label" for="color-picker">Choose a color: </label>
          <input type="color" id="color-picker" name="color-picker" value="#ff0000" >
        </div>
    </div>
    <div class="lrow">
        <a class="control-section button"  href="#" onclick='setColour()' >Set Lamp Colour</a>
    </div>
  </div>
    <div class="footer common-section">ESP8266 Light House Control V1.0.3</div>
</div>
</body>
</html>
)";

static const char _PAGE_TIMERS[] PROGMEM = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8" name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP8266 Timer Control</title>
  <style type="text/css">
  {{STYLE}}
  </style>
</head>
<body>
<script type="text/javascript">
  function setTimers() {
    const scheduledSweep = document.getElementById('scheduled-sweep').value;
    const intervalSweep = document.getElementById('interval-sweep').value;
    const lampTurnOff = document.getElementById('lamp-turn-off').value;
    const lampTurnOn = document.getElementById('lamp-turn-on').value;
    window.location.href = `/timers?scheduledSweep=${scheduledSweep}&intervalSweep=${intervalSweep}&lampTurnOff=${lampTurnOff}&lampTurnOn=${lampTurnOn}`;
  }
  function clearEvents() {
    window.location.href = `/timers?clearEvents=1`;
  }
</script>
<div class="main-container">
  {{NAV_BAR}}
  <div class="header common-section">Timer Control</div>
  <div class="main-section">
    <div class="main-section">
      <div class="control-pair">
        <label class="control-section-label" for="scheduled-sweep">Set scheduled sweep:</label>
        <input class="control-section-input" type="time" id="scheduled-sweep" name="scheduled-sweep" value="{{SCHEDULED_SWEEP}}">
      </div>
      <div class="control-pair">
        <label class="control-section-label" for="interval-sweep">Set interval sweep (minutes):</label>
        <input class="control-section-input" type="number" id="interval-sweep" name="interval-sweep" min="10" value="{{INTERVAL_SWEEP}}">
      </div>
      <div class="control-pair">
        <label class="control-section-label" for="lamp-turn-off">Set lamp turn off time (minutes):</label>
        <input class="control-section-input" type="time" id="lamp-turn-off" name="lamp-turn-off" value="{{LAMP_TURN_OFF}}">
      </div>
      <div class="control-pair">
        <label class="control-section-label" for="lamp-turn-on">Set lamp turn on time (minutes):</label>
        <input class="control-section-input" type="time" id="lamp-turn-on" name="lamp-turn-on" value="{{LAMP_TURN_ON}}">
      </div>
      <div class="control-pair">
        <label class="control-section-label" for="current-time">Current Time: {{CURRENT_TIME}}</label>
        <span id="current-time" class="control-section-input"></span>
      </div>
      <div class="control-pair">
          <label class="control-section-label" for="remaining-time">Minutes until next sweep: {{REMAINING_SWEEP_TIME}}</label>
          <span id="remaining-time" class="control-section-input"></span>
      </div>
    <div class="mrow">
      <a class="button timer-button button-green" href="#" onclick='setTimers()'>Set Timers</a>
    </div>
    <div class="mrow">
      <a class="button timer-button button-green" href="#" onclick='clearEvents()'>Clear Events</a>
    </div>
  </div>
  <div class="footer common-section">ESP8266 Timer Control: {{TIMER_LABEL_STATUS}}</div>
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
<script type="text/javascript">
  let speed = 500;
  let steps = 1024;

  function setMotorParameters() {
    speed = document.getElementById('speed').value;
    steps = document.getElementById('steps').value;
  }

  function moveMotor(direction) {
    setMotorParameters();
    window.location.href = `/motor?direction=${direction}&speed=${speed}&steps=${steps}`;
  }
</script>
<div class="main-container">
  {{NAV_BAR}}
  <div class="header common-section">Motor Control</div>
  <div class="main-section">
    <div class="control-section">
      <div class="control-pair">
        <label class="control-section-label" for="speed">Speed (0-1000):</label>
        <input class="control-section-input" type="number" id="speed" name="speed" min="0" max="1000" value="20">
      </div>
      <div class="control-pair">
        <label class="control-section-label" for="steps">Steps (0-2048):</label>
        <input class="control-section-input" type="number" id="steps" name="steps" min="0" max="2048" value="100">
      </div>
    </div>
    <div class="mrow">
      <a class="button motor-button button-green" href="#" onclick='moveMotor("ccw")'>Move Motor Counter Clockwise</a>
      <a class="button motor-button button-green" href="#" onclick='moveMotor("center")'>Center Motor</a>
      <a class="button motor-button button-green" href="#" onclick='moveMotor("cw")'>Move Motor Clockwise</a>
    </div>
    <div class="mrow">
      <a class="button motor-button button-green sweep-button" href="#" onclick='moveMotor("sweep")'>Sweep</a>
    </div>
  </div>
  <div class="footer common-section">ESP8266 Light House Control V1.0.2: {{MOTOR_LABEL_STATUS}}</div>
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
  padding: 5px 10px;
  margin-bottom: 2px;
  text-align: center;
  transition: background-color 0.3s ease; 
  width: 150px; 
  display: inline-block; 
  box-sizing: border-box; 
}
.motor-button {
  height: 60px;  
 
}
/* Specific styles for lamp buttons */
.lamp-button {
  height: 30px;  

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
  font-size: 1em; 
  margin-top: 2px; 
}
.control-section {
  display: flex;
  flex-wrap: wrap;
  gap: 20px; 
}
.control-pair {
  display: flex;
  align-items: center;
}

.timer-control-pair {
    display: block; /* Make each control-pair a block element to stack vertically */
    width: 80%;
    margin-bottom: 15px;
  }

.control-section-input {
  margin-right: 10px; 
}
.control-section-label {
  white-space: nowrap;
  padding: 10px 20px 10px 10px;
}
.control-section button {
  margin: 10px 0;
  padding: 10px 20px;
  border: none;
  border-radius: 10px;
  background-color: black;
  color: white;
  cursor: pointer;
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

// Scheduled Events
struct ScheduledEvent {
  int hour;
  int minute;
  bool executed;
  int interval; // Interval in minutes, 0 if no interval
  unsigned long lastExecution;
  void (*action)();  // Function pointer to the action
};


ScheduledEvent events[10]; // Array to store events



int eventCount = 0;

void addScheduledEvent(const String& timeStr, void (*action)(), int interval = 0) {
  int hour, minute;
  parseTimeString(timeStr, hour, minute);

  if (eventCount < 10) {
    events[eventCount++] = {hour, minute, false, interval, 0, action};
  } else {
    Serial.println("Event list is full");
    eventCount = 0;
  }
}


void parseTimeString(const String& timeStr, int& hour, int& minute) {
  int colonIndex = timeStr.indexOf(':');
  if (colonIndex == -1) {
    Serial.print("Invalid time format : ");
    Serial.println(timeStr);
    return;
  }

  hour = timeStr.substring(0, colonIndex).toInt();
  minute = timeStr.substring(colonIndex + 1).toInt();
}


void printPageArguments(const PageArgument& args) {
  for (int i = 0; i < args.size(); ++i) {
    Serial.print("Argument [");
    Serial.print(i);
    Serial.print("]: [");
    Serial.print(args.argName(i));
    Serial.print("]=[");
    Serial.print(args.arg(i));
    Serial.println("]");
  }
}

#ifdef ACCELSTEPPER
void setupAccelStepper()
{
  Serial.println("Setting up motor");
  // set the speed and acceleration
  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(100);
  // set target position
  // stepper1.moveTo(stepsPerRevolution);
  stepper1.disableOutputs();
}

//--------------------------------------
// function connect called to (re)connect
// to the broker
//--------------------------------------
void MQTTConnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String mqttClientId  = "ESP8266Client-lighthouse";
    if (mqttClient.connect(mqttClientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected to MQTT");
      mqtt_connected = 1;
      if( mqtt_connected)
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


  // Calculate remaining time to next interval
  int remainingTime(unsigned long currentMillis, int interval,unsigned long lastExecution)  {
    if (interval == 0) return -1; // No interval

    unsigned long elapsedMinutes = (currentMillis - lastExecution) / 60000;
    int remainingMinutes = interval - elapsedMinutes;
    return remainingMinutes < 0 ? 0 : remainingMinutes;
  }

void printScheduledEvents(unsigned long currentMillis) {
  Serial.print("Current Time:" );
  Serial.println(currentTime);
  Serial.println("Scheduled Events:");
  for (int i = 0; i < eventCount; i++) {
    Serial.print("Event ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(events[i].hour);
    Serial.print(":");
    if (events[i].minute < 10) {
      Serial.print("0");
    }
    Serial.print(events[i].minute);
    if (events[i].interval > 0) {
      Serial.print(" every ");
      Serial.print(events[i].interval);
      Serial.print(" minutes");
      int remainingMinutes = remainingTime(currentMillis,events[i].interval,events[i].lastExecution);
      Serial.print(" (next in ");
      Serial.print(remainingMinutes);
      Serial.print(" minutes)");
    }
    Serial.println(events[i].executed ? " (executed)" : " (not executed)");
  }
}


String processTimerArgs(PageArgument& args) {
  // Format and return the current timer settings as a status message
  if (args.hasArg("intervalSweep")) {
      intervalSweep = args.arg("intervalSweep").toInt();
      #ifdef VERBOSE
      Serial.print("pTA: intervalSweep:");
      Serial.println(intervalSweep);
      #endif
      if(intervalSweep < 5)
        addScheduledEvent("00:00", sweep_main_lamp, intervalSweep);
      else
        Serial.println("Not setting intervalSweep");
    }
  if (args.hasArg("lampTurnOff")) {
      lampTurnOff = args.arg("lampTurnOff");
      #ifdef VERBOSE
      Serial.print("pTA: lampTurnOff:");
      Serial.println(lampTurnOff);
      #endif
      addScheduledEvent(lampTurnOff,turn_off_all_lamps,0);
    }
    if (args.hasArg("lampTurnOn")) {
      lampTurnOn = args.arg("lampTurnOn");
      #ifdef VERBOSE
      Serial.print("pTA: lampTurnOn:");
      Serial.println(lampTurnOn);
      #endif
      addScheduledEvent(lampTurnOn,turn_on_all_lamps,0);
    }
    if (args.hasArg("scheduledSweep")) {
      scheduledSweep = args.arg("scheduledSweep");
      #ifdef VERBOSE
      Serial.print("pTA: scheduledSweep:");
      Serial.println(scheduledSweep);
      #endif
      addScheduledEvent(scheduledSweep,sweep_main_lamp,0);
    }
    if (args.hasArg("clearEvents")) {
      clearEvents = args.arg("clearEvents");
      #ifdef VERBOSE
      Serial.print("pTA: clearEvents:");
      Serial.println(clearEvents);
      #endif
      eventCount = 0;

    }
    
  #ifdef VERBOSE  
  printPageArguments(args);
  #endif
  String status = "Time: " + currentTime + " " + " S,SS: " + scheduledSweep ;
  status += ",IS: " + String(intervalSweep);
  status += ",LTO: " + lampTurnOff ;
  return status;
}

String getCurrentTime(PageArgument& args) {
  return currentTime;
}





// Function to process motor arguments
String processMotorArgs(PageArgument& args) {
    String value = "";
    if (args.hasArg("speed")) {
        targetSpeed = args.arg("speed").toInt();
        if (targetSpeed < 0) targetSpeed = 0;
        if (targetSpeed > 1000) targetSpeed = 1000;
        value = "Moved at speed " + String(targetSpeed);
    }
    if (args.hasArg("steps")) {
        targetSteps = args.arg("steps").toInt();
        if (targetSteps < 0) targetSteps = 0;
        if (targetSteps > 2048) targetSteps = 2048;
        value += " for " + String(targetSteps) + " steps in the";
    }
    if (args.hasArg("direction")) {
        String direction = args.arg("direction");
        stepper1.setSpeed(targetSpeed); // Set speed for all directions
        stepper1.enableOutputs(); // Enable motor outputs

        if (direction == "ccw") {
            stepper1.moveTo(-targetSteps);
            motorState = IDLE;
        } else if (direction == "cw") {
            stepper1.moveTo(targetSteps);
            motorState = IDLE;
        } else if (direction == "center") {
            stepper1.moveTo(0);
            motorState = IDLE;
        } else if (direction == "sweep") {
            targetSteps = 500;
            targetSpeed = 10;
            stepper1.moveTo(0);
            motorState = SWEEP;
        } else {
            Serial.println("Received an unknown direction");
            flashLed();
        }
        value += " direction " + String(direction);
    }
    delay(1);
    return value;
}


void setColour(unsigned int colorValue)
{
    // Extract RGB components from the decimal color value
    byte red = (colorValue >> 16) & 0xFF;
    byte green = (colorValue >> 8) & 0xFF;
    byte blue = colorValue & 0xFF;
    // Set the LED color
    mainLampColourReference = CRGB(red, green, blue);
    #ifdef VERBOSE
    Serial.print("red:");
    Serial.println(red);
    Serial.print("green:");
    Serial.println(green);
    Serial.print("blue:");
    Serial.println(blue);
    #endif
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
      Serial.println("Recieved an unknown argument, or no arguments");
      printPageArguments(args);
      flashLed();
    }  
  }
  if (args.hasArg("led_main")) {
    if (args.arg("led_main") == "On")
    {
        turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
        main_lamp_on = true;
    }
    else if (args.arg("led_main") == "Off")
    {
      turn_off(main_leds,NUM_LEDS_MAIN_LAMP);
      main_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      printPageArguments(args);
      flashLed();
    }  
  }
  if (args.hasArg("led_first")) {
    if (args.arg("led_first") == "On")
    {
      #ifdef VERBOSE
      Serial.println("Setting first floor lamp on");
      #endif
      turn_on(first_floor_led, NUM_LEDS_FIRST_FLOOR_LAMP,&mainLampColourReference);
      first_lamp_on = true;
    }
    else if (args.arg("led_first") == "Off")
    {
      #ifdef VERBOSE
      Serial.println("Setting first floor lamp off");
      #endif
      turn_off(first_floor_led, NUM_LEDS_FIRST_FLOOR_LAMP);
      first_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      printPageArguments(args);
      flashLed();
    }  
  }
  if (args.hasArg("led_second")) {
    if (args.arg("led_second") == "On")
    {
      #ifdef VERBOSE
      Serial.println("Setting second floor lamp on");
      #endif
      turn_on(second_floor_led, NUM_LEDS_SECOND_FLOOR_LAMP,&mainLampColourReference);
      second_lamp_on = true;
    }
    else if (args.arg("led_second") == "Off")
    {
      #ifdef VERBOSE
      Serial.println("Setting second floor lamp off");
      #endif
      turn_off(second_floor_led, NUM_LEDS_SECOND_FLOOR_LAMP);
      second_lamp_on = false;
    }
    else
    {
      Serial.println("Recieved an unknown argument");
      printPageArguments(args);
      flashLed();
    }  
  }

  if (args.size() > 0) {
      //selectedColour  = args.arg("led_colour")
      #ifdef VERBOSE
      Serial.println("processLampArgs called with colour: ");
      #endif
      int i = 0;
      if(args.argName(i) == "led_colour")
      {
        // Serial.println("String  match!");
        // Serial.print("[");
        // Serial.print(args.argName(i));
        // Serial.print("]=[");
        // Serial.print(args.arg(i));
        // Serial.println("]");
        // Serial.print("Args size: ");
        // Serial.println(args.size());
        int colorValue = args.arg(i).toInt();
        setColour(colorValue);
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

// Page construction for Timers
PageElement TimerControl(FPSTR(_PAGE_TIMERS), {
  {"NAV_BAR", [](PageArgument& arg) { return String(FPSTR(_NAV_BAR)); } },
  {"STYLE", [](PageArgument& arg) { return String(FPSTR(_STYLE_MAIN)); }},
  {"TIMER_LABEL_STATUS", processTimerArgs},
  {"CURRENT_TIME",getCurrentTime},
  {"INTERVAL_SWEEP",[](PageArgument& arg) { return String(intervalSweep); } },
  {"LAMP_TURN_OFF", [](PageArgument& arg) { return lampTurnOff; }},
  {"LAMP_TURN_ON",[](PageArgument& arg) { return lampTurnOn; }},
  { "SCHEDULED_SWEEP",[](PageArgument& arg) { return scheduledSweep; } },
  {"REMAINING_SWEEP_TIME",[](PageArgument& arg) { return String(remainingSweepTime); } }
});
PageBuilder TimerPage("/timers", {TimerControl});


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
void show_led_states(CRGB* device, int number_of_leds)
{
  int i = 0;
  Serial.print("Showing led states\n"); 
  for(int i = 0; i < number_of_leds; i++) {
    Serial.print(i + 1); // Print LED index (1-based)
    Serial.print(": r=");
    Serial.print(device[i].r);
    Serial.print(", g=");
    Serial.print(device[i].g);
    Serial.print(", b=");
    Serial.println(device[i].b);
  
  }
}
#endif

void turn_on_all_lamps()
{
  Serial.println("Turning On all Lamps");
  turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
  turn_on(first_floor_led, NUM_LEDS_FIRST_FLOOR_LAMP,&mainLampColourReference);
  turn_on(second_floor_led, NUM_LEDS_SECOND_FLOOR_LAMP,&mainLampColourReference);
}

void turn_off_all_lamps()
{
  Serial.println("Turning Off all Lamps");
  turn_off(main_leds,NUM_LEDS_MAIN_LAMP);
  turn_off(first_floor_led,NUM_LEDS_FIRST_FLOOR_LAMP);
  turn_off(second_floor_led,NUM_LEDS_SECOND_FLOOR_LAMP);
}

void sweep_main_lamp()
{
  // turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
  motorState = SWEEP;
}

void turn_on(CRGB* device, int number_of_leds,CRGB* colourReference)
{
    int i = 0;
    #ifdef VERBOSE
        Serial.print("Turning On Leds\n");
    #endif 
    for(i = 0; i < number_of_leds; i++)
    {
        device[i].r = colourReference->r;
        device[i].g = colourReference->g;
        device[i].b = colourReference->b;
        #ifdef VERBOSE
        Serial.print(".red:");
        Serial.println(device[i].r);
        Serial.print(".green:");
        Serial.println(device[i].g);
        Serial.print(".blue:");
        Serial.println(device[i].b);
        #endif
        FastLED.show();
    }
    

    //show_led_states(main_leds,NUM_LEDS_MAIN_LAMP);
}


void turn_off(CRGB * device,int number_of_leds)
{
  int i = 0;
  #ifdef VERBOSE
    Serial.print("Turning Off Leds\n");
  #endif 
  for(i =0;i < number_of_leds;i++)
  {
    device[i] = CRGB::Black;
    FastLED.show();
    //show_led_states(main_leds,NUM_LEDS_MAIN_LAMP);
  }
  
}


void led_setup()
{
  // LED Related -----------------------------------------------------------------
  pinMode(WS2818_MAIN, OUTPUT);
  pinMode(WS2812_FIRST_FLOOR_LAMP, OUTPUT);
  pinMode(WS2812_SECOND_FLOOR_LAMP, OUTPUT);
  FastLED.addLeds<WS2812,WS2818_MAIN,COLOR_ORDER>(main_leds, NUM_LEDS_MAIN_LAMP); //NEOPIXEL
  FastLED.addLeds<WS2812,WS2812_FIRST_FLOOR_LAMP,COLOR_ORDER>(first_floor_led, NUM_LEDS_FIRST_FLOOR_LAMP); //NEOPIXEL
  FastLED.addLeds<WS2812,WS2812_SECOND_FLOOR_LAMP,COLOR_ORDER>(second_floor_led, NUM_LEDS_SECOND_FLOOR_LAMP); //NEOPIXEL
  turn_off(main_leds,NUM_LEDS_MAIN_LAMP);

}




void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: '");
  Serial.print(topic);
  Serial.print("' with payload: ");
  // Convert payload to a C-style string
  char payloadStr[length + 1]; // +1 for null terminator
  for (unsigned int i = 0; i < length; i++) {
    payloadStr[i] = (char)payload[i];
  }
  payloadStr[length] = '\0'; // Null-terminate the string

  Serial.println(payloadStr);
  Serial.println();
  if(strcmp("stat/LARGELIGHTHOUSE/MAIN_LAMP_ON", topic) == 0)
  {
      Serial.print("Turning on main lamp");
      turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
  }
  else if(strcmp("stat/LARGELIGHTHOUSE/MAIN_LAMP_OFF",topic)== 0)
  {
      turn_off(main_leds,NUM_LEDS_MAIN_LAMP);
  }
  else if(strcmp("stat/LARGELIGHTHOUSE/ALL_LAMPS_ON", topic) == 0)
  {
      Serial.print("Turning on all lamps");
      turn_on_all_lamps();
  }
  else if(strcmp("stat/LARGELIGHTHOUSE/ALL_LAMPS_OFF",topic)== 0)
  {
      turn_off_all_lamps();
  }
  else if(strcmp("stat/LARGELIGHTHOUSE/SET_COLOUR_BLUE",topic)== 0)
  {
    int colorValue = atoi(payloadStr); // or use strtol() for more control
    Serial.print("Setting colour to: ");
    Serial.println(colorValue);
    setColour(colorValue);
  }
  else if(strcmp("stat/LARGELIGHTHOUSE/SWEEP",topic)== 0)
  {
    sweep_main_lamp();
  }
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
  TimerPage.transferEncoding(PageBuilder::TransferEncoding_t::ByteStream);
  TimerPage.insert(portal.host());
  mainLampColourReference = CRGB::White;
  led_setup();
#ifdef ACCELSTEPPER
  setupAccelStepper();
#endif
  timeClient.setTimeOffset(0);
  timeClient.begin();
  timeClient.update();
  setTime(timeClient.getEpochTime());
  currentTime = timeClient.getFormattedTime();
  if (enable_events)
    addScheduledEvent("23:30", turn_off_all_lamps);

  delay(1000);
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(MQTTCallback);
  //MQTTConnect();
  // else
  //   Serial.println("MQTT Client failed to start .... ");

}


// Handle motor state machine
void handle_motor_state()
{

  switch (motorState) {
    case SWEEP:
      if (stepper1.distanceToGo() == 0) {
        stepper1.moveTo(targetSteps);
        motorState = MOVE_TO_STEPS;
      }
      break;
    case MOVE_TO_STEPS:
      if (stepper1.distanceToGo() == 0) {
        stepper1.moveTo(-targetSteps);
        motorState = MOVE_TO_NEG_STEPS;
      }
      break;
    case MOVE_TO_NEG_STEPS:
      if (stepper1.distanceToGo() == 0) {
        stepper1.moveTo(0);
        motorState = MOVE_TO_FINAL;
      }
      break;
    case MOVE_TO_FINAL:
      if (stepper1.distanceToGo() == 0) {
        motorState = IDLE;
      }
      break;
    case IDLE:
      // Do nothing
      break;
  }

}

void loop() {
    if (!mqttClient.connected()) {
      MQTTConnect();
    }
  if(mqtt_connected)
  {
    delay(2);//allow the cpu to switch to other tasks
    mqttClient.loop();
  }
  // if (WiFi.status() == WL_CONNECTED)
  //   digitalWrite(ONBOARD_LED, LOW);
  // else
  //   digitalWrite(ONBOARD_LED, HIGH);
  portal.handleClient();

  handle_motor_state();

  // Run the motor to the current target position
  stepper1.run();

  if (stepper1.distanceToGo() != 0) {
    #ifdef VERBOSE
    Serial.print("Seeking targets: Position: ");
    Serial.println(stepper1.currentPosition());
    #endif
  } else {
    stepper1.disableOutputs();
  }
  remainingSweepTime = "20";

    // int currentHour = timeClient.getHours();
    //   int currentMinute = timeClient.getMinutes();
    int currentHour = hour();
    int currentMinute = minute();
    unsigned long currentMillis = millis();

    if(enable_events)
    {
      for (int i = 0; i < eventCount; i++) {
        // Handle one-time events
        if (events[i].interval == 0) {
          if (currentHour == events[i].hour && currentMinute >= events[i].minute && !events[i].executed) {
            events[i].action();
            events[i].executed = true;
          }
        }
        // Handle interval-based events
        else {
          unsigned long elapsedMinutes = (currentMillis - events[i].lastExecution) / 60000;
          if ((elapsedMinutes >= events[i].interval) || (events[i].lastExecution == 0)) {
            events[i].action();
            events[i].lastExecution = currentMillis;
          }
        }
      }


      // Reset executed flags at the start of each hour
      if (currentMinute == 0) {
        for (int i = 0; i < eventCount; i++) {
          events[i].executed = false;
        }
      }
    }
    timerupdate++;
    if ( timerupdate == INTERVAL_ELAPSED)
    {
      timeClient.update();
      currentTime = timeClient.getFormattedTime();
      setTime(timeClient.getEpochTime());
      printScheduledEvents(currentMillis);
      timerupdate = 0;
    }

 
  #ifdef VERBOSE
    Serial.println(currentTime);
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.println(timeClient.getMinutes());
    Serial.print("Selected Colour: ");
    Serial.println(selectedColour);
#endif    
  

}
