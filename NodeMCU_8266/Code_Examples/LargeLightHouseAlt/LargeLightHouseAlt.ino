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
#include <ESP8266WiFi.h>
#include "config.h"
#include <WiFiClient.h>
#include <PubSubClient.h>

//#define VERBOSE


// Defines the number of steps per rotation
const int stepsPerRevolution = 2048;


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

// Function to process motor arguments
void processMotor(String direction) {
  Serial.println(F("Processing Motor ...."));
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
    delay(1);
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

void sweep_main_lamp_on()
{
  // turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
  motorState = SWEEP;
}

void sweep_main_lamp_off()
{
  // turn_on(main_leds,NUM_LEDS_MAIN_LAMP,&mainLampColourReference);
  motorState = IDLE;
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
  else if(strcmp("stat/LARGELIGHTHOUSE/SWEEP_ON",topic)== 0)
  {
    sweep_main_lamp_on();
    processMotor("sweep");
    
  }
    else if(strcmp("stat/LARGELIGHTHOUSE/SWEEP_OFF",topic)== 0)
  {
    sweep_main_lamp_off();
    
  }
}




void setup() {
  delay(1000);  // for stable the module.
  Serial.begin(BAUDRATE);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#ifdef STATIC_IP_ADDRESS  
  // Setup WiFi network
  // WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  Serial.println(F("WiFi connecting."));
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

  mainLampColourReference = CRGB::White;
  led_setup();
#ifdef ACCELSTEPPER
  setupAccelStepper();
#endif


  delay(1000);
  mqttClient.setServer(mqtt_server, mqtt_server_port);
  mqttClient.setCallback(MQTTCallback);

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

  #ifdef VERBOSE
    Serial.print("Selected Colour: ");
    Serial.println(selectedColour);
#endif    
  

}
