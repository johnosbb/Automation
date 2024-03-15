/* Start of Code */
#include <StensTimer.h>
#include "FastLED.h"

// How many leds are connected?
#define NUM_LEDS 8

// Define the Pins
#define LED_PIN 2
#define COLOR_PIN 7
#define MODE_PIN 8
#define MODE_0_LED 11
#define MODE_1_LED 12
#define FREQ_10HZ 50 //100 msec delay 50% Duty Cycle
#define FREQ_20HZ 25 //100 msec delay  50% Duty Cycle
#define LEFT_ACTION 1
#define RIGHT_ACTION 2
#define ON 1
#define OFF 0

StensTimer* mainTimer = NULL;
Timer* leftTimer = NULL;
Timer* rightTimer = NULL;
Timer* inhaleTimer = NULL;
Timer* exhaleTimer = NULL;
#define VERBOSE 1
int leftEyeState = OFF;
int rightEyeState = OFF;

constexpr byte RX {13};
constexpr byte TX {14};
int selectedProgram = 4;
// Define the array of leds
CRGBArray<NUM_LEDS> leds;
CRGB ledReference;
CRGB preferred_color = CRGB::Blue;
unsigned int current_mode = 0;
unsigned int preferred_off_delay = 50;
unsigned int preferred_on_delay = 200;
unsigned int off_delay = preferred_off_delay;
unsigned int on_delay = preferred_on_delay;
unsigned int color = 0;
unsigned int mode = 0;
unsigned long timeNow = millis();
bool inHale = true;

int frequency1 = FREQ_10HZ;
int frequency2 = FREQ_20HZ;
int dynamicFrequency = 20;
int inhaleFrequency = 5000;
int exhaleFrequency = 7000;
long int timerOne = millis();
int breathTime = 4000;
int inhaleTime = breathTime;
int exhaleTime = breathTime + 2500;


int led_color = 0;
#define BRIGHTNESS 200
#define STEPS        300 
#define SPEED       25 
#define SHOW_CONTROL_INPUTS 0

CRGB colors[7] = {CRGB::Blue, CRGB::Red, CRGB::Yellow,CRGB::Green,CRGB::Orange,CRGB::Purple,CRGB::White };
//CRGB colors[7] = {CRGB::Blue, CRGB::Blue CRGB::Blue,CRGB::Blue,CRGB::Blue,CCRGB::Blue,CRGB::Blue };
  
void setup() { 
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<NEOPIXEL,LED_PIN>(leds, NUM_LEDS);
  pinMode(COLOR_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(MODE_0_LED, OUTPUT);
  pinMode(MODE_1_LED, OUTPUT);
  pinMode(A2, INPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  Serial.print("Setting up timer\n");
  mainTimer = StensTimer::getInstance();
  /* Tell StensTimer which callback function to use */
  mainTimer->setStaticCallback(programCallback);
}


void changeProgram()
{ 
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer); 
    if(selectedProgram == 1)
      programOne();
    else if(selectedProgram == 2)  
      programTwo();  
    else if(selectedProgram == 3)  
      programThree(); 
    else if(selectedProgram == 4)  
      programFour(); 
    else if(selectedProgram == 5)  
      programFive(false);      
    else
      ;    
}


void programOne()
{
    ledReference = CRGB::LightSkyBlue;
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, frequency1);
    Serial.println("Setting up program one");
}

void programTwo()
{
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, 50);;
}

void programThree()
{ 
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, 500);;
}

void programFour()
{
    ledReference = CRGB::LightSkyBlue;
    dynamicFrequency = dynamicFrequency + 10; 
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, dynamicFrequency);
    // Serial.print("dynamicFrequency: ");
    // Serial.println(dynamicFrequency);
    
}


void programFive(bool fade)
{        
  if(inHale)
  {    
    if(fade)
    {
      ledReference.fadeLightBy(64);     
    }
    else
    {
      ledReference.setRGB(255,0,0);     //red
      turnOn(LED_PIN);  
    }
  }    
  else
  {    
    // ledReference.setRGB(0,255,255);     // cyan
    ledReference = CRGB::LightSkyBlue;
    turnOn(LED_PIN);  

  }

  


  
}


void programCallback(Timer* timer){
    int action = timer->getAction();

    if(VERBOSE)  
    {
      Serial.print(action);  
      Serial.print("\n");
    }

   /* check if the timer is one we expect */
  if(LEFT_ACTION == action){
    if(leftEyeState) {
      turnOff(LED_PIN);
      leftEyeState = OFF;
    }
    else
    {
      turnOn(LED_PIN);   
      leftEyeState = ON;   
    }
  }
  else if(RIGHT_ACTION == action){
    if(rightEyeState) {
      turnOff(LED_PIN);
      rightEyeState = OFF;
    }
    else
      {
        turnOn(LED_PIN);   
        rightEyeState = ON;   
      }
  }
  else
  {
    Serial.print("programCallback: Received unknown action");  
    Serial.print("\n");    
  }  
  
}

void brighten(int delay)
{
  for(int led = 0; led < NUM_LEDS; led++)
  { 
    leds[led] *= 2;
    FastLED.show();
  }
  FastLED.delay(delay);
  
}


void turnOn(int eye)
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

void turnOff(int eye)
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


void turn_on()
{
  preferred_color = colors[led_color];
  Serial.print("Turn on with color ");
  Serial.print("R: ");
  Serial.print(preferred_color.red);
  Serial.print(" G: ");
  Serial.print(preferred_color.green);
  Serial.print(" B: ");
  Serial.println(preferred_color.blue);
  Serial.print(" Preferred Mode: ");
  Serial.println(led_color);
  Serial.print("\n");
  int brightness = 10;
  for(int led = 0; led < NUM_LEDS; led++)
  { 
    leds[led] = preferred_color;
    FastLED.setBrightness(brightness);
    FastLED.show();
  }
  for  (int i = 0; i < 20;i++)
    brighten(10);
  delay(on_delay);
}


void turn_off()
{
  Serial.print("Turn off\n ");    
  int brightness = 10;
  for(int led = 0; led < NUM_LEDS; led++)
  { 
    leds.fadeToBlackBy(100);
    FastLED.show();
    
  }
  delay(off_delay);
}

void detect_analog_inputs()
{
  on_delay  = analogRead(A2);
  off_delay  = analogRead(A0);
}

void detect_control_inputs()
{
  color = digitalRead(COLOR_PIN); // pin 7
  mode = digitalRead(MODE_PIN); // pin 8
  unsigned long difference = millis() - timeNow;
  if(SHOW_CONTROL_INPUTS)
  {
  Serial.print("On Delay Value: ");
  Serial.println(on_delay);
  Serial.print("Off Delay Value: ");
  Serial.println(off_delay);

  Serial.print("Color Pin: ");
  Serial.println(color);
  Serial.print("Mode Pin: ");
  Serial.println(mode);
  Serial.print("Difference: ");
  Serial.println(difference);
  Serial.print("current_mode: ");
  Serial.println(current_mode);
  }
  if(current_mode == 0)
  {  
    digitalWrite(MODE_0_LED, HIGH);
    digitalWrite(MODE_1_LED, LOW);
  }
  else
  {
    digitalWrite(MODE_1_LED, HIGH);
    digitalWrite(MODE_0_LED, LOW);
  }

  if((difference > 2000))
  { 
  if(mode == 0)
    {

      if(current_mode    == 1)
      {
        current_mode = 0;    
        digitalWrite(MODE_0_LED, HIGH);
        digitalWrite(MODE_1_LED, LOW);    
      }
      else
      {
        current_mode = 1;
        digitalWrite(MODE_1_LED, HIGH);
        digitalWrite(MODE_0_LED, LOW);
        changeProgram();
      }      

      FastLED.delay(1000);
    }  
  }  
}


void loop() { 
  detect_control_inputs(); 
  mainTimer->run();
  if(selectedProgram == 4)
  {
      long int currentTime = millis();   
      if(currentTime - timerOne  > 20000) 
      {                  
        programFour();    
        timerOne = currentTime;      }
  }  
  if(selectedProgram == 5)
  {
      long int currentTime = millis();   
      if(currentTime - timerOne  > breathTime) 
      {       
        if(inHale)  
        {
          inHale = false;
          breathTime = exhaleTime;
        }  
        else
        {
          inHale = true;  
          breathTime = inhaleTime;                    
        }          
        programFive(false);    
        timerOne = currentTime;   
      }
  } 
}





