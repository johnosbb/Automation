
#include <FastLED.h>
#include <ezButton.h>
#include <Keypad.h>
#include <math.h>
#include <stdlib.h>
//#include <Arduino.h>
#include <StensTimer.h>

// Tiny8816mhz

#define LEFT_ACTION 1
#define RIGHT_ACTION 2
#define INHALE_ACTION 3
#define EXHALE_ACTION 4

#define LED_PIN_LEFT_EYE     7
#define LED_PIN_RIGHT_EYE     8
#define NUM_LEDS    12
#define MAX_DELAY 200
#define CYCLE 0
#define FREQ_10HZ 50 //100 msec delay 50% Duty Cycle
#define FREQ_20HZ 25 //100 msec delay  50% Duty Cycle
#define VERBOSE 0
#define KEYBOARD_DEBUG 1
#define SHOW_HSV_TO_RGB_CALCULATIONS 0


const byte ROWS = 1; // number of rows
const byte COLS = 4; // number of columns

char keys[ROWS][COLS] = {
{'1','2','3','4'}
};
byte rowPins[ROWS] = {9};
byte colPins[COLS] = {10,11, 12, 13};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#define VRX_PIN  A0 // Arduino pin connected to VRX pin of joystick
#define VRY_PIN  A1 // Arduino pin connected to VRY pin of joystick
#define SATURATION_PIN  A2 // Arduino pin connected to Saturation potentiometer
#define VALUE_PIN  A3 // Arduino pin connected to Saturation potentiometer
#define SW_PIN   2  // Arduino pin connected to SW pin of joystick
#define ON 1
#define OFF 0

ezButton button(SW_PIN);
bool inHale = true;
bool LockSettings = false;
int SaturationAnalogReading = 0;
int ValueAnalogReading = 0;
int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis
int bValue = 0; // To store value of the button
int frequency1 = FREQ_10HZ;
int frequency2 = FREQ_20HZ;
int dynamicFrequency = 20;
int inhaleFrequency = 5000;
int exhaleFrequency = 7000;
long int timerOne = millis();
int breathTime = 4000;
int inhaleTime = breathTime;
int exhaleTime = breathTime + 2500;

int centre = 508;
double Value = 1.0;
double Saturation = 1.0;
unsigned long color = CRGB::Aqua;
CRGB ledsRight[NUM_LEDS];
CRGB ledsLeft[NUM_LEDS];
CRGB ledReference;
unsigned long colors[12] = {CRGB::DarkBlue, CRGB::MidnightBlue, CRGB::Navy, CRGB::MediumBlue, CRGB::DarkSlateBlue, CRGB::Aqua , CRGB::Indigo, CRGB::DarkViolet, CRGB::DarkMagenta , CRGB::DeepPink, CRGB::Crimson,  CRGB::Red};
int selectedProgram = 1;
/* stensTimer variable to be used later in the code */
StensTimer* mainTimer = NULL;
Timer* leftTimer = NULL;
Timer* rightTimer = NULL;
Timer* inhaleTimer = NULL;
Timer* exhaleTimer = NULL;


int leftEyeState = OFF;
int rightEyeState = OFF;

// Our counter variable
int counter = 0;

void programCallback(Timer* timer){
    int action = timer->getAction();
    if(1)
    {
      if(VERBOSE)  
      {
        Serial.print(action);  
        Serial.print("\n");
      }
    }
   /* check if the timer is one we expect */
  if(LEFT_ACTION == action){
    if(leftEyeState) {
      turnOff(LED_PIN_LEFT_EYE);
      leftEyeState = OFF;
    }
    else
    {
      turnOn(LED_PIN_LEFT_EYE);   
      leftEyeState = ON;   
    }
  }
  else if(RIGHT_ACTION == action){
    if(rightEyeState) {
      turnOff(LED_PIN_RIGHT_EYE);
      rightEyeState = OFF;
    }
    else
      {
        turnOn(LED_PIN_RIGHT_EYE);   
        rightEyeState = ON;   
      }
  }
  else if(INHALE_ACTION == action){
    ledReference.setRGB(255,0,0);     
    turnOn(LED_PIN_RIGHT_EYE);  
    turnOn(LED_PIN_LEFT_EYE);
    Serial.print("programCallback: Inhale action");  
    Serial.print("\n");
    }
  else if(EXHALE_ACTION == action){
    ledReference.setRGB(0,255,255);     
    turnOn(LED_PIN_RIGHT_EYE);  
    turnOn(LED_PIN_LEFT_EYE);
    Serial.print("programCallback: Exhale action");  
    Serial.print("\n");    
    }
  else
  {
    Serial.print("programCallback: Received unknown action");  
    Serial.print("\n");    
  }  
  
}




void programOne()
{
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer);
    leftTimer = mainTimer->setInterval(LEFT_ACTION, 50);  
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, 500);;
}

void programTwo()
{
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer);
    leftTimer = mainTimer->setInterval(LEFT_ACTION, 50);  
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, 50);;
}

void programThree()
{
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer);
    leftTimer = mainTimer->setInterval(LEFT_ACTION, 500);  
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, 500);;
}

void programFour()
{
    dynamicFrequency = dynamicFrequency + 10;
    // if(dynamicFrequency < 50)
    //   dynamicFrequency = 50;
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer);
    leftTimer = mainTimer->setInterval(LEFT_ACTION, dynamicFrequency);  
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
      turnOn(LED_PIN_RIGHT_EYE);  
      turnOn(LED_PIN_LEFT_EYE);
    }
  }    
  else
  {    
    // ledReference.setRGB(0,255,255);     // cyan
    ledReference = CRGB::LightSkyBlue;
    turnOn(LED_PIN_RIGHT_EYE);  
    turnOn(LED_PIN_LEFT_EYE);
  }

  


  
}

void setup() {
  FastLED.addLeds<WS2812, LED_PIN_RIGHT_EYE, GRB>(ledsRight, NUM_LEDS);
  FastLED.addLeds<WS2812, LED_PIN_LEFT_EYE, GRB>(ledsLeft, NUM_LEDS);
  Serial.begin(9600);
    /* Save instance of StensTimer to the tensTimer variable*/
  mainTimer = StensTimer::getInstance();
  /* Tell StensTimer which callback function to use */
  mainTimer->setStaticCallback(programCallback);
  programOne();  
}



void HSVToRGB(double hue, double saturation, double value, int* rgb)
{
  double M = 255 * value;
  double m = M *(1 - saturation);
  double z = (M-m) * (1 - fabs((((int)hue/60)%2)-1));
  int r_prime, g_prime, b_prime;
  if (hue >= 300) {
    r_prime = M;
    g_prime = 0;
    b_prime = z + M;
    
  } else if (hue >= 240) {
    r_prime = z + m;
    g_prime = 0;
    b_prime = M;
    
  } else if (hue >= 180) {
    r_prime = 0;
    g_prime = z + m;
    b_prime = M;
    
  } else if (hue >= 120) {
    r_prime = 0;
    g_prime = M;
    b_prime = z + m;
    
  } else if (hue >= 60) {
    r_prime = z + m;
    g_prime = M;
    b_prime = 0;

  } else { // hue >= 0
    r_prime = M;
    g_prime = z + m;
    b_prime = 0;
  }

    if (SHOW_HSV_TO_RGB_CALCULATIONS) {
    Serial.print("Hue: ");
    Serial.print(hue);
    Serial.print("\n");
    Serial.print("Value: ");
    Serial.print(value);
    Serial.print("\n");
    Serial.print("Saturation: ");
    Serial.print(saturation);
    Serial.print("\n");
    Serial.print("m: ");
    Serial.print(m);
    Serial.print("\n");
    Serial.print("M: ");
    Serial.print(m);
    Serial.print("\n");
    Serial.print("z: ");
    Serial.print(z);
    Serial.print("\n");
    Serial.print("r_prime: ");
    Serial.print(r_prime);
    Serial.print("\n");
    Serial.print("g_prime: ");
    Serial.print(g_prime);
    Serial.print("\n");
    Serial.print("b_prime: ");
    Serial.print(b_prime);
    Serial.print("\n");
  }    

  rgb[0] = (r_prime);
  rgb[1] = (g_prime);
  rgb[2] = (b_prime);

}



void poteniometer()
{

 
  if (!LockSettings)
    SaturationAnalogReading = analogRead(SATURATION_PIN);
  Saturation = (double)((double)SaturationAnalogReading/(double)1023);
  if(VERBOSE)
  {  
    Serial.println("Saturation = ");
    Serial.println(SaturationAnalogReading);
    Serial.println(Saturation); 
  }  
  if (!LockSettings)  
    ValueAnalogReading = analogRead(VALUE_PIN);  
  Value = (double)((double)ValueAnalogReading/(double)1023);
  if(VERBOSE)
  {  
    Serial.println("Val = ");
    Serial.println(ValueAnalogReading);
    Serial.println(Value);
  }
}

void joystick()
{
  char buffer[40];

  if (!LockSettings)
  {
    xValue = analogRead(VRX_PIN);
    yValue = analogRead(VRY_PIN);
  }  
  button.loop();
    // Read the button value
  bValue = button.getState();

  if (button.isPressed()) {
    if(VERBOSE)
      Serial.println("The joystick button is pressed");
    Saturation = 1;
    Value = 1;
    // TODO do something here
  }

  if (button.isReleased()) {
    if(VERBOSE)
      Serial.println("The joystick button is released");
    // TODO do something here
  }
  if(VERBOSE)
  {
    sprintf(buffer,"Raw:  X: %d, Y: %d \n", xValue,yValue);
    Serial.print(buffer);
    Serial.print("\n");
  }
  if ( xValue > 490 && xValue < 534 && yValue > 490 && yValue < 534) {
    ledReference = CRGB::White;

  // Otherwise, calculate hue angle & convert to RGB vals
  // It is assumed/hardcoded for S & V == 1 (in HSV mode)
  } else {

    // Floor round x and y values by the 10's place
    // Eliminates weird arithmetic issues caused by imperfect
    // analog feedback (e.g. max val for each is 1024; only gets to 1018)
    float x_val = xValue/10-51; 
    float y_val = yValue/10-51;

    if (VERBOSE) {
      Serial.print("x_val: ");
      Serial.print(x_val);
      Serial.print("\n");
      Serial.print("y_val: ");
      Serial.print(y_val);
      Serial.print("\n");
    }
        // Use arctan for the hue angle in radians from [-pi,pi]
    double hue = atan2(y_val,x_val);

    if (VERBOSE) {
      Serial.print("Raw Hue: ");
      Serial.print(hue);
      Serial.print("\n\n");
 
    } 
    // Adjust the hue to be in range [0,2pi]
    // then convert the hue from rad to degrees
    hue = hue < 0 ? hue + 2 * 3.14 : hue;
    hue = hue/6.28 * 360;

    if (VERBOSE) {
      Serial.print("Hue: ");
      Serial.print(hue);
      Serial.print("\n\n");
 
    } 

    // Get the RGB vals, change the color accordingly
    int* rgb = (int*) malloc (3 * sizeof(int));
    float v = (abs(x_val)/47)/2 + ((abs(y_val)/47)/2);    
    // getRGBVals(hue, rgb,.2,.2);
    HSVToRGB(hue, Saturation, Value,rgb);
    //getRGBVals(hue, rgb);
  
    ledReference.setRGB(rgb[0],rgb[1],rgb[2]);          
    free(rgb);

  }
}






void turnOn(int eye)
{
  int i = 0;
  CRGB * glassLeds;
  if(eye == LED_PIN_RIGHT_EYE )
    glassLeds = ledsRight;
  else
    glassLeds = ledsLeft;
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
  if(eye == LED_PIN_RIGHT_EYE )
    glassLeds = ledsRight;
  else
    glassLeds = ledsLeft;
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i] = CRGB::Black;
    FastLED.show();
  }
}




void changeProgram()
{
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer); 
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

void handleKeys(byte key)
{
  if(key == '1')
  {
    if(KEYBOARD_DEBUG)
      Serial.println("Received number 1");     
    if(LockSettings)
    {    
      LockSettings   = false;
      Serial.println("LockSettings is disabled");
    }
    else
    {
      LockSettings   = true;
      Serial.println("LockSettings is enabled");
    }
  }
  else if(key == '2')
  {
    if(KEYBOARD_DEBUG)
      Serial.println("Received number 2");
    selectedProgram += 1;      
    if(selectedProgram > 5)
      selectedProgram   = 1;    
    if(KEYBOARD_DEBUG)
    {
      Serial.print("Selected Program is now ");    
      Serial.println(selectedProgram);
    }    
    changeProgram();
  }
  else if(key == '3')
  {
    if(KEYBOARD_DEBUG)    
      Serial.println("Received number 3");
    selectedProgram = 3;
    programThree();
  }
  else if(key == '4')
  {
    if(KEYBOARD_DEBUG)
      Serial.println("Received number 4");

  }
  else
  {
    Serial.println("Received unknown key");
  }

  
}


void loop() {
  char key = keypad.getKey(); 
  char buffer[40];
  if (key != NO_KEY)
  {
      if (VERBOSE) {
        sprintf(buffer,"Key: %c \n", key);
        Serial.print(buffer);
        Serial.print("\n");
      }  
      handleKeys(key);
  }
  if(selectedProgram < 5) // for the lower order programs we use the joystick and potentiometers to determine the colour
  {    
    joystick();    
    poteniometer();
  }  
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
  // frequency = FREQ_10HZ + (xValue * .203);
  frequency1 = FREQ_10HZ + (xValue * .203);
  if (VERBOSE) {
  Serial.print("Saturation: ");
  Serial.print(Saturation);
  Serial.print("\n");
  Serial.print("Value: ");
  Serial.print(Value);
  Serial.print("\n");

  } 
  if(VERBOSE)
  {
    sprintf(buffer," Setting frequency to %d \n", frequency1);
    Serial.print(buffer);
    Serial.print("\n");
  }
 
}
