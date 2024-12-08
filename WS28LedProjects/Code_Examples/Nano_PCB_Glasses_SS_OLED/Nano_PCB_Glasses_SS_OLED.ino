#include <Button2.h>
#include <FastLED.h>
#include <StensTimer.h>
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);




// Pin Definitions
const int LED1_PIN = 12;
const int LED2_PIN = 11;
const int LED3_PIN = 10;
const int LED_RED = LED1_PIN;
const int LED_YELLOW = LED2_PIN;
const int LED_BLUE = LED3_PIN;
const int SW1_PIN = 8;
const int SW2_PIN = 7;
const int SW3_PIN = 6;



#define ON 1
#define OFF 0

// Timing Intervals
#define FREQ_10HZ 50 //100 msec delay 50% Duty Cycle
#define FREQ_20HZ 25 //100 msec delay  50% Duty Cycle




// Button and LED state
Button2 button1(SW1_PIN);
Button2 button2(SW2_PIN);
Button2 button3(SW3_PIN);
#define LONG_TAP 500

/* stensTimer variable to be used later in the code */
StensTimer* mainTimer = NULL;
Timer* leftTimer = NULL;
Timer* rightTimer = NULL;
Timer* programTimer = NULL;
unsigned long int dynamic_frequency = 10;
#define LEFT_ACTION 1
#define RIGHT_ACTION 2
#define PROGRAM_ACTION 3
#define PROGRAM_FREQUENCY 20000
#define PROGRAM_ONE 1
#define PROGRAM_TWO 2
#define PROGRAM_THREE 3
unsigned int active_program = PROGRAM_ONE;


#define LEFT_WS2818  4
#define RIGHT_WS2818 2
#define RIGHT_EYE RIGHT_WS2818
#define LEFT_EYE LEFT_WS2818


//Led Arrays
#define NUM_LEDS 8
#define DARK_BLUE 0
#define MEDIUM_BLUE 3
#define YELLOW 6
#define PINK 9
#define RED 11

unsigned long colors[12] = {CRGB::DarkBlue, CRGB::MidnightBlue, CRGB::Navy, CRGB::MediumBlue, CRGB::DarkSlateBlue, CRGB::Aqua , CRGB::Gold, CRGB::DarkViolet, CRGB::DarkMagenta , CRGB::DeepPink, CRGB::Crimson,  CRGB::Red};
CRGB ledColour; // a reference instance that holds the currently selected led colour
CRGB ledsRight[NUM_LEDS];
CRGB ledsLeft[NUM_LEDS];
int leftEyeState = OFF;
int rightEyeState = OFF;


// Define the structure for the content
typedef struct {
  const char *header;
  const char *content_1;
  const char *content_2;
  int x_header;
  int y_header;
  int x_content_1;
  int y_content_1;
  int x_content_2;
  int y_content_2;
} DisplayItem;

char gHeader[25] = "Status";
char gContent1[25] = "Program One";
char gContent2[25] = "Running";

DisplayItem displayItems[] = {
{
  gHeader, gContent1, gContent2, 0, 0, 0, 20, 0, 40}
};


void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void draw_line(unsigned int a)
{
  u8g2.drawLine(0, a, 127, a);
}

// Function to print an array of DisplayItem
void printContent(const DisplayItem items[], int itemCount) {
  u8g2.firstPage();
  do {
    // Iterate through each item in the array
    for (int i = 0; i < itemCount; i++) {
      // Set cursor and print the header
      u8g2.setCursor(items[i].x_header, items[i].y_header);
      u8g2.print(items[i].header);
      // Set cursor and print the first content
      u8g2.setCursor(items[i].x_content_1, items[i].y_content_1);
      u8g2.print(items[i].content_1);
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_2, items[i].y_content_2);
      u8g2.print(items[i].content_2);
      draw_line(14);
    }
  } while (u8g2.nextPage());
}

void movingLineEffectOn(int eye) {
  CRGB * glassLeds;
  if(eye == RIGHT_EYE )
    glassLeds = ledsRight;
  else
    glassLeds = ledsLeft;
    for (int step = 0; step < 5; step++) {
        // Clear previous LEDs
        //FastLED.clear();
        
        // Light up specific LEDs based on step
        switch(step) {
            case 0:
                glassLeds[0] = ledColour;
                break;
            case 1:
                glassLeds[1] = ledColour;
                glassLeds[7] = ledColour;
                break;
            case 2:
                glassLeds[2] = ledColour;
                glassLeds[6] = ledColour;
                break;
            case 3:
                glassLeds[3] = ledColour;
                glassLeds[5] = ledColour;
                break;
            case 4:
                glassLeds[4] = ledColour;
                break;
        }
        
        FastLED.show();
        delay(20);  // Adjust delay to control speed
    }
}

void movingLineEffectOff(int eye) {
  CRGB * glassLeds;
  if(eye == RIGHT_EYE )
    glassLeds = ledsRight;
  else
    glassLeds = ledsLeft;
    for (int step = 0; step < 5; step++) {
        // Clear previous LEDs
        //FastLED.clear();
        
        // Light up specific LEDs based on step
        switch(step) {
            case 0:
                glassLeds[0] = CRGB::Black;
                break;
            case 1:
                glassLeds[1] = CRGB::Black;
                glassLeds[7] = CRGB::Black;
                break;
            case 2:
                glassLeds[2] = CRGB::Black;
                glassLeds[6] = CRGB::Black;
                break;
            case 3:
                glassLeds[3] = CRGB::Black;
                glassLeds[5] = CRGB::Black;
                break;
            case 4:
                glassLeds[4] = CRGB::Black;
                break;
        }
        
        FastLED.show();
        delay(20);  // Adjust delay to control speed
    }
}

void turnOn(int eye)
{
  int i = 0;
  CRGB * glassLeds;
  if(eye == RIGHT_EYE )
    glassLeds = ledsRight;
  else
    glassLeds = ledsLeft;
  for(i =0;i < NUM_LEDS;i++)
  {

    glassLeds[i].r = ledColour.r;
    glassLeds[i].g = ledColour.g;
    glassLeds[i].b = ledColour.b;
    FastLED.show();
  }
  //movingLineEffectOn(ledColour, eye);
}

void turnOff(int eye)
{
  int i = 0;
  CRGB * glassLeds;
  if(eye == RIGHT_EYE )
  {
    //Serial.println(F("Turning off Right"));
    glassLeds = ledsRight;
  }
  else
  {
    //Serial.println(F("Turning off Left"));
    glassLeds = ledsLeft;
  }
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i] = CRGB::Black;
    FastLED.show();
  }
  //movingLineEffectOff(ledColour, eye);
}

void programCallback(Timer* timer){
    int action = timer->getAction();

  //Serial.println(F("Main Timer Callback"));
   /* check if the timer is one we expect */
  if(LEFT_ACTION == action){
    //Serial.println(F("LEFT_ACTION"));
    if(leftEyeState) {
      if(active_program == PROGRAM_THREE)
        movingLineEffectOff(LEFT_WS2818);
      else
        turnOff(LEFT_WS2818);
      leftEyeState = OFF;
    }
    else
    {
        if(active_program == PROGRAM_THREE)
          movingLineEffectOn(LEFT_WS2818);
        else
          turnOn(LEFT_WS2818);   
      leftEyeState = ON;   
    }
  }
  else if(RIGHT_ACTION == action){
   // Serial.println(F("RIGHT_ACTION"));
    if(rightEyeState) {
        if(active_program == PROGRAM_THREE)
          movingLineEffectOff(RIGHT_WS2818);
        else    
          turnOff(RIGHT_WS2818);
      rightEyeState = OFF;
    }
    else
      {
        if(active_program == PROGRAM_THREE)
          movingLineEffectOn(RIGHT_WS2818);
        else  
          turnOn(RIGHT_WS2818);   
        rightEyeState = ON;   
      }
  }
  else if(PROGRAM_ACTION == action){
    if(active_program == PROGRAM_TWO)
    {
      Serial.println(F("PROGRAM_ACTION"));
      Serial.print(" Frequency:  ");
      Serial.println(dynamic_frequency);
      dynamic_frequency = dynamic_frequency + 1;
      setIntervals(dynamic_frequency, 10);
    }
    else if(active_program == PROGRAM_ONE)
    {
      Serial.println(F("PROGRAM_ACTION"));
      Serial.print(" Frequency:  ");
      Serial.println(10);
      setIntervals(10, 10);
    }
    else if(active_program == PROGRAM_THREE)
    {
      Serial.println(F("PROGRAM_ACTION"));
      Serial.print(" Frequency:  ");
      Serial.println(10);
      setIntervals(10, 10);
    }


  }
}


void setIntervals(int left_frequency, int intermediate_frequency)
{
    if(leftTimer)
        mainTimer->deleteTimer(leftTimer);
    leftTimer = mainTimer->setInterval(LEFT_ACTION, left_frequency);  
    if(rightTimer)
      mainTimer->deleteTimer(rightTimer);
    rightTimer = mainTimer->setInterval(RIGHT_ACTION, left_frequency + intermediate_frequency); // was 500
}



// Function to flash an LED for a given duration
void flashLed(int ledPin, unsigned long duration) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        digitalWrite(ledPin, HIGH);
        delay(250);
        digitalWrite(ledPin, LOW);
        delay(250);
    }
}

// Function to turn an LED on steadily for a given duration
void steadyOnLed(int ledPin, unsigned long duration) {
    digitalWrite(ledPin, HIGH);         // Turn on the LED
    delay(duration);                    // Keep it on for the specified duration
    digitalWrite(ledPin, LOW);          // Turn off the LED
}

void longClickButtonSW1()
{
    static bool led1State = false;
    led1State = !led1State;
    digitalWrite(LED1_PIN, led1State ? HIGH : LOW);
    ledColour = colors[PINK];
    strcpy(gContent1,"Colour Selected");
    strcpy(gContent2,"Pink");
}


void longClickButtonSW2()
{
    steadyOnLed(LED_YELLOW, 3000);
    ledColour = colors[YELLOW];
    strcpy(gContent1,"Colour Selected");
    strcpy(gContent2,"Yellow");
}

void shortClickButtonSW2() //yellow
{
    flashLed(LED_YELLOW, 3000); 
    strcpy(gContent1,"Program Selected");
    strcpy(gContent2,"Two");
    active_program = PROGRAM_TWO;
}

void shortClickButtonSW1() //yellow
{
    flashLed(LED_RED, 3000); 
    strcpy(gContent1,"Program Selected");
    strcpy(gContent2,"Three");
    active_program = PROGRAM_THREE;
}


void longClickButtonSW3()
{
    static bool led3State = false;
    led3State = !led3State;
    digitalWrite(LED3_PIN, led3State ? HIGH : LOW);
    ledColour = colors[DARK_BLUE];
    strcpy(gContent1,"Colour Selected");
    strcpy(gContent2,"Blue");
}


void handleTapSW2(Button2& b) {
    // check for really long clicks
    unsigned int duration = b.wasPressedFor();
    if (duration > LONG_TAP) {
    Serial.print(F("SW2: long tap detected: "));
    Serial.println(duration);
    longClickButtonSW2();
    }
    else
    {
      Serial.print(F("SW2: short tap detected: "));
      Serial.println(duration);
      shortClickButtonSW2();
    }
   }

void handleTapSW1(Button2& b) {
    // check for really long clicks
    unsigned int duration = b.wasPressedFor();
    if (duration > LONG_TAP) {
    Serial.print(F("SW1: long tap detected: "));
    Serial.println(duration);
    longClickButtonSW1();
    }
    else
    {
      Serial.print(F("SW1: short tap detected: "));
      Serial.println(duration);
      shortClickButtonSW1();
    }
   }



void handleTapSW3(Button2& b) {
    // check for really long clicks
    unsigned int duration = b.wasPressedFor();
    if (duration > LONG_TAP) {
    Serial.print(F("SW3: long tap detected: "));
    Serial.println(duration);
    longClickButtonSW3();
    }
    else
    {
      Serial.print(F("SW3: short tap detected: "));
      Serial.println(duration);
    }
   }


void setupProgramTimer()
{
  if(programTimer)
    mainTimer->deleteTimer(programTimer);
  programTimer = mainTimer->setInterval(PROGRAM_ACTION, PROGRAM_FREQUENCY); 
}

void setup() {
    // Set up LEDs
    u8g2.begin();
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(RIGHT_WS2818, OUTPUT);
    pinMode(LEFT_WS2818, OUTPUT);
    FastLED.addLeds<WS2812, RIGHT_WS2818, GRB>(ledsRight, NUM_LEDS);
    FastLED.addLeds<WS2812, LEFT_WS2818, GRB>(ledsLeft, NUM_LEDS);
    ledColour = CRGB::White;
    // Initialize serial communication for output
    Serial.begin(9600);
    delay(200);
    Serial.println(F("Initialised Timers"));
    mainTimer = StensTimer::getInstance();
    mainTimer->setStaticCallback(programCallback);
    button1.begin(SW1_PIN);
    button2.begin(SW2_PIN);
    button3.begin(SW3_PIN);

    button1.setTapHandler(handleTapSW1);
    button2.setTapHandler(handleTapSW2);
    button3.setTapHandler(handleTapSW3);

    turnOff(RIGHT_WS2818);
    turnOff(LEFT_WS2818);
    Serial.println(F("Starting Program One"));
    u8g2_prepare();
    u8g2.begin();
    setupProgramTimer();
    setIntervals(10,10);

}





void loop() {
    // Continuously check button states
    printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
    button1.loop();
    button2.loop();
    button3.loop();
    mainTimer->run();
}




