#include <U8g2lib.h>
#include <Button2.h>
#include <FastLED.h>
#include <TaskScheduler.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


#define DEBUG_SERIAL  // Uncomment to enable debugging

// Debugging Levels
#define DEBUG_LEVEL_NONE       0  // No debugging
#define DEBUG_LEVEL_ERROR      1  // Errors only
#define DEBUG_LEVEL_WARN       2  // Warnings and Errors
#define DEBUG_LEVEL_INFO       3  // Info, Warnings, and Errors
#define DEBUG_LEVEL_DEBUG      4  // Basic Debugging
#define DEBUG_LEVEL_VERBOSE    5  // Verbose Debugging
#define DEBUG_LEVEL_VERY_VERBOSE 6  // Very Verbose Debugging
#define LONG_TAP 500

// Set the desired debug level here
#define DEBUG_LEVEL DEBUG_LEVEL_DEBUG

#ifdef DEBUG_SERIAL
  // Error
  #define DEBUG_PRINT_ERROR(x)  if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR) { Serial.print("[ERROR] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Warning
  #define DEBUG_PRINT_WARN(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_WARN) { Serial.print("[WARNING] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Info
  #define DEBUG_PRINT_INFO(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO) { Serial.print("[INFO] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Basic Debug
  #define DEBUG_PRINT(x)        if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) { Serial.print("[DEBUG] "); Serial.print(x); }
  #define DEBUG_PRINTLN(x)      if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) { Serial.print("[DEBUG] "); Serial.println(x); }

  // Verbose Debug
  #define DEBUG_PRINT_VERBOSE(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_VERBOSE) { Serial.print("[VERBOSE] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Very Verbose Debug
  #define DEBUG_PRINT_VERY_VERBOSE(x) if (DEBUG_LEVEL >= DEBUG_LEVEL_VERY_VERBOSE) { Serial.print("[VERY_VERBOSE] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }
#else
  #define DEBUG_PRINT_ERROR(x)
  #define DEBUG_PRINT_WARN(x)
  #define DEBUG_PRINT_INFO(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_VERBOSE(x)
  #define DEBUG_PRINT_VERY_VERBOSE(x)
#endif

#define LEFT_WS2818  4
#define RIGHT_WS2818 2
#define RIGHT_EYE RIGHT_WS2818
#define LEFT_EYE LEFT_WS2818
int currentEye = RIGHT_EYE;
#define ON 1
#define OFF 0
#define NUM_LEDS 16
unsigned long colors[12] = {CRGB::DarkBlue, CRGB::MidnightBlue, CRGB::Navy, CRGB::MediumBlue, CRGB::DarkSlateBlue, CRGB::Aqua , CRGB::Gold, CRGB::DarkViolet, CRGB::DarkMagenta , CRGB::DeepPink, CRGB::Crimson,  CRGB::Red};
CRGB ledColour; // a reference instance that holds the currently selected led colour
CRGB ledsRight[NUM_LEDS];
CRGB ledsLeft[NUM_LEDS];
int leftEyeState = OFF;
int rightEyeState = OFF;


Scheduler runner;

#if __cplusplus >= 201103L
  #define HAS_CONSTEXPR 1
#else
  #define HAS_CONSTEXPR 0
#endif

#if  HAS_CONSTEXPR
// Pin Definitions using constexpr
constexpr int LED2_PIN = 11;
constexpr int LED_YELLOW = LED2_PIN;
constexpr int SW1_PIN = 8;
constexpr int SW2_PIN = 7;
constexpr int SW3_PIN = 6;
constexpr int POT_PIN_1 = A0; // First potentiometer on pin A0
constexpr int POT_PIN_2 = A2; // Second potentiometer on pin A1
#else
// Pin Definitions using #define
#define LED2_PIN 11
#define LED_YELLOW LED2_PIN
#define SW2_PIN 7
#define POT_PIN_1 A0 // First potentiometer on pin A0
#define POT_PIN_2 A1 // Second potentiometer on pin A1

#endif

Button2 button2(SW2_PIN);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Define the structure for the content
typedef struct {
  const char *header;
  const char *content_1;
  const char *content_2;
  const char *content_3;
  int x_header;
  int y_header;
  int x_content_1;
  int y_content_1;
  int x_content_2;
  int y_content_2;
  int x_content_3;
  int y_content_3;
} DisplayItem;

char gHeader[25] = "Status";
char gContent1[25] = "Program: ";
char gContent2[25] = "No Program";
char gContent3[25] = "";


#define PROGRAM_ONE 1
#define PROGRAM_TWO 2
#define PROGRAM_THREE 3
#define PROGRAM_FOUR 4
#define PROGRAM_IDLE 0
unsigned int active_program = PROGRAM_IDLE;
int currentStep = 0;
int normalizedBaseFrequency  = 10;


DisplayItem displayItems[] = {
{
  gHeader, gContent1, gContent2,gContent3, 0,0,0,20,80,20,0,40}
};

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

unsigned long int dynamicInterval = 10;
void leftEyeCallback();
void rightEyeCallback();
void sequenceCallback();

Task leftEyeTask(10, TASK_FOREVER, &leftEyeCallback);
Task rightEyeTask(10, TASK_FOREVER, &rightEyeCallback);
Task sequenceTask(100, TASK_FOREVER, &sequenceCallback);

void leftEyeCallback()
{
    DEBUG_PRINT_VERBOSE(F("LEFT_EYE_ACTION"));
    if(leftEyeState) // if the eye is on
    {     
        DEBUG_PRINT_VERY_VERBOSE(F("leftEyeState is on, turning off"));
        turnOff(LEFT_WS2818); // turn it off
        leftEyeState = OFF;
    }
    else // it is off
    {

        if(active_program == PROGRAM_TWO)
        { 
          DEBUG_PRINT_VERY_VERBOSE(F("leftEyeState is off, turning on"));
          dynamicInterval = dynamicInterval + 1;
          leftEyeTask.setInterval(dynamicInterval);
          rightEyeTask.setInterval(dynamicInterval + 10);
          turnOn(LEFT_WS2818); 
        }
        else if(active_program == PROGRAM_ONE)
        {
          DEBUG_PRINT_VERY_VERBOSE(F("leftEyeState is off, turning on"));
          turnOn(LEFT_WS2818);

        }
      leftEyeState = ON;   
    }
}

void rightEyeCallback()
{
    DEBUG_PRINT_VERBOSE(F("RIGHT_EYE_ACTION"));
    if(rightEyeState) // if the eye is on
    {
      DEBUG_PRINT_VERY_VERBOSE(F("rightEyeState is on, turning off"));
      turnOff(RIGHT_WS2818); // turn it off
      rightEyeState = OFF;
    }
    else //it is off
    {
       if(active_program == PROGRAM_TWO)
        {
          turnOn(RIGHT_WS2818);
        }
        else if(active_program == PROGRAM_ONE)
        {
          DEBUG_PRINT_VERY_VERBOSE(F("rightEyeState is off, turning on"));
          turnOn(RIGHT_WS2818);   
        }
      rightEyeState = ON;   
    }
}

void programThree() {
    static CRGB *glassLeds;         // Pointer to the current LED array
    static bool movingForward = true; // Direction: true = forward, false = reverse
    static bool holdAtEnd = false;    // Flag to hold step 3 for one cycle
    if(currentEye == LEFT_EYE)
    {
      glassLeds =  ledsRight;
    }
    else
    {
      glassLeds =  ledsLeft;
    }
    // Clear all LEDs
    for (int i = 0; i < 16; i++) {
        glassLeds[i] = CRGB::Black;
    }
    // Light up the current LED
    glassLeds[currentStep] = ledColour;

    // Show the updated LED state
    FastLED.show();

    currentStep++;
    if(currentStep > 3)
    {
      if(currentEye == LEFT_EYE)
      {
        glassLeds =  ledsRight;
        currentStep = 0;
        currentEye = RIGHT_EYE;
      }
      else
      {
        glassLeds =  ledsLeft;
        currentStep = 0;
        currentEye = LEFT_EYE;
      }
    }

}





void programFour() {
    static CRGB *glassLeds; // Static to persist between calls
    // Set the LEDs array pointer based on the current eye
    if (currentStep == 0) { // Only update on the first step of the sequence
        glassLeds = (currentEye == RIGHT_EYE) ? ledsRight : ledsLeft;
    }
    // Clear previous step
    for (int i = 0; i < 16; i++) {
        glassLeds[i] = CRGB::Black; // Assuming CRGB::Black turns LEDs off
    }
    // Light up LEDs based on the current step
    switch (currentStep) {
        case 0:
            glassLeds[0] = ledColour;
            glassLeds[1] = ledColour;
            glassLeds[2] = ledColour;
            glassLeds[3] = ledColour;
            break;
        case 1:
            glassLeds[4] = ledColour;
            glassLeds[5] = ledColour;
            glassLeds[6] = ledColour;
            glassLeds[7] = ledColour;
            break;
        case 2:
            glassLeds[8] = ledColour;
            glassLeds[9] = ledColour;
            glassLeds[10] = ledColour;
            glassLeds[11] = ledColour;
            break;
        case 3:
            glassLeds[12] = ledColour;
            glassLeds[13] = ledColour;
            glassLeds[14] = ledColour;
            glassLeds[15] = ledColour;
            break;
    }

    // Show the LEDs
    FastLED.show();
    // Increment the step, reset if necessary
    currentStep++;
    if (currentStep > 3) {
      if(currentEye == RIGHT_EYE)
        currentEye = LEFT_EYE;
      else
        currentEye = RIGHT_EYE;
        currentStep = 0; // Reset sequence
    }
}

void sequenceCallback()
{
    DEBUG_PRINT_VERBOSE(F("SEQUENCE_ACTION"));
    if(active_program == PROGRAM_FOUR)
    {
      programFour();
      sequenceTask.setInterval(normalizedBaseFrequency);
    }
    else if(active_program == PROGRAM_THREE)
    {
      programThree();
      sequenceTask.setInterval(normalizedBaseFrequency);
    }
}





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
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_3, items[i].y_content_3);
      u8g2.print(items[i].content_3);
      draw_line(14);
    }
  } while (u8g2.nextPage());
}

void updateDisplay()
{
    switch(active_program) {
        case 0:
            strcpy(gContent2,"Idle");
            break;
        case 1: 
            strcpy(gContent2,"One");
            break;
        case 2: 
            strcpy(gContent2,"Two");
            break;
        case 3: 
            strcpy(gContent2,"Three");
            break;
        case 4: 
            strcpy(gContent2,"Four");
            break;
        default:
            strcpy(gContent2,"Error");
            break; 

    }
    printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
}

// Function to flash an LED for a given duration
void flashLed(int ledPin, unsigned long duration) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
        digitalWrite(ledPin, HIGH);
        delay(200);
        digitalWrite(ledPin, LOW);
        delay(200);
    }
}

void updateFrequencies(int firstInterval, int secondInterval)
{
    char firstFreqBuffer[10] = ""; // Buffer for the first frequency
    char secondFreqBuffer[10] = ""; // Buffer for the second frequency
    float firstFrequency = 1.0/firstInterval;
    dtostrf(firstFrequency, 0, 2, firstFreqBuffer); // (value, width, precision, buffer)
    if(secondInterval > 0)
    {
      float secondFrequency = 1.0/secondInterval;
      dtostrf(secondFrequency, 0, 2, secondFreqBuffer); // (value, width, precision, buffer)
    }
    sprintf(gContent3, "%sHz %sHz", firstFreqBuffer, secondFreqBuffer);
    DEBUG_PRINT_VERY_VERBOSE(gContent3);
    updateDisplay();
}



void setIntervals(int leftInterval, int intermediateInterval, int sequenceInterval)
{
    DEBUG_PRINTLN(F("Setting Intervals: "));
    DEBUG_PRINT_VERY_VERBOSE(leftInterval);
    DEBUG_PRINT_VERY_VERBOSE(F(" "));
    DEBUG_PRINT_VERY_VERBOSE(intermediateInterval);
    DEBUG_PRINT_VERY_VERBOSE(F(" "));
    DEBUG_PRINT_VERY_VERBOSE(sequenceInterval);

    runner.disableAll();
    if(active_program != PROGRAM_IDLE)
    {
  
      if(leftInterval > 0)
      {
        // leftTimer = mainTimer->setInterval(LEFT_EYE_ACTION, leftInterval);
        leftEyeTask.enable();
        DEBUG_PRINT_VERBOSE(F("Setting LEFT_EYE_ACTION Timer: "));
        DEBUG_PRINT_VERBOSE(leftInterval);
        leftEyeTask.setInterval(leftInterval);
        updateFrequencies(leftInterval,0);
      }

      if(intermediateInterval > 0)
      {
        rightEyeTask.enable();
        int rightInterval = leftInterval + intermediateInterval;
        // rightTimer = mainTimer->setInterval(RIGHT_EYE_ACTION, leftInterval + intermediateInterval); // was 500
        DEBUG_PRINT_VERBOSE(F("Setting RIGHT_EYE_ACTION Timer: "));
        DEBUG_PRINT_VERBOSE(rightInterval);
        updateFrequencies(leftInterval,rightInterval);
        rightEyeTask.setInterval(rightInterval);
        updateFrequencies(leftInterval,rightInterval);
      }

      if(sequenceInterval > 0)
      {
        sequenceTask.enable();
        DEBUG_PRINT_VERBOSE(F("Setting SEQUENCE_ACTION Timer: "));
        DEBUG_PRINT_VERBOSE(sequenceInterval);
        sequenceTask.setInterval(sequenceInterval);
        updateFrequencies(sequenceInterval,0);
      }
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
}

void turnOff(int eye)
{
  int i = 0;
  CRGB * glassLeds;
  if(eye == RIGHT_EYE )
  {
    glassLeds = ledsRight;
  }
  else
  {
    glassLeds = ledsLeft;
  }
  for(i =0;i < NUM_LEDS;i++)
  {
    glassLeds[i] = CRGB::Black;
    FastLED.show();
  }

}

void selectProgram()
{
    strcpy(gContent1,"Program: ");
    active_program += 1;
    if(active_program == 5)
      active_program = PROGRAM_IDLE;
    DEBUG_PRINT_VERY_VERBOSE(F("Active Program: "));
    DEBUG_PRINT_VERY_VERBOSE(active_program);
    if(active_program == PROGRAM_FOUR)
    {
      DEBUG_PRINTLN(F("Program four selected."));
      currentStep = 0;
      currentEye = LEFT_EYE;
      setIntervals(0, 0, 100);
    }
    else if(active_program == PROGRAM_TWO)
    {
      leftEyeState = OFF;
      rightEyeState = OFF;
      DEBUG_PRINTLN(F("Program two selected."));
      setIntervals(10, 10,0);
    }
    else if(active_program == PROGRAM_THREE)
    {
      DEBUG_PRINTLN(F("Program three selected."));
      setIntervals(0,0,500);
      currentStep = 0;
      currentEye = LEFT_EYE;
    }
    else if(active_program == PROGRAM_ONE)
    {
      leftEyeState = OFF;
      rightEyeState = OFF;
      DEBUG_PRINTLN(F("Program one selected."));
      setIntervals(10, 10,0);
    }
    else if(active_program == PROGRAM_IDLE)
    {
      DEBUG_PRINT(F("Idle Program: ."));
      turnOff(LEFT_EYE);
      turnOff(RIGHT_EYE);
      runner.disableAll();
    }
    else
    {
      DEBUG_PRINT_ERROR(F("No Active Program: ."));
    }
  updateDisplay();

}


void shortClickButtonSW2() //yellow
{
    DEBUG_PRINT_INFO(F("shortClickButtonSW2"));
    selectProgram();
    flashLed(LED_YELLOW, 1000); 

}


void handleTapSW2(Button2& b) {
    DEBUG_PRINT_INFO(F("handleTapSW2"));
    // check for really long clicks
    unsigned int duration = b.wasPressedFor();
    if (duration > LONG_TAP) {
    DEBUG_PRINT(F("SW2: long tap detected: "));
    }
    else
    {
      DEBUG_PRINT(F("SW2: short tap detected: "));
      DEBUG_PRINTLN(duration);
      shortClickButtonSW2();
    }
   }

void readAnallogValues()
{
  int baseFrequencyrSelection = analogRead(POT_PIN_1);
  int potColourSelection = analogRead(POT_PIN_2);

  // Normalize values to 0–100 range
  normalizedBaseFrequency = (int)map(baseFrequencyrSelection, 0, 1023, 10, 1000);
  int normalizedColourSelection = (int)map(potColourSelection, 0, 1017, 0, 11);

  // Ensure values stay within bounds
  normalizedBaseFrequency = constrain(normalizedBaseFrequency, 10, 1000);
  // normalizedColourSelection = constrain(normalizedColourSelection, 0, 11);
  ledColour = colors[normalizedColourSelection];
  DEBUG_PRINT(F("Base Frequency: "));
  DEBUG_PRINTLN(normalizedBaseFrequency);

  // DEBUG_PRINT(normalizedColourSelection);
  // DEBUG_PRINTLN(potColourSelection);

}


void setup() {
    // Set up LEDs
    pinMode(LED2_PIN, OUTPUT);
    pinMode(SW2_PIN, INPUT);
    pinMode(RIGHT_WS2818, OUTPUT);
    pinMode(LEFT_WS2818, OUTPUT);
    FastLED.addLeds<WS2812, RIGHT_WS2818, GRB>(ledsRight, NUM_LEDS);
    FastLED.addLeds<WS2812, LEFT_WS2818, GRB>(ledsLeft, NUM_LEDS);
    ledColour = CRGB::White;
    Serial.begin(9600);
    delay(200);
    DEBUG_PRINT_INFO(F("Initialised nano .."));
    runner.init();
    runner.addTask(leftEyeTask);
    runner.addTask(rightEyeTask);
    runner.addTask(sequenceTask);
    DEBUG_PRINT_INFO(F("Setting up OLED Display"));
    u8g2_prepare();
    u8g2.begin();
    DEBUG_PRINT_INFO(F("Updating Display"));
    updateDisplay();
        DEBUG_PRINT_INFO(F("Initialised Buttons"));
    button2.begin(SW2_PIN);
    button2.setTapHandler(handleTapSW2);
}

void loop()
{
      button2.loop();
      runner.execute();
      readAnallogValues();
}