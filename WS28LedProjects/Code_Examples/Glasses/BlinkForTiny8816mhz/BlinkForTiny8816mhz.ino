/* Start of Code */

#include "FastLED.h"

// How many leds are connected?
#define NUM_LEDS 16

// Define the Pins
#define DATA_PIN 2
#define COLOR_PIN 7

// Define the array of leds
CRGB leds[NUM_LEDS];

long PreferredColour = CRGB::Blue;
unsigned int PreferredOffDelay = 50;
unsigned int PreferredOnDelay = 200;

void setup() { 
  FastLED.addLeds<NEOPIXEL,DATA_PIN>(leds, NUM_LEDS);
  pinMode(COLOR_PIN, INPUT);
  Serial.begin(9600);
}



void LeftToRightLong(unsigned int offDelay)
{
    // Change led colors
  for(int led = 0; led < NUM_LEDS; led+=2)
  { 

    leds[led] = PreferredColour;
    leds[led+1] = PreferredColour;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(PreferredOnDelay);
    leds[led] = CRGB::Black;
    leds[led+1] = CRGB::Black;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(offDelay);
  }
}


void LeftToRight(unsigned int offDelay)
{
    // Change led colors
  for(int led = 0; led < NUM_LEDS; led++)
  { 

    leds[led] = PreferredColour;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(PreferredOnDelay);
    leds[led] = CRGB::Black;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(offDelay);
  }
}

void RightToLeft(unsigned int offDelay)
{
    // Change led colors
  for(int led = NUM_LEDS; led > -1; led--)
  { 

    leds[led] = PreferredColour;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(PreferredOnDelay);
    leds[led] = CRGB::Black;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(offDelay);
  }
}

void RightToLeftLong(unsigned int offDelay)
{
    // Change led colors
  for(int led = NUM_LEDS; led > -1; led-=2)
  { 

    leds[led] = PreferredColour;
    leds[led-1] = PreferredColour;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(PreferredOnDelay);
    leds[led] = CRGB::Black;
    leds[led-1] = CRGB::Black;
    FastLED.setBrightness(50);
    FastLED.show();
    delay(offDelay);
  }
}

void loop() { 
  unsigned int color = 0;
  unsigned int sensorValue = analogRead(A0);
  //Serial.println(sensorValue);  
  // Clear the existing led values
  FastLED.clear();
  LeftToRightLong(sensorValue);
  color = digitalRead(COLOR_PIN);
  RightToLeftLong(sensorValue);
  color = digitalRead(COLOR_PIN);
  Serial.println(color);
  if(color == 0)
  {
  if(PreferredColour == CRGB::Red )    
    PreferredColour = CRGB::Blue;  
  else
    PreferredColour = CRGB::Red;    
  }    





}



