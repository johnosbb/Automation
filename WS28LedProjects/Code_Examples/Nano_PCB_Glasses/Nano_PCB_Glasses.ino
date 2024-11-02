#include <Button2.h>

// Pin Definitions
const int LED1_PIN = 12;
const int LED2_PIN = 11;
const int LED3_PIN = 10;
const int SW1_PIN = 8;
const int SW2_PIN = 7;
const int SW3_PIN = 6;

// Button and LED state
Button2 button1(SW1_PIN);
Button2 button2(SW2_PIN);
Button2 button3(SW3_PIN);

void setup() {
    // Set up LEDs
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);

    // Initialize serial communication for output
    Serial.begin(9600);

    // Attach event handlers for each button
    button1.setClickHandler(handleClick1);
    button1.setDoubleClickHandler(handleDoubleClick1);
    button1.setLongClickHandler(handleLongClick1);

    button2.setClickHandler(handleClick2);
    button2.setDoubleClickHandler(handleDoubleClick2);
    button2.setLongClickHandler(handleLongClick2);

    button3.setClickHandler(handleClick3);
    button3.setDoubleClickHandler(handleDoubleClick3);
    button3.setLongClickHandler(handleLongClick3);
}

void loop() {
    // Continuously check button states
    button1.loop();
    button2.loop();
    button3.loop();
}

// Event handlers for Button1 (SW1)
void handleClick1(Button2& btn) {
    static bool led1State = false;
    led1State = !led1State;
    digitalWrite(LED1_PIN, led1State ? HIGH : LOW);
    Serial.println("SW1: click detected");
}

void handleDoubleClick1(Button2& btn) {
    Serial.println("SW1: double click detected");
    steadyOnLed(LED1_PIN, 3000);  // Turn on LED1 for 3 seconds
}

void handleLongClick1(Button2& btn) {
    Serial.println("SW1: long click detected");
    flashLed(LED1_PIN, 3000);
}

// Event handlers for Button2 (SW2)
void handleClick2(Button2& btn) {
    static bool led2State = false;
    led2State = !led2State;
    digitalWrite(LED2_PIN, led2State ? HIGH : LOW);
    Serial.println("SW2: click detected");
}

void handleDoubleClick2(Button2& btn) {
    Serial.println("SW2: double click detected");
    steadyOnLed(LED2_PIN, 3000);  // Turn on LED2 for 3 seconds
}

void handleLongClick2(Button2& btn) {
    Serial.println("SW2: long click detected");
    flashLed(LED2_PIN, 3000);
}

// Event handlers for Button3 (SW3)
void handleClick3(Button2& btn) {
    static bool led3State = false;
    led3State = !led3State;
    digitalWrite(LED3_PIN, led3State ? HIGH : LOW);
    Serial.println("SW3: click detected");
}

void handleDoubleClick3(Button2& btn) {
    Serial.println("SW3: double click detected");
    steadyOnLed(LED3_PIN, 3000);  // Turn on LED3 for 3 seconds
}

void handleLongClick3(Button2& btn) {
    Serial.println("SW3: long click detected");
    flashLed(LED3_PIN, 3000);
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
