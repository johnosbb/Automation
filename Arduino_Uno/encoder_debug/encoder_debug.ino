/*
  Arduino code for reading encoder pin without using interrupts
  Modified by ChatGPT
  Original Author: Revant Adlakha
  Lab: SVL
*/

int encoder_pin1 = 2; // Pin connected to encoder
unsigned long lastPrintTime = 0; // Track last time we printed

void setup() {
  pinMode(encoder_pin1, INPUT); // Set pin as input
  Serial.begin(9600);           // Start serial communication
}

void loop() {
  // Check if 1 second (1000ms) has passed
  if (millis() - lastPrintTime >= 1000) {
    int pinState = digitalRead(encoder_pin1); // Read the pin
    Serial.print("Pin 2 State: ");
    Serial.println(pinState);                 // Print the pin state (0 or 1)
    lastPrintTime = millis();                 // Update last print time
  }
}
