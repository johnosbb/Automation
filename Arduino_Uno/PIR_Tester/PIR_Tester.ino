// Define the PIR sensor pin
const int pirPin = 2; // Digital pin 2

// Variable to store the PIR sensor state
int pirState = LOW;

// Variable to store the previous PIR sensor state
int previousPirState = LOW;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Set the PIR pin as an input
  pinMode(pirPin, INPUT);
}

void loop() {
  // Read the PIR sensor value
  pirState = digitalRead(pirPin);

  // Check if the PIR state has changed
  if (pirState != previousPirState) {
    // If the state has changed, print the new state
    if (pirState == HIGH) {
      Serial.println("Motion detected!");
    } else {
      Serial.println("Motion stopped!");
    }
    // Update the previous state
    previousPirState = pirState;
  }
  delay(50); // Small delay to avoid rapid readings
}