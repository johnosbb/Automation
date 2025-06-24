#include <Wire.h>
#include "Adafruit_MCP23X17.h"

// Define the interrupt pin on your microcontroller
#define INT_PIN 32

// Create an instance of the MCP23X17 expander
Adafruit_MCP23X17 mcp;

// Flag to indicate if an interrupt has occurred
volatile bool interruptOccurred = false;

// Variables for timed printing
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 30000; // 30 seconds in milliseconds

// Interrupt Service Routine (ISR)
void IRAM_ATTR handleInterrupt() {
  interruptOccurred = true;
}

// Function to set up the MCP23017
void setupMCP() {
  Serial.println("MCP23xxx Configuration!");
  // Initialize I2C communication with the MCP23017
  // Use 0x20 for the default I2C address if you haven't changed it
  if (!mcp.begin_I2C(0x20)) {
    Serial.println("Error. Could not find MCP23017. Check wiring and I2C address!");
    while (1); // Halt if MCP23017 is not found
  }
  Serial.println("MCP23017 Found!");

  // Configure the interrupt pin on the microcontroller
  pinMode(INT_PIN, INPUT_PULLUP); // Important for interrupt triggering

  // Mirror INTA and INTB, use active-low, open-drain
  // This means if any interrupt-enabled pin changes state, the INT_PIN will go LOW
  mcp.setupInterrupts(true, false, LOW);

  // Configure GPA pins (0-7) as inputs with pull-ups and enable interrupt on CHANGE
  // This means an interrupt will be triggered when a pin changes from HIGH to LOW or LOW to HIGH
  for (uint8_t i = 0; i <= 7; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    mcp.setupInterruptPin(i, CHANGE); // interrupt when input state changes
  }

  // Configure GPB pins (8-15) as outputs
  for (uint8_t i = 8; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW); // Initialize outputs to LOW
  }

  // Attach interrupt to the ESP32 pin
  // The handleInterrupt function will be called when INT_PIN goes LOW (FALLING edge)
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);

  Serial.println("MCP23017 setup complete. GPA pins are inputs with interrupts, GPB pins are outputs.");
  Serial.println("Connect inputs to GPA0-7. GPB0-7 will mirror their state.");
}

// Function to process MCP23017 interrupts
void process_mpc_inta() {
  // Check if an interrupt has occurred and the INT_PIN is actually LOW
  // digitalRead is generally not recommended in ISRs, but checking it here
  // after the ISR sets the flag is safe in the loop.
  if (interruptOccurred) {
    // Clear the interrupt flag immediately to be ready for the next interrupt
    interruptOccurred = false;

    // Get the pin that triggered the interrupt
    uint8_t interruptPin = mcp.getLastInterruptPin();

    // Get the state of all pins at the time of the interrupt
    uint16_t capturedInterruptState = mcp.getCapturedInterrupt();

    Serial.print("\nInterrupt detected on pin: ");
    Serial.println(interruptPin);
    Serial.print("Pin states at time of interrupt (0b_GPB_GPA): 0b");
    Serial.println(capturedInterruptState, BIN);

    // Clear the interrupts on the MCP23017
    mcp.clearInterrupts();
    updateOutputs();

  }
}


void updateOutputs()
{
    // Now, read the current state of all GPA pins and set the corresponding GPB pins
    for (uint8_t i = 0; i <= 7; i++) {
      uint8_t gpa_pin_state = mcp.digitalRead(i); // Read GPA pin state
      mcp.digitalWrite(i + 8, gpa_pin_state);     // Write to corresponding GPB pin (GPA0 -> GPB0, GPA1 -> GPB1, etc.)
    }

    Serial.println("GPB pins updated to reflect GPA input states.");
}


// Function to print MCP pin status
void printMcpStatus() {
  Serial.println("\n--- MCP23017 Pin Status ---");
  // Read all input pins (GPA0-7)
  uint8_t gpa_values = 0;
  for (uint8_t i = 0; i <= 7; i++) {
    if (mcp.digitalRead(i)) {
      gpa_values |= (1 << i); // Set the bit if pin is HIGH
    }
  }
  Serial.print("GPA (Inputs) Status (0b_bit7...bit0): 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print((gpa_values >> i) & 1);
  }
  Serial.println();

  // Read all output pins (GPB0-7)
  uint8_t gpb_values = 0;
  for (uint8_t i = 8; i <= 15; i++) {
    if (mcp.digitalRead(i)) { // digitalRead works for output pins too
      gpb_values |= (1 << (i - 8)); // Set the bit if pin is HIGH
    }
  }
  Serial.print("GPB (Outputs) Status (0b_bit7...bit0): 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print((gpb_values >> i) & 1);
  }
  Serial.println();
  Serial.println("---------------------------\n");
}


void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial); // Wait for Serial monitor to open
  Serial.print("\nInitialised Serial Port\n ");

  setupMCP(); // Call the MCP23017 setup function
  lastPrintTime = millis(); // Initialize the timer
}

void loop() {
  // Continuously check for MCP23017 interrupts
  process_mpc_inta();

  // Check if it's time to print the status
  if (millis() - lastPrintTime >= printInterval) {
    printMcpStatus();
    updateOutputs();
    lastPrintTime = millis(); // Reset the timer
  }

  // Add other non-blocking tasks here if needed
}