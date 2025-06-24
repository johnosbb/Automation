#include <Adafruit_MCP23X17.h>

#define BUTTON_PIN 1   // MCP23X17 pin used for interrupt
#define INT_PIN 32     // ESP32 pin attached to INTA/B

Adafruit_MCP23X17 mcp;

volatile bool interruptOccurred = false; // Flag to indicate interrupt

void IRAM_ATTR handleInterrupt() {
  interruptOccurred = true; // Set the interrupt flag
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("MCP23xxx Interrupt Test!");

  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }

  pinMode(INT_PIN, INPUT_PULLUP); // Important for interrupt triggering

  // Mirror INTA and INTB, use active-low, open-drain
  mcp.setupInterrupts(true, false, LOW);

  // Configure pins 0 to 1 as inputs with pull-ups and enable interrupt. Only using two pins now.
  for (uint8_t i = 0; i <= 1; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    mcp.setupInterruptPin(i, CHANGE); // interrupt when pulled LOW
  }

  // Attach interrupt to the ESP32 pin
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING); // Assuming the MCP23X17 pulls INT_PIN LOW

  Serial.println("Looping...");
}





void process_interrupt()
{
  if (interruptOccurred) {
    interruptOccurred = false; // Reset the flag immediately

    delayMicroseconds(10); // Short delay before reading registers

    uint8_t lastInterruptPin = mcp.getLastInterruptPin(); // get the pin

    if (lastInterruptPin != 255) { // ignore interrupts from pin 255

      Serial.print("Interrupt detected on pin: ");
      Serial.println(lastInterruptPin);
      Serial.print("Pin states at time of interrupt: 0b");
      Serial.println(mcp.getCapturedInterrupt(), 2);

      // Debugging: Print captured interrupt values.
      Serial.print("Captured Interrupt: 0x");
      Serial.println(mcp.getCapturedInterrupt(), HEX);

      delay(250); // debounce.

      mcp.clearInterrupts(); // Clear the interrupt

      //Debugging: print captured interrupt after clear.
      Serial.print("Captured Interrupt after clear: 0x");
      Serial.println(mcp.getCapturedInterrupt(), HEX);
    } else {
      Serial.println("Ignored interrupt from pin 255");
      mcp.clearInterrupts(); // clear the interrupt even if ignored.
    }
  }
}


void check_mcp_inputs()
{
    // Read and print PA1 value
  int pa1Value = mcp.digitalRead(1);
  Serial.print("PA1: ");
  Serial.print(pa1Value);

    // Read and print PA0 value
  int pa0Value = mcp.digitalRead(0);
  Serial.print(" PA0: ");
  Serial.print(pa0Value);

  //Read and print all PA0-PA7 values
  //for(int i = 0; i < 8; i++){
  //  Serial.print(" PA");
  //  Serial.print(i);
  //  Serial.print(": ");
  //  Serial.print(mcp.digitalRead(i));
  //}

  Serial.println(); // Newline

  if (interruptOccurred) {
    Serial.println("Interrupt occurred!");
    interruptOccurred = false; // Reset the flag
  }

  delay(100); // Add a small delay to avoid flooding the serial monitor
}

void loop() {
  process_interrupt();
  //check_mcp_inputs();
}