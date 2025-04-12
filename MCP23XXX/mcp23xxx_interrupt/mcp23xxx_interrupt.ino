// NOTE: This is a simple example that only reads the INTA or INTB pin
//       state. No actual interrupts are used on the host microcontroller.
//       MCP23XXX supports the following interrupt modes:
//           * CHANGE - interrupt occurs if pin changes to opposite state
//           * LOW - interrupt occurs while pin state is LOW
//           * HIGH - interrupt occurs while pin state is HIGH

// ok to include only the one needed
// both included here to make things simple for example
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>

#define BUTTON_PIN 1   // MCP23XXX pin used for interrupt

#define INT_PIN 32      // microcontroller pin attached to INTA/B

// only used for SPI
#define CS_PIN 6

// uncomment appropriate line
//Adafruit_MCP23X08 mcp;
Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(115200);
  Serial.println("MCP23xxx Interrupt Test!");

  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }


  pinMode(INT_PIN, INPUT);

// Register       	Description
// DEFVAL	          Bitmask: expected/default value on pin
// INTCON	          Bitmask: compare to DEFVAL or last pin
// GPINTEN	        Bitmask: enable interrupts on pins

  // Mirror INTA and INTB, use active-low, open-drain
  mcp.setupInterrupts(true, false, LOW);
  // You can use the Adafruit_MCP23X17::write8() method to write directly to any register by address.
  // // Enable interrupts on GPA0–GPA7
  // mcp.write8(0x04, 0xFF);  // GPINTENA – enable interrupt on all GPA pins

  // // Set DEFVALA to 0xFF = expect HIGH
  // mcp.write8(0x06, 0xFF);  // DEFVALA – expected values (HIGH)

  // // Set INTCONA to compare to DEFVALA (not previous state)
  // mcp.write8(0x08, 0xFF);  // INTCONA – compare to DEFVAL instead of last state


  // Configure pins 0 to 7 as inputs with pull-ups and enable interrupt
  for (uint8_t i = 0; i <= 7; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    mcp.setupInterruptPin(i, CHANGE);  // interrupt when pulled LOW
  }

  Serial.println("Looping...");
}


void loop() {
  if (!digitalRead(INT_PIN)) {
    Serial.print("Interrupt detected on pin: ");
    Serial.println(mcp.getLastInterruptPin());
    Serial.print("Pin states at time of interrupt: 0b");
    Serial.println(mcp.getCapturedInterrupt(), 2);
    delay(250);  // debounce
    // NOTE: If using DEFVAL, INT clears only if interrupt
    // condition does not exist.
    // See Fig 1-7 in datasheet.
    mcp.clearInterrupts();  // clear
  }
}