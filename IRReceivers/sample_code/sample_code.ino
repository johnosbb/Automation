#define IR_RECEIVER_PIN 4  
#define BUFFER_SIZE 100  

volatile unsigned long pulseTimes[BUFFER_SIZE];  
volatile int pulseIndex = 0;  
volatile unsigned long lastTime = 0;  
volatile uint32_t lastDecodedData = 0;  // Store last valid decoded command

void IRAM_ATTR handleIRSignal() {
  unsigned long currentTime = micros();
  unsigned long pulseDuration = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseDuration < 100000 && pulseIndex < BUFFER_SIZE) {  // Ignore noise
    pulseTimes[pulseIndex++] = pulseDuration;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_RECEIVER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), handleIRSignal, CHANGE);
}

void loop() {
  if (pulseIndex > 0) {
    delay(100);  // Ensure full signal capture before processing

    noInterrupts();
    int count = pulseIndex;
    pulseIndex = 0;
    interrupts();

    Serial.println("\nCaptured Full IR Signal:");

    if (count < 4) {  // Too short, ignore
      Serial.println("Signal too short, possible noise.");
      return;
    }

    // Check for NEC Repeat Signal (9ms LOW + 2.25ms HIGH)
    if (count >= 4 && pulseTimes[0] > 8500 && pulseTimes[0] < 9500 &&
        pulseTimes[1] > 2000 && pulseTimes[1] < 2500 &&
        pulseTimes[2] > 300 && pulseTimes[2] < 700) {
      Serial.println("NEC Repeat Detected! Using last command...");
      Serial.print("Repeated NEC Data: 0x");
      Serial.println(lastDecodedData, HEX);
      return;
    }

    // Check NEC Header: 9ms LOW + 4.5ms HIGH
    if (!(pulseTimes[0] > 8500 && pulseTimes[0] < 9500 &&
          pulseTimes[1] > 4000 && pulseTimes[1] < 5000)) {
      Serial.println("Not a valid NEC signal.");
      return;
    }

    Serial.println("Valid NEC signal detected!");

    // Decode NEC signal
    uint32_t rawData = 0;
    String binaryData = "";
    
    int index = 2;  // Start after header

    for (int i = 0; i < 32; i++) {
      if (index + 1 >= count) break;

      unsigned long mark = pulseTimes[index++];  // 560µs LOW
      unsigned long space = pulseTimes[index++]; // Variable HIGH

      if (space > 1200) {  // ~1690µs → bit 1
        rawData |= (1UL << i);
        binaryData += "1";
      } else {  // ~560µs → bit 0
        binaryData += "0";
      }
    }

    Serial.print("Decoded Binary: ");
    Serial.println(binaryData);

    // Reverse the entire 32-bit rawData to fix bit order
    uint32_t decodedData = reverseBits32(rawData);
    lastDecodedData = decodedData;  // Store last valid key press

    Serial.print("Decoded NEC Data: 0x");
    Serial.println(decodedData, HEX);
  }
}

// Function to reverse all 32 bits
uint32_t reverseBits32(uint32_t num) {
  num = (num & 0xFFFF0000) >> 16 | (num & 0x0000FFFF) << 16;
  num = (num & 0xFF00FF00) >> 8  | (num & 0x00FF00FF) << 8;
  num = (num & 0xF0F0F0F0) >> 4  | (num & 0x0F0F0F0F) << 4;
  num = (num & 0xCCCCCCCC) >> 2  | (num & 0x33333333) << 2;
  num = (num & 0xAAAAAAAA) >> 1  | (num & 0x55555555) << 1;
  return num;
}
