# Overview

This library provides an Arduino-compatible NEC IR remote decoder for HX1838 and similar IR receivers. It captures and processes infrared signals, extracts 32-bit NEC codes, and provides a simple interface for retrieval.

## How It Works
--------
### Interrupt-Driven Signal Capture

An interrupt is attached to the specified IR receiver pin.
Each pulse transition (HIGH/LOW) is recorded in pulseTimes[] along with its duration (in microseconds).

### Signal Processing & Noise Filtering

After receiving a signal, we pause briefly (100ms) to ensure full capture.
If the signal is too short (<4 transitions), it is discarded as noise.
NEC repeat signals (9ms LOW + 2.25ms HIGH) are detected and ignored.
Valid NEC signals (9ms LOW + 4.5ms HIGH) proceed to decoding.

### Decoding NEC Signals

The length of each pulse determines whether it is a 1 or 0.
The 32-bit NEC code is reconstructed from the captured pulse train.
Since the NEC protocol transmits LSB (Least Significant Bit) first, we reverse the bit order for correct decoding.

### Retrieving the Decoded Data

Use available() to check if a valid signal was received.
Use getDecodedData() to retrieve the decoded 32-bit NEC command in hexadecimal format.

## Usage Example

```cpp
#include "HX1838Decoder.h"

IRDecoder irDecoder(4);  // Create an instance with IR receiver pin 4

void setup() {
    Serial.begin(115200);
    irDecoder.begin();
}

void loop() {
    if (irDecoder.available()) {
        Serial.print("Decoded NEC Data: 0x");
        Serial.println(irDecoder.getDecodedData(), HEX);
    }
}
```

## Key Features

- Interrupt-based decoding (efficient and accurate)
- Filters out noise and NEC repeat signals
- Reconstructs 32-bit NEC codes with correct bit order
- Simple API for integration into Arduino projects


__Important__: The IR receiver must be connected to an interrupt-capable pin.
Check your microcontroller’s documentation for interrupt-capable pins.

- Arduino Uno/Nano: Pins 2, 3
- ESP8266: Pins 0, 2, 4, 5, 12, 13, 14, 15
- ESP32: Almost all GPIO pins (except 6-11)