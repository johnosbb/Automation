# SnoopDogNG — ESP32 Robot Firmware

## Overview
- ESP32-based robot car with OLED display, IR remote control, ultrasonic obstacle sensing, rotary encoder RPM, and an MCP23017 IO expander for buttons/LEDs.
- Differential drive motor control via LEDC PWM; manual (IR) and autonomous navigation modes with automatic recovery.
- WiFi client streams JSON telemetry to a TCP log server.
- Secondary UART link uses a framed TLV protocol (COBS + CRC-8) to communicate with a companion board (e.g., LuckFox).

## Hardware & Pinout (ESP32)
- IR receiver (NEC): `GPIO4`
- Rotary encoder input: `GPIO39`
- HC-SR04 ultrasonic: `TRIGGER GPIO16`, `ECHO GPIO36`
- MCP23017 IO expander (I2C): INT to `GPIO32` (pins 0–7 inputs with pull-ups, 8–15 outputs for LEDs)
- OLED (U8g2): I2C (use your board’s SDA/SCL)
- Motor control pins:
  - Left: `DIR1 GPIO26`, `DIR2 GPIO27`, `SPEED (PWM) GPIO14`
  - Right: `DIR1 GPIO19`, `DIR2 GPIO18`, `SPEED (PWM) GPIO23`
- Error LED: `GPIO17`
- UART2 to companion: `TX GPIO13`, `RX GPIO34` (requires external pull-up on RX)

## System Diagram
```
                 +-------------------+          +---------------------+
 IR Remote ----> |  HX1838 `irDecoder`|          |  MCP23017 (I2C)     |
  (NEC)          |  (IR codes)        |          |  SW1/SW2 + LEDs     |
                 +----------+---------+          |  INT -> `INT_PIN`   |
                            |                    +----------+----------+
                            |                                |
                            v                                v
                 +-------------------------+        +-------------------------+
                 | process_navigation_     |        | process_mpc_inta()      |
                 | information(distance)   |        | -> STOP, log, recovery  |
                 |  - Manual: IR -> state  |        +-----------+-------------+
                 |  - Auto: HC-SR04 logic  |                    |
                 +------------+------------+                    |
                              |                                 |
                              +---------------+-----------------+
                                              v
                                        +-----------+
                                        | stateMachine(code,distance,reason)
                                        +-----+-----+
                                              |
                          +-------------------+-------------------+
                          |                                       |
                          v                                       v
      +---------------------------------+          +----------------------------+
      | Motor Control (LEDC PWM)        |          | OLED Display (U8g2)        |
      | moveForward/back/rotate/stop    |          | `printContent` +           |
      | -> pins for L/R dir + speed     |          | `updateProgram/Status`     |
      +----------------------+----------+          +-------------+--------------+
                             |                                     |
                             v                                     v
                       L/R DC Motors                         User Feedback

 Sensors:
   +---------------------------+           +---------------------------+
   | HC-SR04 `processHCSR04()` |           | Rotary Encoder `count()`  |
   | distance (cm)             |           | pulses -> rpm             |
   +-------------+-------------+           +--------------+------------+
                 |                                        |
                 v                                        v
        feeds auto nav                         `processEncoder()` -> RPM

 Telemetry & Control Links:
   +-------------------------------------------+     +------------------------+
   | WiFi TCP `WiFiClient` -> `log_server.py`  |<----| JSON: {time, distance, |
   | `processLogs()` with retries              |     | state, reason, rpm}    |
   +-------------------+-----------------------+     +------------------------+

   +----------------------+    framed UART (COBS + CRC8)     +-----------------+
   | ESP32 UART2          | <------------------------------> | LuckFox (Python)|
   | `sendCmd/pollUartRx` |    0x7E ENC CRC 0x7E            | `luckfox_...py` |
   | TLVs: NAV_*, ACK     |                                  | send_nav/ACK    |
   +----------------------+                                  +-----------------+
```

## Build & Flash (Arduino IDE)
- Install ESP32 board support in Arduino IDE, select your ESP32 board.
- Libraries: U8g2, Adafruit_MCP23X17, NewPing, ArduinoJson, HX1838Decoder (or equivalent IR NEC decoder).
- Create `config.h` with your WiFi/network config used by the sketch (examples below are referenced by the code):
  - `ssid`, `pass` (defaults)
  - `knownNetworks[]` and `numNetworks` (SSID/password list)
  - Static IP: `device_ip`, `gateway_ip`, `subnet_mask`, `dns_ip_1`, `dns_ip_2`
- Open `SnoopDogNG.ino`, build, and upload.

## Runtime Behavior
- Mode toggle: IR `#` or SW2 toggles Auto/Manual.
  - Manual: IR movement keys drive immediately (FORWARD/LEFT/RIGHT/REVERSE/STOP).
  - Auto: HC-SR04 distance thresholds trigger STOP and `performRecovery()` (reverse + alternating turns).
- Emergency STOP: SW1 forces STOP, sets Manual, and lights error LED.
- Telemetry: JSON every ~1s to TCP server (default `serverIP` in sketch, port `5000`).

## Telemetry Server (`log_server.py`)
- Runs a TCP server that tolerates concatenated/partial JSON messages.
- Writes formatted lines to `car_log.txt` and prints them.
- Usage:
  - Ensure the ESP32 can reach the host IP set in `SnoopDogNG.ino` (`serverIP`, default `192.168.1.191`).
  - Run: `python3 log_server.py`

## LuckFox UART Bridge (`luckfox_command_processor.py`)
- Linux-side UART handler for `/dev/ttyS3` at 115200 using the same COBS+CRC TLV protocol.
- On start it sends a NAV LEFT test command, then processes incoming frames.
- Usage (on the companion):
  - Enable the relevant UART (e.g., LuckFox UART7) per board tools.
  - Wire to ESP32 UART2: LuckFox TX -> ESP32 `GPIO34` (via proper level/pull-up as needed), LuckFox RX -> ESP32 `GPIO13`, and common GND.
  - Run: `python3 luckfox_command_processor.py`

## Notes
- Adjust `serverIP`/`serverPort` in `SnoopDogNG.ino` to your logging host.
- MCP23017 default I2C address is used by the Adafruit library; change in code if your wiring differs.
- The included `cobs.*` and `crc8.h` implement the framing used by both ESP32 and the Python bridge.

