

#include <U8g2lib.h>
#include <debug.h>
#include "WiFi.h"
#include <config.h>
#include <esp_system.h> // For esp_read_mac
#include "HX1838Decoder.h"
#include <NewPing.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <Adafruit_MCP23X17.h>
#include "cobs.h"          // tinycobs or your own
#include "crc8.h" 

uint8_t frame[64];



#define BUTTON_PIN 1   // MCP23XXX pin used for interrupt



#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define HARDWARE_SERIAL
#ifdef HARDWARE_SERIAL
// Uart
HardwareSerial SerialExt(2);             // use hardware UART #2
constexpr uint8_t UART_RX_PIN = 34;      // RX pin with external pull up
constexpr uint8_t UART_TX_PIN = 13;      // TX
#endif

// LEDs
// These are references to MCP23017
#define STATUS_LED 4 // GPB pin 4 is connected to a blue status Led
#define MODE_LED 3   // GPB pin 3 is connected to a yellow mode Led
#define SW1 2  // GPA2
#define SW2 3  // GPA3


#define INT_PIN 32      // microcontroller pin attached to INTA/B
// Motor Control

// Left Motor Rear
#define ENABLE_MOTOR_CONTROL
#define MOTOR_LEFT_DIRECTION_1  26
#define MOTOR_LEFT_DIRECTION_2  27
#define MOTOR_LEFT_SPEED  14
#define MOTOR_RIGHT_DIRECTION_1  19
#define MOTOR_RIGHT_DIRECTION_2  18
#define MOTOR_RIGHT_SPEED  23


// Infrared Receiver
#define IR_RECEIVE_PIN 4 // GPIO4
// HC-020K  Encoder Pin Output
#define ENCODER_PIN 39 // GPIO39

// Setting PWM properties
#define FREQUENCY 30000
#define PWM_CHANNEL_LEFT    0  // PWM channel for left motor
#define PWM_CHANNEL_RIGHT   1  // PWM channel for right motor
#define RESOLUTION  8
#define DEFAULT_DUTY_CYCLE 200
#define REDUCED_SPEED  200 // Speed for the left motors (reduced speed)
#define HIGHER_SPEED  255  // Speed for the right motors (higher speed)

// HC‐SR04 Ultra Sonic Sensor
#define ENABLE_HC_SR04
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
#define HC_SR04_ECHO_PIN 36 // GPIO36
#define HC_SR04_TRIGGER_PIN 16 // GPIO16

#ifdef ENABLE_HC_SR04
NewPing sonar(HC_SR04_TRIGGER_PIN, HC_SR04_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
#endif

#define ERROR_LED_PIN 17 // GPIO17

// Define the structure for the content
typedef struct {
  const char *header;
  const char *content_1;
  const char *content_2;
  const char *content_3;
  int x_header;
  int y_header;
  int x_content_1;
  int y_content_1;
  int x_content_2;
  int y_content_2;
  int x_content_3;
  int y_content_3;
} DisplayItem;

char gHeader[25] = "Status";
char gContent1[25] = "Program: ";
char gContent2[25] = "No Program";
char gContent3[25] = "";

IRDecoder irDecoder(IR_RECEIVE_PIN);  // IR Receiver on IR_RECEIVE_PIN Remote Control

#define MCP23017
#ifdef MCP23017
Adafruit_MCP23X17 mcp;
#endif

// Define IR codes
const unsigned long KEY_0 = 0xFF4AB5; 
const unsigned long KEY_1 = 0xFF6897;
const unsigned long KEY_2 = 0xFF9867;
const unsigned long KEY_3 = 0xFFB04F;
const unsigned long KEY_4 = 0xFF30CF;
const unsigned long KEY_5 = 0xFF18E7;
const unsigned long KEY_6 = 0xFF7A85;
const unsigned long KEY_7 = 0xFF10EF;
const unsigned long FORWARD = 0xFF629D;
const unsigned long LEFT = 0xFF22DD;
const unsigned long RIGHT = 0xFFC23D;
const unsigned long REVERSE = 0xFFA857;
const unsigned long STOP = 0xFF02FD; // OK key on Lafvin keyboard
const unsigned long ASTERISK = 0xFF42BD; // 
const unsigned long HASH = 0xFF52AD; // 

// Add a global variable to help with debouncing SW2 (or any button)
bool lastSw2State = false;

WiFiClient client;
const char* serverIP = "192.168.1.191"; // Replace with your server's IP address
const uint16_t serverPort = 5000;       // Port for communication
unsigned long lastLogTime = 0;
const unsigned long logInterval = 1000; // Log every second


// Variables to track stuck state
unsigned long stopStartTime = 0;  // Time when STOP state starts
int stopCount = 0;                // Counts consecutive stops
const unsigned long stuckThreshold = 3000;  // 3 seconds threshold to detect stuck
const int maxStopAttempts = 3;  // Maximum stop occurrences before drastic action
int recoveryAttempts = 0;  // Keeps track of recovery attempts
unsigned int enableNavigation = 1;

// Rotary Encoder
unsigned int rpm;
volatile byte pulses;
unsigned long encoder_previous_time;
unsigned int pulses_per_turn= 20; // Depends on the number of spokes on the encoder wheel

// State enumeration
enum State {
  STATE_IDLE,
  STATE_FORWARD,
  STATE_REVERSE,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_STOP
};


// Reasoninging enumeration
enum Reasoning {
  IR_CONTROL,
  CLEAR_PATH,
  OUT_OF_RANGE,
  BLOCKED,
  RECOVERING,
  UNCERTAIN,
  CRASH_DETECT
};


#define OBSTACLE_STOP_THRESHOLD 30
#define OBSTACLE_SLOW_THRESHOLD 40

volatile bool interruptOccurred = false; // Flag to indicate interrupt

void IRAM_ATTR handleInterrupt() {
  interruptOccurred = true; // Set the interrupt flag
}


// Current state
State currentState = STATE_IDLE;


DisplayItem displayItems[] = {
{
  gHeader, gContent1, gContent2, gContent3, 0,0,0,20,80,20,0,40}
};

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define ERROR 1
#define RECONNECTING 3
#define RECEIVING 4
#define CONNECTED 5
#define NORMAL 0
#define MANUAL 6

#define MOVING_FORWARD 0
#define MOVING_REVERSE 1
#define STOPPING 2

#define TEST_DELAY 4000

unsigned int running_state = NORMAL;
unsigned int motor_status = STOPPING;
uint32_t chipId = 0;
unsigned int driving_speed = DEFAULT_DUTY_CYCLE;


void count() // Counting the number of pulses for calculation of rpm
{
  pulses++;  
}




/* ─── forward declaration ─── */
void serialDebug(const uint8_t *enc, size_t len, uint8_t crc);

/* ─── send one TLV as a framed packet ─── */
void sendCmd(uint8_t type, const uint8_t *val, uint8_t len)
{
    uint8_t p[32];
    p[0] = type;
    p[1] = len;
    memcpy(&p[2], val, len);

    uint8_t enc[40];
    size_t  encLen = cobs_encode(p, 2 + len, enc);

    uint8_t crc8   = crc8_buf(enc, encLen);

    serialDebug(enc, encLen, crc8);        // <-- print the frame in hex

    SerialExt.write(0x7E);
    SerialExt.write(enc, encLen);
    SerialExt.write(crc8);
    SerialExt.write(0x7E);
}

/* ─── helper to print the frame in USB console ─── */
void serialDebug(const uint8_t *enc, size_t len, uint8_t crc)
{
  Serial.print(F("TX : 7E"));
  for (size_t i = 0; i < len; ++i) {
    Serial.printf(" %02X", enc[i]);
  }
  Serial.printf(" %02X 7E\n", crc);
}

/* 1.  Ask the Ultra to TAKE A SINGLE PHOTO
 *    TLV = [0x01, 0x00]  (type=0x01, length=0)  */
void triggerStill()
{
    Serial.println("Requesting Still image");
    sendCmd(0x01,            // type  (CMD_TAKE_PHOTO)
            nullptr,         // no value bytes
            0);              // length = 0
}

/**
 * @brief Turns on a selected LED connected to the MCP23017.
 *
 * @param ledPin The GPB pin number (0-15) corresponding to the LED. GPB0 is pin 0
 * Use defined constants like STATUS_LED (blue) or MODE_LED (yellow).
 */
void turnOnLed(uint8_t ledPin) {
  ledPin = ledPin + 8; // Convert GPB pin (0-7) to MCP pin (8-15)
  // Turn on the LED (assuming active-high for the LED)
  mcp.digitalWrite(ledPin, HIGH);
  switch (ledPin) {
    case STATUS_LED:
      DEBUG_PRINTLN(F("Blue Status LED ON"));
      break;
    case MODE_LED:
      DEBUG_PRINTLN(F("Yellow Mode LED ON"));
      break;
    default:
      DEBUG_PRINT(F("LED on pin "));
      DEBUG_PRINT(ledPin);
      DEBUG_PRINTLN(F(" ON"));
      break;
  }
}

/**
 * @brief Turns off a selected LED connected to the MCP23017.
 *
 * @param ledPin The GPB pin number (0-15) corresponding to the LED.
 * Use defined constants like STATUS_LED (blue) or MODE_LED (yellow).
 */
void turnOffLed(uint8_t ledPin) {
  ledPin = ledPin + 8; // Convert GPB pin (0-7) to MCP pin (8-15)
  // Turn off the LED (assuming active-high for the LED)
  mcp.digitalWrite(ledPin, LOW);
  switch (ledPin) {
    case STATUS_LED:
      DEBUG_PRINTLN(F("Blue Status LED OFF"));
      break;
    case MODE_LED:
      DEBUG_PRINTLN(F("Yellow Mode LED OFF"));
      break;
    default:
      DEBUG_PRINT(F("LED on pin "));
      DEBUG_PRINT(ledPin);
      DEBUG_PRINTLN(F(" OFF"));
      break;
  }
}

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void draw_line(unsigned int a)
{
  u8g2.drawLine(0, a, 127, a);
}

// Function to print an array of DisplayItem
void printContent(const DisplayItem items[], int itemCount) {

  u8g2.firstPage();
  do {
    // Iterate through each item in the array
    for (int i = 0; i < itemCount; i++) {
      // Set cursor and print the header
      u8g2.setCursor(items[i].x_header, items[i].y_header);
      u8g2.print(items[i].header);
      // Set cursor and print the first content
      u8g2.setCursor(items[i].x_content_1, items[i].y_content_1);
      u8g2.print(items[i].content_1);
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_2, items[i].y_content_2);
      u8g2.print(items[i].content_2);
      // Set cursor and print the second content
      u8g2.setCursor(items[i].x_content_3, items[i].y_content_3);
      u8g2.print(items[i].content_3);
      draw_line(14);
    }
  } while (u8g2.nextPage());
}

void updateProgram() 
{
    switch(running_state) {
        case 0:
            strcpy(gContent2,"Normal");
            break;
        case 1: 
            strcpy(gContent2,"Error");
            break;
        case 2: 
            strcpy(gContent2,"Recovering");
            break;
        case 3: 
            strcpy(gContent2,"Reconnecting");
            break;
        case 4: 
            strcpy(gContent2,"Receiving");
            break;
        case 5: 
            strcpy(gContent2,"Connected");
            break;
        case 6: 
            strcpy(gContent2,"Manual");
            break;
        default:
            strcpy(gContent2,"Error");
            break; 
    }
    printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
}


void updateMotorStatus()
{
    switch(motor_status) {
        case 0:
            strcpy(gContent3,"Moving Forward");
            break;
        case 1: 
            strcpy(gContent3,"Moving Backward");
            break;
        case 2: 
            strcpy(gContent3,"Stopping");
            break;
        case 3: 
            strcpy(gContent3,"Three");
            break;
        case 4: 
            strcpy(gContent3,"Four");
            break;
        default:
            strcpy(gContent3,"Error");
            break; 
    }
    printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));
}


#ifdef MCP23017

// | Library pin # | Port bit | Package pin (DIP/SSOP/QFN) | Silkscreen label |
// |--------------:|----------|----------------------------|------------------|
// | 0  | GPA0 | 21 | A0 |
// | 1  | GPA1 | 22 | A1 |
// | 2  | GPA2 | 23 | A2 |
// | 3  | GPA3 | 24 | A3 |
// | 4  | GPA4 | 25 | A4 |
// | 5  | GPA5 | 26 | A5 |
// | 6  | GPA6 | 27 | A6 |
// | 7  | GPA7 | 28 | A7 |
// | 8  | GPB0 | 1  | B0 |
// | 9  | GPB1 | 2  | B1 |
// | 10 | GPB2 | 3  | B2 |
// | 11 | GPB3 | 4  | B3 |
// | 12 | GPB4 | 5  | B4 |
// | 13 | GPB5 | 6  | B5 |
// | 14 | GPB6 | 7  | B6 |
// | 15 | GPB7 | 8  | B7 |
void setupMCP() {
  Serial.println("MCP23xxx Configuration!");
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }
  pinMode(INT_PIN, INPUT_PULLUP); // Important for interrupt triggering
  // Mirror INTA and INTB, use active-low, open-drain
  mcp.setupInterrupts(true, false, LOW);
  // Configure pins 0 to 1 as inputs with pull-ups and enable interrupt. Only using two pins now.
  for (uint8_t i = 0; i <= 7; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    mcp.setupInterruptPin(i, CHANGE); // interrupt when pulled LOW
  }
  // Configure GPB pins (8-15) as outputs
  for (uint8_t i = 8; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW); // Initialize outputs to LOW
  }
  // Attach interrupt to the ESP32 pin
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING); // Assuming the MCP23X17 pulls INT_PIN LOW
}
#endif

void setupRotaryEncoder() {
  rpm=0;
  pulses=0;
  encoder_previous_time=0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0
}

void setupWifi()
{
  WiFi.setMinSecurity(WIFI_AUTH_WEP); // Lower min security to WEP.
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  // Serial.print("WIFI status = ");
  // Serial.println(WiFi.getMode());
  // Get MAC Address for Wi-Fi Station Interface
  String macAddress = WiFi.macAddress();
  Serial.print("Wi-Fi MAC Address: ");
  Serial.println(macAddress);
}

#ifdef ENABLE_HC_SR04
int processHCSR04()
{
  delay(50);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  DEBUG_PRINT(F("Distance: "));
  DEBUG_PRINT(distance);
  DEBUG_PRINTLN(F("cm"));
  return distance;
}
#endif

void scanWifiNetworks()
{
    DEBUG_PRINT_INFO(F("scan start"));
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    DEBUG_PRINT_INFO(F("scan done"));
    if (n == 0) {
        DEBUG_PRINT_INFO(F("no networks found"));
    } else {
        DEBUG_PRINT_INFO(String(n).c_str() + String(F(" networks found")));
        for (int i = 0; i < n; ++i) {
            // Create the string with SSID and RSSI for each network found
            String networkInfo = String(i + 1) + String(F(": ")) + WiFi.SSID(i) + String(F(" (")) + String(WiFi.RSSI(i)) + String(F(")"));

            // Append encryption type information
            if (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) {
                networkInfo += F(" *");
            }

            DEBUG_PRINT_INFO(networkInfo.c_str());
            delay(10);
        }
    }
    DEBUG_PRINT_INFO(F(""));
    // Wait a bit before scanning again
    delay(5000);
}





#ifdef ENABLE_MOTOR_CONTROL
void setupMotorControl()
{
  pinMode(MOTOR_LEFT_DIRECTION_1, OUTPUT);
  pinMode(MOTOR_LEFT_DIRECTION_2, OUTPUT);
  pinMode(MOTOR_LEFT_SPEED, OUTPUT);
  pinMode(MOTOR_RIGHT_DIRECTION_1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIRECTION_2, OUTPUT);
  pinMode(MOTOR_RIGHT_SPEED, OUTPUT);
    // configure LEDC PWM for the left motor
  ledcAttachChannel(MOTOR_LEFT_SPEED, FREQUENCY, RESOLUTION, PWM_CHANNEL_LEFT);
  ledcAttachChannel(MOTOR_RIGHT_SPEED, FREQUENCY, RESOLUTION, PWM_CHANNEL_RIGHT);
}


// Function to move forward
void moveForward(int directionPin1, int directionPin2, int speedPin, int duty_cycle) {
    DEBUG_PRINTLN(F("Motor Moving Forward"));
    motor_status = MOVING_FORWARD;
    updateMotorStatus();
    digitalWrite(directionPin1, LOW);
    digitalWrite(directionPin2, HIGH);
    ledcWrite(speedPin, duty_cycle); // Set the speed using duty cycle (0-255)
}

// Function to move backward
void moveBackward(int directionPin1, int directionPin2, int speedPin, int duty_cycle) {
    // Move DC motor backwards at maximum speed
    DEBUG_PRINTLN(F("Motor Moving Backwards"));
    motor_status = MOVING_REVERSE;
    updateMotorStatus();
    digitalWrite(directionPin1, HIGH);
    digitalWrite(directionPin2, LOW);
    ledcWrite(speedPin, duty_cycle); // Set the speed using duty cycle (0-255)
}

// Function to stop the motor
void stopMotor(int directionPin1, int directionPin2, int speedPin) {
    DEBUG_PRINTLN(F("Motor Stopped"));
    motor_status = STOPPING;
    updateMotorStatus();
    digitalWrite(directionPin1, LOW);
    digitalWrite(directionPin2, LOW);
    ledcWrite(speedPin, 0); // Set speed to 0
}

// Function to execute a gentle turn to the right
void gentleTurnRight(int leftDirectionPin1, int leftDirectionPin2, int leftSpeedPin,
                     int rightDirectionPin1, int rightDirectionPin2, int rightSpeedPin) {
    moveForward(leftDirectionPin1, leftDirectionPin2, leftSpeedPin, HIGHER_SPEED);  // Left motors move at higher speed
    moveForward(rightDirectionPin1, rightDirectionPin2, rightSpeedPin, REDUCED_SPEED); // Right motors move at reduced speed
}

// Function to execute a gentle turn to the left
void gentleTurnLeft(int leftDirectionPin1, int leftDirectionPin2, int leftSpeedPin,
                    int rightDirectionPin1, int rightDirectionPin2, int rightSpeedPin) {
    moveForward(leftDirectionPin1, leftDirectionPin2, leftSpeedPin, REDUCED_SPEED);  // Left motors move at reduced speed
    moveForward(rightDirectionPin1, rightDirectionPin2, rightSpeedPin, HIGHER_SPEED); // Right motors move at higher speed
}


// Function to rotate right (spin in place clockwise)
void rotateRight(int leftDirectionPin1, int leftDirectionPin2, int leftSpeedPin,
                 int rightDirectionPin1, int rightDirectionPin2, int rightSpeedPin, int duty_cycle) {
    moveForward(leftDirectionPin1, leftDirectionPin2, leftSpeedPin, duty_cycle);  // Left motors move forward
    moveBackward(rightDirectionPin1, rightDirectionPin2, rightSpeedPin, duty_cycle); // Right motors move backward
}

// Function to rotate left (spin in place counterclockwise)
void rotateLeft(int leftDirectionPin1, int leftDirectionPin2, int leftSpeedPin,
                int rightDirectionPin1, int rightDirectionPin2, int rightSpeedPin, int duty_cycle) {
    moveBackward(leftDirectionPin1, leftDirectionPin2, leftSpeedPin, duty_cycle); // Left motors move backward
    moveForward(rightDirectionPin1, rightDirectionPin2, rightSpeedPin, duty_cycle);  // Right motors move forward
}




void testMotor() {
  // Move the DC motor forward at maximum speed
  moveForward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, HIGHER_SPEED);
  // moveForward(MotorLeftFrontDirectionPin1, MotorLeftFrontDirectionPin2, MotorLeftFrontSpeedPin, HIGHER_SPEED);
  delay(TEST_DELAY);
  // Stop the DC motor
  DEBUG_PRINTLN(F("Motor Stopped"));
  motor_status = STOPPING;
  updateMotorStatus();
  stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
  // stopMotor(MotorLeftFrontDirectionPin1, MotorLeftFrontDirectionPin2, MotorLeftFrontSpeedPin);
  delay(TEST_DELAY);
  moveBackward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, REDUCED_SPEED);
  // moveBackward(MotorLeftFrontDirectionPin1, MotorLeftFrontDirectionPin2, MotorLeftFrontSpeedPin, REDUCED_SPEED);
  delay(TEST_DELAY);
  // Stop the DC motor
  stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
  // stopMotor(MotorLeftFrontDirectionPin1, MotorLeftFrontDirectionPin2, MotorLeftFrontSpeedPin);
  delay(TEST_DELAY);
  
}



void stateMachine(unsigned int code,int distance, Reasoning reason)
{
    // Serial.println(distance);
    // Serial.println(reason);
    // Handle state transitions based on the received code
    switch (code) {
      case FORWARD:
        currentState = STATE_FORWARD;
        break;
      case REVERSE:
        currentState = STATE_REVERSE;
        break;
      case LEFT:
        currentState = STATE_LEFT;
        break;
      case RIGHT:
        currentState = STATE_RIGHT;
        break;
      case STOP:
        currentState = STATE_STOP;
        break;
      case KEY_1:
        driving_speed = 100;  
        break;
      case KEY_2:
        driving_speed = 200;  
        break;
      case KEY_3:
        driving_speed = 255;  
        break;
      case KEY_4:
        // moveForward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);  // Left motors move at reduced speed
        break;
      case KEY_5:
        // moveForward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);  // Left motors move at reduced speed
        break;   
      case KEY_6:
        // moveForward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);  
        // moveBackward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);  
        break;  
      case KEY_7:
        // moveForward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);  
        // moveBackward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);  
        break;    
      case HASH:
        enableNavigation = 0;
        stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
        stopMotor(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED);
        Serial.println(F("Disabling Navigation... "));
        // moveForward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);  
        // moveBackward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);  
        break;          
      default:
        // Unknown code, do nothing
        break;
    }
  
    
  // Execute the current state
  switch (currentState) {
    case STATE_FORWARD:
      moveForward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);
      moveForward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);
      break;
    case STATE_REVERSE:
      moveBackward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, driving_speed);
      moveBackward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);
      break;
    case STATE_LEFT:
      rotateLeft(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED,
                     MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);
      break;
    case STATE_RIGHT:
      rotateRight(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED,
                      MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, driving_speed);
      break;
    case STATE_STOP:
      stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
      stopMotor(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED);
      break;
    case STATE_IDLE:
    default:
      // Do nothing
      break;
  }
  processLogs(currentState, distance,reason,0);
}
#endif
void showKey(unsigned int decodedValue) {
    Serial.print(F("Key Pressed: "));
    switch (decodedValue) {
        case 0xFF4AB5: Serial.println("Key 0"); break;
        case 0xFF6897: Serial.println("Key 1"); break;
        case 0xFF9867: Serial.println("Key 2"); break;
        case 0xFFB04F: Serial.println("Key 3"); break;
        case 0xFF30CF: Serial.println("Key 4"); break;
        case 0xFF18E7: Serial.println("Key 5"); break;
        case 0xFF629D: Serial.println("Forward"); break;
        case 0xFF22DD: Serial.println("Left"); break;
        case 0xFFC23D: Serial.println("Right"); break;
        case 0xFFA857: Serial.println("Backward"); break;
        case 0xFF02FD: Serial.println("Stop"); break;
        case 0xFF42BD: Serial.println("*"); break;
        case 0xFF52AD: Serial.println("#"); break;        
        default: Serial.println("Unknown Key");
    }
}



void checkIRDecoder() {
    if (irDecoder.available()) {
        turnOnLed(STATUS_LED);  
        Serial.print(F("Decoded NEC Data: 0x"));
        Serial.print(irDecoder.getDecodedData(), HEX);
        // 0xFF6897 Key 1
        // 0xFF9867 Key 2
        // 0xFFB04F Key 3
        // 0xFF30CF Key 4
        // 0xFF18E7 Key 5
        // 0xFF629D Forward
        // 0xFF22DD Left
        // 0xFF22DD Right
        // 0xFFA857 Backward
        // 0xFF02FD Stop
        if (irDecoder.isRepeatSignal()) {
            Serial.println(F(" (REPEATED)"));
        } else {
            Serial.println(F(" (NEW PRESS)"));
        }
        delay(100);
        turnOffLed(STATUS_LED);
    }
}

void setup() {
  // pinMode(SW1, INPUT);  // Configure GPIO34 as an input
  // pinMode(SW2, INPUT);  // Configure GPIO34 as an input


  pinMode(HC_SR04_ECHO_PIN, INPUT);
  pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  
  digitalWrite(ERROR_LED_PIN, HIGH); // Turn on initially
  Serial.begin(115200);

#ifdef HARDWARE_SERIAL
    // Secondary port
  SerialExt.begin(
      115200,            // baud rate
      SERIAL_8N1,        // data/parity/stop
      UART_RX_PIN,       // receive pin
      UART_TX_PIN        // transmit pin
  );
#endif
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  delay(100);
  Serial.println("Serial initialised!");
  #if 1
  #ifdef ENABLE_MOTOR_CONTROL
  setupMotorControl();
  #endif
  setupRotaryEncoder();
  #ifdef MCP23017
  setupMCP();
  #endif
  irDecoder.begin();
  DEBUG_PRINT_INFO(F("Setting up OLED Display"));
  u8g2_prepare();
  u8g2.begin();
  DEBUG_PRINT_INFO(F("Updating Display"));
  updateProgram();
  turnOnLed(STATUS_LED);
  setupWifi();
  WiFi.config(device_ip,  gateway_ip, subnet_mask,dns_ip_1,dns_ip_2);
  WiFi.begin(ssid, pass);
  DEBUG_PRINT_INFO(F("WiFi connecting."));
  while (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINT_INFO(F("-"));
      delay(500);
      DEBUG_PRINT_INFO(F("."));
  }
  digitalWrite(ERROR_LED_PIN, LOW); // Turn off after connecting

  DEBUG_PRINT_INFO(F("WiFi connected."));
  turnOffLed(STATUS_LED);
  #endif
}


void reconnectToServer() {
  Serial.print(F("Connecting to server at "));
  digitalWrite(ERROR_LED_PIN, HIGH); // Turn on when reconnecting
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  running_state = RECONNECTING;
  updateProgram();
  if (client.connect(serverIP, serverPort)) {
      Serial.println(F("Connected to server!"));
      running_state = CONNECTED;
      digitalWrite(ERROR_LED_PIN, LOW); // Turn off when connected
      updateProgram();
  } else {
      Serial.println(F("Connection failed."));
      running_state = ERROR;
      updateProgram();
      digitalWrite(ERROR_LED_PIN, HIGH); // Turn on when error occurs
      delay(1000);
  }
}

void processLogs(unsigned int state, unsigned int distance, unsigned int reason, unsigned int priority)
{
    if (!client.connected()) {
        reconnectToServer();
    }
    unsigned long currentMillis = millis();
    if ((currentMillis - lastLogTime >= logInterval) || priority) {
        lastLogTime = currentMillis;
        // Create a JSON object
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["time"] = millis();
        jsonDoc["distance"] = distance;
        jsonDoc["state"] = state;
        jsonDoc["reason"] = reason;
        jsonDoc["rpm"] = rpm;
        // Convert JSON to string
        String logMessage;
        serializeJson(jsonDoc, logMessage);
        // Serial.print(F("Sending: "));
        // Serial.println(logMessage);
        const int maxRetries = 3;  // Retry sending up to 3 times
        int retries = 0;
        bool success = false;
        while (retries < maxRetries) {
            size_t bytesWritten = client.write((const uint8_t*)logMessage.c_str(), logMessage.length());

            if (bytesWritten == logMessage.length()) {
                success = true;
                break;  // Exit loop if successful
            } else {
                Serial.println(F("Failed to send log message, retrying..."));
                retries++;
                delay(100);  // Small delay before retrying
            }
        }
        if (!success) {
            Serial.println(F("Error: Failed to send log message after multiple attempts."));
            running_state = ERROR;
            digitalWrite(ERROR_LED_PIN, HIGH); // Turn on when error occurs
            updateProgram();
            client.stop();  // Reset connection to avoid being stuck
            reconnectToServer();
        }
    }
}

#ifdef ENABLE_MOTOR_CONTROL
void performRecovery(unsigned int distance) {
    if(enableNavigation)
    {
      turnOnLed(MODE_LED);
      stopCount = 0;
      stopStartTime = 0;
      int turnTime = 500 + (recoveryAttempts * 200);  // Increase turn duration gradually
      int reverseTime = 1000 + (recoveryAttempts * 300); // Increase reverse time
      // Alternate left/right to avoid looping
      stateMachine(REVERSE, distance, RECOVERING);
      delay(reverseTime);
      
      if (recoveryAttempts % 2 == 0) {
          stateMachine(LEFT, distance, RECOVERING);
      } else {
          stateMachine(RIGHT, distance, RECOVERING);
      }
      delay(turnTime);
      recoveryAttempts++;
      if (recoveryAttempts > 3) recoveryAttempts = 0;
      turnOffLed(MODE_LED);
  }
}
#endif

#ifdef ENABLE_MOTOR_CONTROL
void process_navigation_information(unsigned int distance)
{
    if (irDecoder.available()) {
        unsigned int decodedData = irDecoder.getDecodedData();
        Serial.print(F("Decoded NEC Data: 0x"));
        showKey(decodedData);
        Serial.print(decodedData, HEX);

        // Add a simple debounce mechanism for all new IR presses
        // This ensures a command isn't re-processed too quickly
        // after being received once.
        static unsigned long lastIRCommandTime = 0;
        const unsigned long IR_DEBOUNCE_DELAY = 300; // milliseconds

        if (millis() - lastIRCommandTime < IR_DEBOUNCE_DELAY && !irDecoder.isRepeatSignal()) {
            // This is a new press, but too soon after the last one, ignore.
            // Allow repeat signals to pass if desired for continuous movement holding down key.
            return;
        }
        lastIRCommandTime = millis(); // Update the time of the last processed command

        // If '#' is pressed, toggle the navigation mode
        if (decodedData == HASH) {
            enableNavigation = !enableNavigation; // Toggle 0 to 1, or 1 to 0
            if (enableNavigation) {
                Serial.println(F("Navigation ENABLED."));
                strcpy(gContent2, "Auto Nav");
                running_state = NORMAL;
            } else {
                Serial.println(F("Navigation DISABLED (Manual Control)."));
                strcpy(gContent2, "Manual");
                running_state = MANUAL;
                stateMachine(STOP, distance, IR_CONTROL); // Stop motors immediately
            }
            updateProgram();
            // Crucial: A short delay after a mode change command to prevent
            // immediate re-interpretation if the button is held slightly too long.
            delay(2000); // Small delay to "settle" after a mode toggle
            return; // Exit after processing HASH
        }
        if(decodedData == KEY_4)
        {
            triggerStill();
        }

        // Process other IR commands only if in Manual mode (navigation disabled)
        if (!enableNavigation) {
            stateMachine(decodedData, distance, IR_CONTROL);
            stopCount = 0;
            stopStartTime = 0;
            recoveryAttempts = 0;
        } else {
            if (decodedData == STOP) { // Allow STOP command even in autonomous mode
                stateMachine(STOP, distance, IR_CONTROL);
            }
        }
    }

    // Process autonomous navigation logic ONLY if navigation is enabled
    if (enableNavigation)
    {
        if (distance == 0) {
            Serial.print(F("Out of Range: "));
            Serial.println(distance);
            stateMachine(FORWARD, distance, OUT_OF_RANGE);
        }
        else if ((distance < OBSTACLE_STOP_THRESHOLD) && (distance > OBSTACLE_SLOW_THRESHOLD)) {
            int randomTurn = random(0, 2);
            if (randomTurn == 0) {
                stateMachine(LEFT, distance, UNCERTAIN);
            } else {
                stateMachine(RIGHT, distance, UNCERTAIN);
            }
            stopCount = 0;
        }
        else if (distance < OBSTACLE_STOP_THRESHOLD) {
            if (stopStartTime == 0) {
                stopStartTime = millis();
            }
            stateMachine(STOP, distance, BLOCKED);
            stopCount++;

            if ((millis() - stopStartTime > stuckThreshold) || stopCount >= maxStopAttempts) {
                performRecovery(distance);
            }
        }
        else {
            stateMachine(FORWARD, distance, CLEAR_PATH);
            stopCount = 0;
            stopStartTime = 0;
        }
    }
}
#endif

void processEncoder()
{
  if(millis()-encoder_previous_time >=100)
  { // Updating every 0.1 seconds
    detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
    rpm = (60 * 100 / pulses_per_turn )/ (millis() - encoder_previous_time)* pulses;
    encoder_previous_time=millis();
    pulses=0;
    DEBUG_PRINT(F("RPM= "));
    DEBUG_PRINT(rpm);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0  
  }
}

#ifdef MCP23017
void process_inta_interrupt()
{
  if (interruptOccurred) {
    interruptOccurred = false; // Reset the flag immediately
    delayMicroseconds(10); // Short delay before reading registers
    uint8_t lastInterruptPin = mcp.getLastInterruptPin(); // get the pin
    if (lastInterruptPin != 255) { // ignore interrupts from pin 255
      Serial.print("Interrupt detected on pin: ");
      Serial.println(lastInterruptPin);
      Serial.print("Pin states at time of interrupt: 0b");
      processLogs(currentState, 2,CRASH_DETECT,1);
      #ifdef ENABLE_MOTOR_CONTROL
      process_navigation_information(2);
      #endif
      Serial.println(mcp.getCapturedInterrupt(), 2);

      // Debugging: Print captured interrupt values.
      Serial.print("Captured Interrupt: 0x");
      Serial.println(mcp.getCapturedInterrupt(), HEX);
      delay(250); // debounce.
      //Debugging: print captured interrupt after clear.
      Serial.print("Captured Interrupt after clear: 0x");
      Serial.println(mcp.getCapturedInterrupt(), HEX);
    } else {
      Serial.println("Ignored interrupt from pin 255");
      mcp.clearInterrupts(); // clear the interrupt even if ignored.
    }
    mcp.clearInterrupts(); // Clear the interrupt
  }
}


void process_mpc_inta()
{
  if (!digitalRead(INT_PIN))
  {
    Serial.print("Interrupt detected on pin: ");
    Serial.println(mcp.getLastInterruptPin());
    Serial.print("Pin states at time of interrupt: 0b");
    Serial.println(mcp.getCapturedInterrupt(), 2);
    processLogs(currentState, 2,6,1);
    #ifdef ENABLE_MOTOR_CONTROL
    stateMachine(STOP, 2, BLOCKED);
    stopCount++;
    performRecovery(2);
    #endif
    // delay(250);  // debounce
    // // NOTE: If using DEFVAL, INT clears only if interrupt
    // // condition does not exist.
    // // See Fig 1-7 in datasheet.
    mcp.clearInterrupts();  // clear
  }
}
#endif

void loop() {
  printContent(displayItems, sizeof(displayItems) / sizeof(displayItems[0]));

  unsigned int distance;

  bool sw1_state = mcp.digitalRead(SW1);   // HIGH = released, LOW = pressed
  bool sw2_state = mcp.digitalRead(SW2);
  // Example: SW1 as a 'Stop All' button (highest priority)
  if (sw1_state == LOW) { // Assuming active LOW
      Serial.println(F("SW1 Pressed: Emergency Stop!"));
      stateMachine(STOP, distance, CRASH_DETECT); // Use CRASH_DETECT or a new REASON_MANUAL_STOP
      enableNavigation = 0; // Force into manual mode and stop
      strcpy(gContent2, "Emergency Stop"); // Update display
      updateProgram();
      digitalWrite(ERROR_LED_PIN, HIGH); // Indicate error/stop state
      delay(500); // Debounce
  }
  // Example: SW2 to temporarily activate/deactivate navigation (alternative to IR # key)
  // Be careful if both IR and SW2 toggle the same variable, ensure debouncing.
  if (sw2_state == LOW) { // Assuming active LOW
      if (!lastSw2State) { // Only trigger on a state change (press)
          enableNavigation = !enableNavigation; // Toggle mode
          if (enableNavigation) {
              Serial.println(F("SW2 Pressed: Navigation ENABLED."));
              strcpy(gContent2, "Auto Nav");
              running_state = NORMAL;
          } else {
              Serial.println(F("SW2 Pressed: Navigation DISABLED (Manual)."));
              strcpy(gContent2, "Manual");
              running_state = MANUAL;
              stateMachine(STOP, distance, IR_CONTROL); // Stop when entering manual
          }
          updateProgram();
          digitalWrite(MODE_LED, enableNavigation ? HIGH : LOW); // Indicate mode
          delay(500); // Debounce
      }
      lastSw2State = true; // Store current state for next loop
  } else {
      lastSw2State = false; // Reset state when button is released
      digitalWrite(ERROR_LED_PIN, LOW); // Turn off error LED if not pressed
  }
  // --- End Re-added SW1 and SW2 handling ---


  #ifdef ENABLE_HC_SR04
  distance = processHCSR04();
  #endif

  #ifdef ENABLE_MOTOR_CONTROL
  // process_navigation_information now handles IR and decides motor action based on 'enableNavigation'
  process_navigation_information(distance);
  #endif

  processEncoder();

  #ifdef MCP23017
  process_mpc_inta();
  #endif
#ifdef HARDWARE_SERIAL
  //   /* Echo data from external device to USB console */
  // while (SerialExt.available())
  // {
  //   int c = SerialExt.read();
  //   Serial.write(c);
  // }

  // /* And from USB console back to the external device */
  // while (Serial.available())
  // {
  //   int c = Serial.read();
  //   SerialExt.write(c);
  // }
  #endif

}

