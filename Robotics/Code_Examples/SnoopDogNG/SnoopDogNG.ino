/* The true ESP32 chip ID is essentially its MAC address.
This sketch provides an alternate chip ID that matches
the output of the ESP.getChipId() function on ESP8266
(i.e. a 32-bit integer matching the last 3 bytes of
the MAC address. This is less unique than the
MAC address chip ID, but is helpful when you need
an identifier that can be no more than a 32-bit integer
(like for switch...case).

created 2020-06-07 by cweinhofer
with help from Cicicok */

#include <U8g2lib.h>
#include <debug.h>
#include "WiFi.h"
#include <config.h>
#include <esp_system.h> // For esp_read_mac
#include "HX1838Decoder.h"
#include <NewPing.h>
#include <ArduinoJson.h>


#include <Adafruit_MCP23X17.h>

#define BUTTON_PIN 1   // MCP23XXX pin used for interrupt

#define INT_PIN 32      // microcontroller pin attached to INTA/B


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


// Motor Control

// Left Motor Rear
#define ENABLE_MOTOR_CONTROL
#define MOTOR_LEFT_DIRECTION_1  26
#define MOTOR_LEFT_DIRECTION_2  27
#define MOTOR_LEFT_SPEED  14
#define MOTOR_RIGHT_DIRECTION_1  19
#define MOTOR_RIGHT_DIRECTION_2  18
#define MOTOR_RIGHT_SPEED  23
#define SW1 34  // GPIO34
#define SW2 35  // GPIO35


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
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
#define HC_SR04_ECHO_PIN 36 // GPIO36
#define HC_SR04_TRIGGER_PIN 16 // GPIO16
NewPing sonar(HC_SR04_TRIGGER_PIN, HC_SR04_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


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



IRDecoder irDecoder(IR_RECEIVE_PIN);  // IR Receiver on IR_RECEIVE_PIN
Adafruit_MCP23X17 mcp;

// Define IR codes
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
const unsigned long STOP = 0xFF02FD;


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


// Reasoning enumeration
enum Reason {
  IR_CONTROL,
  CLEAR_PATH,
  OUT_OF_RANGE,
  BLOCKED,
  RECOVERING,
  UNCERTAIN
};


#define OBSTACLE_STOP_THRESHOLD 30
#define OBSTACLE_SLOW_THRESHOLD 40

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



void setupMCP() {
  Serial.println("MCP23xxx Configuration!");

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

}


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

int processHCSR04()
{

  delay(50);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int distance = sonar.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(F("cm"));
  return distance;
}

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

#endif

void stateMachine(unsigned int code,int distance, Reason reason)
{
    Serial.print(F("State Machine Recieved Distance: "));
    Serial.println(distance);
    Serial.print(F("State Machine Recieved Reason: "));
    Serial.println(reason);
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
      default:
        // Unknown code, do nothing
        break;
    }
  
  //showKey(code);
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
  processLogs(currentState, distance,reason);
}

void showKey(unsigned int decodedValue) {
    Serial.print(F("Key Pressed: "));
    switch (decodedValue) {
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
        default: Serial.println("Unknown Key");
    }
}



void checkIRDecoder() {
    if (irDecoder.available()) {
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
    }
}

void setup() {
  pinMode(SW1, INPUT);  // Configure GPIO34 as an input
  pinMode(SW2, INPUT);  // Configure GPIO34 as an input
  pinMode(HC_SR04_ECHO_PIN, INPUT);
  pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  
  digitalWrite(ERROR_LED_PIN, HIGH); // Turn on initially
  Serial.begin(115200);
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  #ifdef ENABLE_MOTOR_CONTROL
  setupMotorControl();
  #endif
  setupRotaryEncoder();
  setupMCP();
  irDecoder.begin();
  DEBUG_PRINT_INFO(F("Setting up OLED Display"));
  u8g2_prepare();
  u8g2.begin();
  DEBUG_PRINT_INFO(F("Updating Display"));
  updateProgram();
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

void processLogs(unsigned int state, unsigned int distance, unsigned int reason)
{
    if (!client.connected()) {
        reconnectToServer();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastLogTime >= logInterval) {
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

        Serial.print(F("Sending: "));
        Serial.println(logMessage);

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


void process_navigation_information(unsigned int distance)
{
    if (irDecoder.available()) {
        unsigned int decodedData = irDecoder.getDecodedData();
        Serial.print(F("Decoded NEC Data: 0x"));
        Serial.print(decodedData, HEX);
        stateMachine(decodedData, distance, IR_CONTROL);
        running_state = RECEIVING;
        updateProgram();
    }

    if (distance == 0) {
        // Target is out of range, keep moving forward
        Serial.print(F("Out of Range: "));
        Serial.println(distance);
        stateMachine(FORWARD, distance, OUT_OF_RANGE);
    } 
    else if ((distance < OBSTACLE_STOP_THRESHOLD) && (distance > OBSTACLE_SLOW_THRESHOLD)) {
        // Randomly turn left or right if distance is between 20 and 30
        int randomTurn = random(0, 2);  // Generates either 0 or 1
        if (randomTurn == 0) {
            stateMachine(LEFT, distance, UNCERTAIN);
        } else {
            stateMachine(RIGHT, distance, UNCERTAIN);
        }
        stopCount = 0; // Reset stop counter when moving
    }
    else if (distance < OBSTACLE_STOP_THRESHOLD) {
        // If the car stops, track how long it's been stuck
        if (stopStartTime == 0) {
            stopStartTime = millis();  // Start tracking time when first stopping
        }

        stateMachine(STOP, distance, BLOCKED);
        stopCount++;

        // If stuck for too long or stopped too many times, try recovery
        if ((millis() - stopStartTime > stuckThreshold) || stopCount >= maxStopAttempts) {
            stopCount = 0;
            stopStartTime = 0;

            int turnTime = 500 + (recoveryAttempts * 200);  // Increase turn duration gradually
            int reverseTime = 1000 + (recoveryAttempts * 300); // Increase reverse time

            // Alternate left/right to avoid looping
            if (recoveryAttempts % 2 == 0) {
                stateMachine(REVERSE, distance, RECOVERING);
                delay(reverseTime);
                stateMachine(LEFT, distance, RECOVERING);
                delay(turnTime);
            } else {
                stateMachine(REVERSE, distance, RECOVERING);
                delay(reverseTime);
                stateMachine(RIGHT, distance, RECOVERING);
                delay(turnTime);
            }

            recoveryAttempts++;  // Increase attempts for next time
            if (recoveryAttempts > 3) recoveryAttempts = 0;  // Reset if too many tries
        }
    }
    else {
        // Continue moving forward if distance is greater than or equal to 30
        stateMachine(FORWARD, distance, CLEAR_PATH);
        stopCount = 0; // Reset stop counter when moving
        stopStartTime = 0; // Reset stuck timer
    }
}


void processEncoder()
{
  if(millis()-encoder_previous_time >=100)
  { // Updating every 0.1 seconds
    detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
    rpm = (60 * 100 / pulses_per_turn )/ (millis() - encoder_previous_time)* pulses;
    encoder_previous_time=millis();
    pulses=0;
    Serial.print("RPM= ");
    Serial.println(rpm);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0  
  }
}

void ProcessMCP()
{
  if (!digitalRead(INT_PIN))
  {
    Serial.print("Interrupt detected on pin: ");
    Serial.println(mcp.getLastInterruptPin());
    Serial.print("Pin states at time of interrupt: 0b");
    Serial.println(mcp.getCapturedInterrupt(), 2);
    // delay(250);  // debounce
    // // NOTE: If using DEFVAL, INT clears only if interrupt
    // // condition does not exist.
    // // See Fig 1-7 in datasheet.
    mcp.clearInterrupts();  // clear
  }
}

void loop() {
  unsigned int sw1 = digitalRead(SW1);  // Read the input state
  unsigned int distance;
  int pinState = digitalRead(ENCODER_PIN); // Get current logic level
  // DEBUG_PRINTLN(sw1);               // Print the value to Serial Monitor
  unsigned int sw2 = digitalRead(SW2);  // Read the input state
  // DEBUG_PRINTLN(sw2);               // Print the value to Serial Monitor
  //testMotor();
  #ifdef ENABLE_MOTOR_CONTROL
  if(sw1 == 0)
  {
    // moveBackward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, HIGHER_SPEED);
    // moveBackward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, HIGHER_SPEED);
    Serial.println(F("SW1 Switch ...."));
    distance = processHCSR04();

  }
  if(sw2 == 0)
  {
    // stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
    // stopMotor(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED);
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  #endif
  distance = processHCSR04();
  process_navigation_information(distance);
  processEncoder();
      

  Serial.print("Encoder Pin State: ");
  Serial.println(pinState); // Will be 0 (LOW) or 1 (HIGH)
  ProcessMCP();
    //checkIRDecoder();
    //delay(1000);
  // Serial.println("Main Loop .....");
}
