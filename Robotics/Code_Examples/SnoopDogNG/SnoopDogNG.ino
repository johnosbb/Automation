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

#define PROGRAM_ONE 1
#define PROGRAM_TWO 2
#define PROGRAM_THREE 3
#define PROGRAM_FOUR 4
#define PROGRAM_IDLE 0

#define MOVING_FORWARD 0
#define MOVING_REVERSE 1
#define STOPPING 2

#define TEST_DELAY 4000

unsigned int active_program = PROGRAM_IDLE;
unsigned int motor_status = STOPPING;
uint32_t chipId = 0;
unsigned int driving_speed = DEFAULT_DUTY_CYCLE;


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
    switch(active_program) {
        case 0:
            strcpy(gContent2,"Idle");
            break;
        case 1: 
            strcpy(gContent2,"One");
            break;
        case 2: 
            strcpy(gContent2,"Two");
            break;
        case 3: 
            strcpy(gContent2,"Three");
            break;
        case 4: 
            strcpy(gContent2,"Four");
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
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");
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

void stateMachine(unsigned int code,unsigned int distance, unsigned int reason)
{
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
  
  showKey(code);
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
    Serial.print("Key Pressed: ");
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
        Serial.print("Decoded NEC Data: 0x");
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
            Serial.println(" (REPEATED)");
        } else {
            Serial.println(" (NEW PRESS)");
        }
    }
}

void setup() {
  pinMode(SW1, INPUT);  // Configure GPIO34 as an input
  pinMode(SW2, INPUT);  // Configure GPIO34 as an input
  pinMode(HC_SR04_ECHO_PIN, INPUT);
  pinMode(HC_SR04_TRIGGER_PIN, OUTPUT);

  Serial.begin(115200);
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.disconnect();
  #ifdef ENABLE_MOTOR_CONTROL
  setupMotorControl();
  #endif
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
}


void reconnectToServer() {
  Serial.print("Connecting to server at ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);

  if (client.connect(serverIP, serverPort)) {
      Serial.println("Connected to server!");
  } else {
      Serial.println("Connection failed.");
      delay(1000);
  }

}

void processLogs(unsigned int state,unsigned int distance,unsigned int reason)
{
    if (!client.connected()) {
        reconnectToServer();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastLogTime >= logInterval) {
        lastLogTime = currentMillis;
        String logMessage = "Time: " + String(millis()) + " ms, Distance: " + String(distance) + " cm, State: " + state + ", Reason: " + reason + "\n";
        Serial.print("Sending: ");
        Serial.println(logMessage);

        client.print(logMessage);
    }
}

void loop() {
  unsigned int sw1 = digitalRead(SW1);  // Read the input state
  unsigned int distance;
  // DEBUG_PRINTLN(sw1);               // Print the value to Serial Monitor
  unsigned int sw2 = digitalRead(SW2);  // Read the input state
  // DEBUG_PRINTLN(sw2);               // Print the value to Serial Monitor
  //testMotor();
  #ifdef ENABLE_MOTOR_CONTROL
  if(sw1 == 0)
  {
    // moveBackward(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED, HIGHER_SPEED);
    // moveBackward(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED, HIGHER_SPEED);
    Serial.println("SW1 Switch ....");
    distance = processHCSR04();

  }
  if(sw2 == 0)
  {
    // stopMotor(MOTOR_LEFT_DIRECTION_1, MOTOR_LEFT_DIRECTION_2, MOTOR_LEFT_SPEED);
    // stopMotor(MOTOR_RIGHT_DIRECTION_1, MOTOR_RIGHT_DIRECTION_2, MOTOR_RIGHT_SPEED);
    distance = processHCSR04();
  }
  #endif
  distance = processHCSR04();
  if (irDecoder.available()) {
        unsigned int decodedData = irDecoder.getDecodedData();
        Serial.print("Decoded NEC Data: 0x");
        Serial.print(decodedData, HEX);
        stateMachine(decodedData,distance,IR_CONTROL);
      }

    if (distance == 0) {
      // Target is out of range, keep moving forward
      stateMachine(FORWARD,distance,OUT_OF_RANGE);  // Replace FORWARD with the appropriate command to move forward
    } 
    else if ((distance < OBSTACLE_STOP_THRESHOLD) && (distance > OBSTACLE_SLOW_THRESHOLD)) {
      // Randomly turn left or right if distance is between 20 and 30
      int randomTurn = random(0, 2);  // Generates either 0 or 1
      if (randomTurn == 0) {
        stateMachine(LEFT,distance,UNCERTAIN);
      } else {
        stateMachine(RIGHT,distance,UNCERTAIN);
      }
      stopCount = 0; // Reset stop counter when moving
    }

    else if (distance < OBSTACLE_STOP_THRESHOLD) {
      // If the car stops, track how long it's been stuck
      if (stopStartTime == 0) {
        stopStartTime = millis();  // Start tracking time when first stopping
      }
      
      stateMachine(STOP,distance,BLOCKED);
      stopCount++;

      // If stuck for too long or stopped too many times, try recovery
      if ((millis() - stopStartTime > stuckThreshold) || stopCount >= maxStopAttempts) {
        stopCount = 0; // Reset stop count
        stopStartTime = 0; // Reset timer

        // Try a recovery move
        int recoveryMove = random(0, 2); // Randomly pick a strategy
        if (recoveryMove == 0) {
          stateMachine(REVERSE,distance,RECOVERING);
          delay(1000);  // Move back for a second
          stateMachine(LEFT,distance,RECOVERING);
          delay(500);   // Turn left
        } else {
          stateMachine(REVERSE,distance,RECOVERING);
          delay(1000);
          stateMachine(RIGHT,distance,RECOVERING);
          delay(500);
        }
      }
    }

    else {
      // Continue moving forward if distance is greater than or equal to 30
      stateMachine(FORWARD,distance,CLEAR_PATH);
      stopCount = 0; // Reset stop counter when moving
      stopStartTime = 0; // Reset stuck timer
    }
    //checkIRDecoder();
    //delay(1000);
  // Serial.println("Main Loop .....");
}
