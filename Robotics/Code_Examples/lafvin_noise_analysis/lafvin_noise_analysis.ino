#include <Servo.h>
#include <IRremote.h>
#include <StensTimer.h>
// #include <Arduino.h>
#include <U8g2lib.h> // https://github.com/olikraus/u8g2/blob/master/doc/faq.txt#L167 how to reduce memory
#include <Adafruit_GFX.h>
#include "Adafruit_VL6180X.h"
#include <avr/wdt.h>


#define OLED_ENABLED
//#define LASER_DETECTION


#define DEBUG_SERIAL  // Uncomment to enable debugging

// Debugging Levels
#define DEBUG_LEVEL_NONE       0  // No debugging
#define DEBUG_LEVEL_ERROR      1  // Errors only
#define DEBUG_LEVEL_WARN       2  // Warnings and Errors
#define DEBUG_LEVEL_INFO       3  // Info, Warnings, and Errors
#define DEBUG_LEVEL_DEBUG      4  // Basic Debugging
#define DEBUG_LEVEL_VERBOSE    5  // Verbose Debugging
#define DEBUG_LEVEL_VERY_VERBOSE 6  // Very Verbose Debugging

// Set the desired debug level here
#define DEBUG_LEVEL DEBUG_LEVEL_DEBUG

#ifdef DEBUG_SERIAL
  // Error
  #define DEBUG_PRINT_ERROR(x)  if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR) { Serial.print("[ERROR] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Warning
  #define DEBUG_PRINT_WARN(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_WARN) { Serial.print("[WARNING] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Info
  #define DEBUG_PRINT_INFO(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO) { Serial.print("[INFO] "); Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Basic Debug
  #define DEBUG_PRINT(x)        if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) {  Serial.print(x); }
  #define DEBUG_PRINTLN(x)      if (DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG) {  Serial.println(x); }

  // Verbose Debug
  #define DEBUG_PRINT_VERBOSE(x)   if (DEBUG_LEVEL >= DEBUG_LEVEL_VERBOSE) {  Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }

  // Very Verbose Debug
  #define DEBUG_PRINT_VERY_VERBOSE(x) if (DEBUG_LEVEL >= DEBUG_LEVEL_VERY_VERBOSE) {  Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" - "); Serial.println(x); }
#else
  #define DEBUG_PRINT_ERROR(x)
  #define DEBUG_PRINT_WARN(x)
  #define DEBUG_PRINT_INFO(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_VERBOSE(x)
  #define DEBUG_PRINT_VERY_VERBOSE(x)
#endif







#ifdef LASER_DETECTION
Adafruit_VL6180X laserSensor = Adafruit_VL6180X();
#endif

#ifdef OLED_ENABLED
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);  // assumes hardware I2C
// Use software I²C directly with U8G2
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/8, /* data=*/9, /* reset=*/U8X8_PIN_NONE);
#endif


/* Main Defines */
#define RIGHT 180
#define MIDDLE 90
#define LEFT 0
#define ANTI_CLOCKWISE LOW
#define CLOCKWISE HIGH
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 4
#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_PWM 6
#define LEFT_MICRO_SWITCH A1
#define RIGHT_MICRO_SWITCH A2
#define ULTRA_SONIC_SERVO 10
#define MOTOR_SPEED 100
#define MOTOR_TURN_SPEED_HIGH 255
#define MOTOR_TURN_SPEED_LOW 100
#define SETTLING_DELAY 1000
#define MAIN_LOOP_DELAY_DEFAULT 1000
#define TURN_DELAY 400
#define RECOVERY_TIME 3000
#define ULTRA_SONIC_TRIGGER_PIN 12
#define ULTRA_SONIC_ECHO_PIN 13
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 2
#define DIRECTION_STOP 0
#define SAFE_DISTANCE 30  // Example threshold in cm

/* General Globals */
float max_valid_distance = 400.0;
bool movementEnabled = false;
bool automationEnabled = true;
int motor_speed = MOTOR_SPEED;
char bluetooth_data;
Servo UltraSonicServo;
IRrecv irrecv(3);
int lastKnownDirection = DIRECTION_STOP;
unsigned int disableMotorsDuringDetection = 0;

/* Timer Section */
StensTimer* mainTimer;
Timer* microSwitchDetectTimer = NULL;
Timer* laserDetectTimer = NULL;
Timer* irReceiveTimer = NULL;
Timer* MovementTimer = NULL;
#define MICRO_DETECT_ACTION 1
#define IR_RECEIVE_ACTION 2
#define MOVEMENT_ACTION 3
#define LASER_DETECT_ACTION 4




void processIR()
{
  long ir_rec = 0;
  decode_results results;
  DEBUG_PRINT_VERBOSE("Checking IR Commands");
  if (irrecv.decode(&results)) {    
    ir_rec = results.value;
    DEBUG_PRINT(F("Received IR code: "));
    DEBUG_PRINT(ir_rec);
    irrecv.resume(); // Receive the next value
  } 
  else
    return;

  // Check which code was received and execute the corresponding function
  if (ir_rec == 0xFF629D) { 
    DEBUG_PRINTLN(F("IR Command: Move Forward"));
    forward();
  }
  else if (ir_rec == 0xFFA857) {
   
    DEBUG_PRINTLN(F("IR Command: Move Backward"));
    backward();
  }
  else if (ir_rec == 0xFF22DD) { // left arrow
    DEBUG_PRINTLN(F("IR Command: Turn Left"));
    turnLeft();
  }
  else if (ir_rec == 0xFFC23D) { // right arrow
    DEBUG_PRINTLN(F("IR Command: Turn Right"));
    turnRight();
  }
  else if (ir_rec == 0xFF30CF) { // key 4  
    DEBUG_PRINTLN(F("IR Command: Rotate Left"));
    rotateLeft();
  }
  else if (ir_rec == 0xFF7A85) { // key 6 
    DEBUG_PRINTLN(F("IR Command: Rotate Right"));
    rotateRight();
  }
  else if (ir_rec == 0xFF02FD) {
    DEBUG_PRINTLN(F("IR Command: Stop"));
    stopMotors();
    

  }
  else if (ir_rec == 0xFF6897) { //Key number 1
    int DR = getDistanceAtAngle(180);  // Distance on the right (0 degrees)
    int DL = getDistanceAtAngle(90);    // Distance on the left (180 degrees)
    int DA = getDistanceAtAngle(0);    // Distance on the left (180 degrees)
  }
  else if (ir_rec == 0xFF9867) { //Key number 2
    DEBUG_PRINTLN(F("IR Command: Key: 2"));
    max_valid_distance = 2000;
    
  }
  else if (ir_rec == 0xFFB04F) { //Key number 3
    max_valid_distance = 3000;
  }
  else if (ir_rec == 0xFF10EF) { //Key number 7
    DEBUG_PRINTLN(F("IR Command: Key: 7"));
    disableMotorsDuringDetection = 1;
  }
  else if (ir_rec == 0xFF38C7) { //Key number 8
    DEBUG_PRINTLN(F("IR Command: Key: 8"));
    readUltrasonicDistance();
  }
  else if (ir_rec == 0xFF5AA5) { //Key number 9
    DEBUG_PRINTLN(F("IR Command: Key: 9"));
    movementEnabled = true;
  }
  else if (ir_rec == 0xFF42BD) { //Key number *
    DEBUG_PRINTLN(F("IR Command: Key: *"));
    DEBUG_PRINTLN(F("IR Command: Enabling Automation"));
    printMessageF(F("Enablng Automation"));
    automationEnabled = true;

  }
  else if (ir_rec == 0xFF52AD) { //Key number #
    DEBUG_PRINTLN(F("IR Command: Key: #"));
    DEBUG_PRINTLN(F("IR Command: Disabling Automation"));
    printMessageF(F("Disablng Automation"));
    automationEnabled = false;
    // Kick the watchdog timer
    //wdt_reset();
    delay(SETTLING_DELAY);

  }
  else if (ir_rec == 0x0 || ir_rec == 0xFFFFFF) { 

  }
  else 
  {
    DEBUG_PRINT(F("IR Command: Unknown Command: "));
    DEBUG_PRINTLN(ir_rec);
  }
  ir_rec = 0;  // Clear the last command
}

void printMessageR(char * str)
{
  #ifdef OLED_ENABLED
  u8g2.clearBuffer();	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR08_tr);
  u8g2.setCursor(4, 28);
  u8g2.print(str);
  u8g2.sendBuffer();
  #endif
}


void printMessageF(const __FlashStringHelper* str)
{
  #ifdef OLED_ENABLED
  u8g2.clearBuffer();	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.setCursor(4, 28);
  u8g2.print(str);
  u8g2.sendBuffer();
  #endif
}



void programCallback(Timer* timer)
{
  int action = timer->getAction();
    //  
    // DEBUG_PRINTLN(F("Main Timer Callback"));
    // #endif
   /* check if the timer is one we expect */
  if(MICRO_DETECT_ACTION == action)
  {
    // DEBUG_PRINT_VERBOSE(F("MICRO_DETECT_ACTION"));
    // if(isObstacleDetectedByMicroSwitches())
    //   reverseAwayFromObstacle();
  }
  else if(LASER_DETECT_ACTION == action){
    // DEBUG_PRINT_VERBOSE(F("LASER_DETECT_ACTION"));
    // laserDetect();
  }
  else if(IR_RECEIVE_ACTION == action){   
    DEBUG_PRINT_VERBOSE(F("IR_RECEIVE_ACTION"));
    processIR();
  }
  else if(MOVEMENT_ACTION == action)
  {
    // DEBUG_PRINT_VERBOSE(F("MOVEMENT_ACTION"));
    // if(automationEnabled)
    //   makeMovementDecision();
  }

}





void programOne(int microSwitchDetectInterval, int laserDetectInterval, int irReceiveInterval, int movementDetectInterval)
{
     
    DEBUG_PRINTLN(F("Initialising timers"));

    if(microSwitchDetectTimer)
        mainTimer->deleteTimer(microSwitchDetectTimer);
    microSwitchDetectTimer = mainTimer->setInterval(MICRO_DETECT_ACTION, microSwitchDetectInterval); 
    #ifdef LASER_DETECTION
    if(laserDetectTimer)
        mainTimer->deleteTimer(laserDetectTimer);
    laserDetectTimer = mainTimer->setInterval(LASER_DETECT_ACTION, laserDetectInterval);  
    #endif
    if(irReceiveTimer)
      mainTimer->deleteTimer(irReceiveTimer);
    irReceiveTimer = mainTimer->setInterval(IR_RECEIVE_ACTION, irReceiveInterval);
    if(MovementTimer)
      mainTimer->deleteTimer(MovementTimer);
    MovementTimer = mainTimer->setInterval(MOVEMENT_ACTION, movementDetectInterval);  
}







int read_microswitch_left()
{
  return digitalRead(LEFT_MICRO_SWITCH);
}

int read_microswitch_right()
{
  return digitalRead(RIGHT_MICRO_SWITCH);
}

bool isObstacleDetectedByMicroSwitches() {
  if((digitalRead(LEFT_MICRO_SWITCH) == 0 || digitalRead(RIGHT_MICRO_SWITCH) == 0))
  {
      DEBUG_PRINT(F("Objected Detected Micro Switches "));
      printMessageF(F("Objected Detected Micro Switches"));
      return true;
  }

    return false;
}



// Average readings to get a reliable distance
float readUltrasonicDistance() {
  const int readings = 5;
  float total = 0;
  int validReadings = 0;
  int invalidReadings = 0;
  if(disableMotorsDuringDetection)
  {
    pauseMotors();
    UltraSonicServo.detach();
  }
  DEBUG_PRINT(F("Reading Ultrasonic Sensors: "));
  for (int i = 0; i < readings; i++) {
    digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRA_SONIC_TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);

    float temp_duration = pulseIn(ULTRA_SONIC_ECHO_PIN, HIGH);
    float temp_distance = (temp_duration * 0.0343) / 2;

    if (temp_distance > 0 && temp_distance <= max_valid_distance) {
      total += temp_distance;
      validReadings++;
    }
    // Kick the watchdog timer
    //wdt_reset();
    delay(10);
  }
  if(  disableMotorsDuringDetection)
  {
    UltraSonicServo.attach(ULTRA_SONIC_SERVO);
    resumeMotors();
  }
  invalidReadings = readings - validReadings;
  if(invalidReadings > 0)
    printMessageF(F("Invalid"));
  DEBUG_PRINT("Invalid Readings: ");
  DEBUG_PRINTLN(invalidReadings);
  DEBUG_PRINT("Distance: ");
  DEBUG_PRINTLN(total / validReadings);
  return validReadings > 0 ? total / validReadings : 0;
}



void moveServo(int angle)
{
    UltraSonicServo.write(angle);
    // Kick the watchdog timer
    //wdt_reset();
    delay(SETTLING_DELAY);
}

int getDistanceAtAngle(int angle) {

  if(angle == 180)
  {
    moveServo(LEFT);
  }
  else if(angle == 0)
   moveServo(RIGHT);
  else
     moveServo(MIDDLE);
  // Kick the watchdog timer
  //wdt_reset();
  delay(SETTLING_DELAY);
  int distance = readUltrasonicDistance();
  DEBUG_PRINT(F("Scanning Obstacles on: ")); 
  if (angle == 0) {
    DEBUG_PRINT(F("Right"));
  } else if (angle == 180) {
    DEBUG_PRINT(F("Left"));
  } else if (angle == 90) {
    DEBUG_PRINT(F("Middle"));
  } else {
    DEBUG_PRINT(angle);  // Print the actual angle if it's not 0, 90, or 180
  }
  DEBUG_PRINT(F(", distance: "));
  DEBUG_PRINTLN(distance);
  return distance;
}

void makeMovementDecision() {
    // Read sensor values
    int DM = getDistanceAtAngle(90);   // Distance in the middle (90 degrees)
    if(DM == 0)
    {  
        DEBUG_PRINT(F("0 distance:"));
        DEBUG_PRINTLN(DM);
        printMessageF(F("0 Middle Distance"));
    }


    // Prioritize moving forward if the middle path is clear and sides are clear
    if ((DM >= SAFE_DISTANCE ) || (DM == 0)) {
        printMessageF(F("Clear Ahead"));
        DEBUG_PRINTLN(F("Clear ahead"));
        forward();  // Continue moving forward if no object is detected ahead and sides are clear
        return;     // Exit the function to avoid unnecessary checks
    } else {
        printMessageF(F("Object Middle"));
        DEBUG_PRINT(F("Object Middle:"));
        DEBUG_PRINT(DM);
    }

    // Stop movement and check surroundings if the middle path is blocked
    stopMotors();
    
    int DR = getDistanceAtAngle(180);  // Distance on the right (0 degrees)
    int DL = getDistanceAtAngle(0);    // Distance on the left (180 degrees)

    // Decide on direction based on side distances
    if (DL > DR) {
        printMessageF(F("Left is clear"));
        DEBUG_PRINT(F("Left is Clear"));
        rotateLeft();  // Prefer rotating left if the left side is clear
    } else if (DR > DL) {
        printMessageF(F("Right is clear"));
        DEBUG_PRINT(F("Right is Clear"));
        rotateRight();  // Prefer rotating right if the right side is clear
    } else {
        // If surrounded or no clear side, move backward and take a random turn
        printMessageF(F("Surrounded"));   
        DEBUG_PRINT(F("Surrounded - backwards and random turn"));
        backward();     // If surrounded, move backward
        randomTurn();   // Take a random turn to reassess the situation
    }
}



void randomTurn() {
  if (random(1, 10) > 5) {  
    DEBUG_PRINTLN("Choosing random left turn.");
    printMessageF(F("Random Left"));
    rotateLeft();
  } else {  
    DEBUG_PRINTLN(F("Choosing random right turn."));
    printMessageF(F("Random Right"));
    rotateRight();
  }
  // Kick the watchdog timer
  //wdt_reset();
  delay(TURN_DELAY/2); // make the turn small
  stopMotors();
}

// Movement Control Functions
void forward() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, motor_speed);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, motor_speed); 
    DEBUG_PRINTLN(F("Moving forward..."));
    lastKnownDirection = DIRECTION_FORWARD;
    printMessageF(F("Moving Forward"));
  }
  else
  {
    DEBUG_PRINTLN(F("Movement Disabled."));
  }
}

void backward() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_SPEED);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_SPEED); 
    DEBUG_PRINTLN(F("Moving backward..."));
    lastKnownDirection = DIRECTION_BACKWARD;
  }
}

void turnLeft() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_LOW);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);   
    DEBUG_PRINTLN(F("Rotating left..."));
      // Kick the watchdog timer
    //wdt_reset();
    delay(TURN_DELAY);
    stopMotors();
  }
}

void turnRight() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_LOW);  
    DEBUG_PRINTLN(F("Rotating right..."));
    // Kick the watchdog timer
    //wdt_reset();
    delay(TURN_DELAY);
    stopMotors();
  }
}



void rotateLeft() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH); 
    DEBUG_PRINTLN(F("Rotating left..."));
    // Kick the watchdog timer
    //wdt_reset();
    delay(TURN_DELAY);
    stopMotors();
  }
}

void rotateRight() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    DEBUG_PRINTLN(F("Rotating right..."));
    // Kick the watchdog timer
    //wdt_reset();
    delay(TURN_DELAY);
    stopMotors();
  }
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0); 
    DEBUG_PRINTLN(F("Stopping"));
    lastKnownDirection = DIRECTION_STOP;
}

void pauseMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    DEBUG_PRINTLN(F("Pausing Motors"));

}

void resumeMotors() {
  if(lastKnownDirection == DIRECTION_FORWARD)
  {
    DEBUG_PRINTLN(F("Resuming Forward"));
    forward(); // Resume forward movement after a stop
  }
  else if(lastKnownDirection == DIRECTION_BACKWARD)
  {
    DEBUG_PRINTLN(F("Resuming Backward"));
    backward(); // Resume forward movement after a stop
  }
  else
  {
    
    DEBUG_PRINTLN(F("Resuming ... Stop"));

    stopMotors();
  }
}


void reverseAwayFromObstacle()
{
    int choice = random(3); // Generate a random number between 0 and 2
    
    if (choice == 0) {
        rotateRight();
    } else if (choice == 1) {
        rotateLeft();
    } else {
        backward();
    }    
    delay(TURN_DELAY * 4);
    stopMotors();
}


void setupLaserSensor()
{
#ifdef LASER_DETECTION
  if (! laserSensor.begin()) {
    DEBUG_PRINTLN(F("Failed to find sensor"));
    printMessageF(F("Laser failed"));
    while (1);
  }
  DEBUG_PRINTLN("Sensor found!");
  printMessageF(F("Laser ok"));
#endif // LASER_DETECTION
}


void showLaserSensorError(uint8_t status)
{
  #ifdef LASER_DETECTION
    // Some error occurred, print it out!
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
      
      DEBUG_PRINTLN(F("System error"));

    }
    else if (status == VL6180X_ERROR_ECEFAIL) {
      
      DEBUG_PRINTLN(F("ECE failure"));

    }
    else if (status == VL6180X_ERROR_NOCONVERGE) {
      
      DEBUG_PRINTLN(F("No convergence"));

    }
    else if (status == VL6180X_ERROR_RANGEIGNORE) {
      
      DEBUG_PRINTLN(F("Ignoring range"));

    }
    else if (status == VL6180X_ERROR_SNR) {
      
      DEBUG_PRINTLN(F("Signal/Noise error"));

    }
    else if (status == VL6180X_ERROR_RAWUFLOW) {
      
      DEBUG_PRINTLN(F("Raw reading underflow"));

    }
    else if (status == VL6180X_ERROR_RAWOFLOW) {
      
      DEBUG_PRINTLN(F("Raw reading overflow"));

    }
    else if (status == VL6180X_ERROR_RANGEUFLOW) {
      
      DEBUG_PRINTLN(F("Range reading underflow"));
  
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW) {
      
      DEBUG_PRINTLN(F("Range reading overflow"));

    }
    #endif // LASER_DETECTION
}
void laserDetect()
{
  #ifdef LASER_DETECTION
    float lux = laserSensor.readLux(VL6180X_ALS_GAIN_5);
    if(lux == -1)
    {
      
       DEBUG_PRINTLN(F("Error occured reading lux level from VL6180X!"));

      printMessageF(F("Laser Lux Error"));
      return;
    }
    
    DEBUG_PRINT(F("Lux: ")); DEBUG_PRINTLN(lux);

    uint8_t range = laserSensor.readRange();
    uint8_t status = laserSensor.readRangeStatus();
    if(range == 255 )
    {
      
       DEBUG_PRINTLN(F("Error occured reading range from VL6180X!"));

      printMessageF(F("Laser Range Error"));
      return;
    }
    if (status == VL6180X_ERROR_NONE)
    {
      
      DEBUG_PRINT(F("Range: ")); DEBUG_PRINTLN(range);

      printMessageF(F("Laser in Range"));
      if ( range < 15 )
      {
        printMessageF(F("Laser Collision"));
        reverseAwayFromObstacle();
        randomTurn();
      }
      else
      {
        printMessageF(F("Laser in Range"));
        stopMotors();
      }
      return;


    }
    else
    {
      
      DEBUG_PRINTLN(F("Error obtaining Range: ")); 

      showLaserSensorError(status);
      return;
    }


    #endif // LASER_DETECTION
}

void setup() {
  #ifdef OLED_ENABLED
  u8g2.begin();
  #endif
  irrecv.enableIRIn();
  pinMode(ULTRA_SONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MICRO_SWITCH, INPUT_PULLUP); // Enable the internal pull-up resistor
  pinMode(RIGHT_MICRO_SWITCH, INPUT_PULLUP); // Enable the internal pull-up resistor
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  UltraSonicServo.attach(ULTRA_SONIC_SERVO);  // Attach the servo at startup
  UltraSonicServo.write(90); // Center position
  delay(SETTLING_DELAY);
  Serial.begin(9600);
  printMessageF(F("Snoop Dog"));
   
  DEBUG_PRINTLN(F("Initialising Watchdog"));

  //wdt_enable(WDTO_8S); // Set an 8 second watchdog 
  mainTimer = StensTimer::getInstance(); // the order of these next three instructions is important, set the static callback before setting timer intervals
  mainTimer->setStaticCallback(programCallback);
  programOne(100,100,200,2000);
  #ifdef LASER_DETECTION
  setupLaserSensor();
  #endif
					// transfer internal memory to the display
}


void loop() {
  // Ensure the main timer processes all scheduled tasks
  mainTimer->run();
  // 
  // DEBUG_PRINTLN(F("Checking Watchdog..."));
  // #endif
  // Kick the watchdog timer
  ////wdt_reset();
  // Adding a small delay to avoid continuous loop iteration without processing delays
  delay(10);


}





