#include <Servo.h>
#include <IRremote.h>
#include <StensTimer.h>
// #include <Arduino.h>
#include <U8g2lib.h> // https://github.com/olikraus/u8g2/blob/master/doc/faq.txt#L167 how to reduce memory
#include <Adafruit_GFX.h>
#include "Adafruit_VL6180X.h"


//#define SERIAL_DEBUG // Uncomment this line to enable serial debugging


Adafruit_VL6180X vl = Adafruit_VL6180X();
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);  // assumes hardware I2C
// Use software I²C directly with U8G2
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/8, /* data=*/9, /* reset=*/U8X8_PIN_NONE);

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
#define DISABLE_MOTORS_DURING_DETECTION 1
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
bool movementEnabled = true;
bool automationEnabled = true;
int motor_speed = MOTOR_SPEED;
char bluetooth_data;
Servo UltraSonicServo;
IRrecv irrecv(3);
int lastKnownDirection = DIRECTION_STOP;


/* Timer Section */
StensTimer* mainTimer;
Timer* microSwitchDetectTimer = NULL;
Timer* laserDetectTimer = NULL;
Timer* irReceiveTimer = NULL;
Timer* MovementTimer = NULL;
#define MIRCO_DETECT_ACTION 1
#define IR_RECEIVE_ACTION 2
#define MOVEMENT_ACTION 3
#define LASER_DETECT_ACTION 4




void processIR()
{
  long ir_rec = 0;
  decode_results results;
  #ifdef SERIAL_DEBUG
  Serial.println("Checking IR Commands");
  #endif
  if (irrecv.decode(&results)) {    
    ir_rec = results.value;
    #ifdef SERIAL_DEBUG
    Serial.print(F("Received IR code: 0x"));
    Serial.println(ir_rec, HEX);  // Print the received code in hex format
    #endif
    irrecv.resume(); // Receive the next value
  } 
  else
    return;

  // Check which code was received and execute the corresponding function
  if (ir_rec == 0xFF629D) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Move Forward"));
    #endif
    forward();
  }
  else if (ir_rec == 0xFFA857) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Move Backward"));
    #endif
    backward();
  }
  else if (ir_rec == 0xFF22DD) { // left arrow
  #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Turn Left"));
    #endif
    turnLeft();
  }
  else if (ir_rec == 0xFFC23D) { // right arrow
  #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Turn Right"));
    #endif
    turnRight();
  }
  else if (ir_rec == 0xFF30CF) { // key 4
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Rotate Left"));
    #endif
    rotateLeft();
  }
  else if (ir_rec == 0xFF7A85) { // key 6
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Rotate Right"));
    #endif
    rotateRight();
  }
  else if (ir_rec == 0xFF02FD) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Stop"));
    #endif
    stopMotors();
    

  }
  else if (ir_rec == 0xFF6897) { //Key number 1
    max_valid_distance = 1000;
  }
  else if (ir_rec == 0xFF9867) { //Key number 2
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Key: 2"));
    #endif
    max_valid_distance = 2000;
    
  }
  else if (ir_rec == 0xFFB04F) { //Key number 3
    max_valid_distance = 3000;
  }
  else if (ir_rec == 0xFF42BD) { //Key number *
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Key: *"));
    Serial.println(F("IR Command: Enabling Automation"));
    #endif
    printMessageF(F("Enablng Automation"));
    automationEnabled = true;

  }
  else if (ir_rec == 0xFF52AD) { //Key number #
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR Command: Key: #"));
    Serial.println(F("IR Command: Disabling Automation"));
    #endif
    printMessageF(F("Disablng Automation"));
    automationEnabled = false;
    delay(SETTLING_DELAY);

  }
  else if (ir_rec == 0x0 || ir_rec == 0xFFFFFF) { //Key number #

  }
  else 
  {
    #ifdef SERIAL_DEBUG
    Serial.print(F("IR Command: Unknown Command: "));
    Serial.println(ir_rec, HEX);
    #endif
  }
  ir_rec = 0;  // Clear the last command
}

void printMessageR(char * str)
{
  u8g2.clearBuffer();	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR08_tr);
  u8g2.setCursor(4, 28);
  u8g2.print(str);
  u8g2.sendBuffer();
}


void printMessageF(const __FlashStringHelper* str)
{
  u8g2.clearBuffer();	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.setCursor(4, 28);
  u8g2.print(str);
  u8g2.sendBuffer();
}



void programCallback(Timer* timer)
{
  int action = timer->getAction();
    // #ifdef SERIAL_DEBUG 
    // Serial.println(F("Main Timer Callback"));
    // #endif
   /* check if the timer is one we expect */
  if(MIRCO_DETECT_ACTION == action)
  {
    #ifdef SERIAL_DEBUG
    Serial.println(F("MIRCO_DETECT_ACTION"));
    #endif
    if(isObstacleDetectedByMicroSwitches())
      reverseAwayFromObstacle();
  }
  else if(LASER_DETECT_ACTION == action){
    #ifdef SERIAL_DEBUG
    Serial.println(F("LASER_DETECT_ACTION"));
    #endif
    laserDetect();
  }
  else if(IR_RECEIVE_ACTION == action){
    #ifdef SERIAL_DEBUG
    Serial.println(F("IR_RECEIVE_ACTION"));
    #endif
    processIR();
  }
  else if(MOVEMENT_ACTION == action)
  {
    #ifdef SERIAL_DEBUG 
    Serial.println(F("MOVEMENT_ACTION"));
    #endif
    if(automationEnabled)
      makeMovementDecision();
  }

}





void programOne(int microSwitchDetect, int irReceive, int movementDetect)
{
    #ifdef SERIAL_DEBUG 
    Serial.println(F("Initialising timers"));
    #endif
    if(microSwitchDetectTimer)
        mainTimer->deleteTimer(microSwitchDetectTimer);
    microSwitchDetectTimer = mainTimer->setInterval(MIRCO_DETECT_ACTION, microSwitchDetect); 
    if(laserDetectTimer)
        mainTimer->deleteTimer(laserDetectTimer);
    laserDetectTimer = mainTimer->setInterval(LASER_DETECT_ACTION, laserDetect);  
    if(irReceiveTimer)
      mainTimer->deleteTimer(irReceiveTimer);
    irReceiveTimer = mainTimer->setInterval(IR_RECEIVE_ACTION, irReceive);
    if(MovementTimer)
      mainTimer->deleteTimer(MovementTimer);
    MovementTimer = mainTimer->setInterval(MOVEMENT_ACTION, movementDetect);  
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
      #ifdef SERIAL_DEBUG
      Serial.print(F("Objected Detected Micro Switches "));
      #endif
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
  if(DISABLE_MOTORS_DURING_DETECTION)
  {
    pauseMotors();
    UltraSonicServo.detach();
  }

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
    delay(10);
  }
  if(  DISABLE_MOTORS_DURING_DETECTION)
  {
    UltraSonicServo.attach(ULTRA_SONIC_SERVO);
    resumeMotors();
  }
  return validReadings > 0 ? total / validReadings : 0;
}



void moveServo(int angle)
{
    UltraSonicServo.write(angle);
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
  delay(SETTLING_DELAY);
  int distance = readUltrasonicDistance();
  #ifdef SERIAL_DEBUG
  Serial.print(F("Scanning Obstacles on: ")); 
  if (angle == 0) {
    Serial.print(F("Right"));
  } else if (angle == 180) {
    Serial.print(F("Left"));
  } else if (angle == 90) {
    Serial.print(F("Middle"));
  } else {
    Serial.print(angle);  // Print the actual angle if it's not 0, 90, or 180
  }
  Serial.print(F(", distance: "));
  Serial.println(distance);
  #endif
  return distance;
}

void makeMovementDecision() {
    // Read sensor values
    int DM = getDistanceAtAngle(90);   // Distance in the middle (90 degrees)
    if(DM == 0)
    {
        #ifdef SERIAL_DEBUG
        Serial.print(F("0 distance:"));
        Serial.println(DM);
        #endif
        printMessageF(F("0 Middle Distance"));
    }
    // Assume the sides are clear since IR sensors are removed
    int sidesAreClear = 1;

    // Prioritize moving forward if the middle path is clear and sides are clear
    if ((DM >= SAFE_DISTANCE && sidesAreClear) || (DM == 0)) {
        printMessageF(F("Clear Ahead"));
        #ifdef SERIAL_DEBUG
        Serial.println(F("Clear ahead"));
        #endif
        forward();  // Continue moving forward if no object is detected ahead and sides are clear
        return;     // Exit the function to avoid unnecessary checks
    } else {
        printMessageF(F("Object Middle"));
        #ifdef SERIAL_DEBUG
        Serial.print(F("Object Middle:"));
        Serial.print(DM);
        #endif
    }

    // Stop movement and check surroundings if the middle path is blocked
    stopMotors();
    
    int DR = getDistanceAtAngle(180);  // Distance on the right (0 degrees)
    int DL = getDistanceAtAngle(0);    // Distance on the left (180 degrees)

    // Decide on direction based on side distances
    if (DL > DR) {
        printMessageF(F("Left is clear"));
        #ifdef SERIAL_DEBUG
        Serial.print(F("Left is Clear"));
        #endif
        rotateLeft();  // Prefer rotating left if the left side is clear
    } else if (DR > DL) {
        printMessageF(F("Right is clear"));
        #ifdef SERIAL_DEBUG
        Serial.print(F("Right is Clear"));
        #endif
        rotateRight();  // Prefer rotating right if the right side is clear
    } else {
        // If surrounded or no clear side, move backward and take a random turn
        printMessageF(F("Surrounded"));
        #ifdef SERIAL_DEBUG
        Serial.print(F("Surrounded - backwards and random turn"));
        #endif
        backward();     // If surrounded, move backward
        randomTurn();   // Take a random turn to reassess the situation
    }
}



void randomTurn() {
  if (random(1, 10) > 5) {
    #ifdef SERIAL_DEBUG
    Serial.println("Choosing random left turn.");
    #endif
    printMessageF(F("Random Left"));
    rotateLeft();
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Choosing random right turn."));
    #endif
    printMessageF(F("Random Right"));
    rotateRight();
  }
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
    #ifdef SERIAL_DEBUG
    Serial.println(F("Moving forward..."));
    #endif
    lastKnownDirection = DIRECTION_FORWARD;
  }
}

void backward() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_SPEED);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_SPEED);
    #ifdef SERIAL_DEBUG
    Serial.println(F("Moving backward..."));
    #endif
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
    #ifdef SERIAL_DEBUG
    Serial.println(F("Rotating left..."));
    #endif
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
    #ifdef SERIAL_DEBUG
    Serial.println(F("Rotating right..."));
    #endif
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
    #ifdef SERIAL_DEBUG
    Serial.println(F("Rotating left..."));
    #endif
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
    #ifdef SERIAL_DEBUG
    Serial.println(F("Rotating right..."));
    #endif
    delay(TURN_DELAY);
    stopMotors();
  }
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    #ifdef SERIAL_DEBUG
    Serial.println(F("Stopping"));
    #endif
    lastKnownDirection = DIRECTION_STOP;
}

void pauseMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    #ifdef SERIAL_DEBUG
    Serial.println(F("Pausing Motors"));
    #endif
}

void resumeMotors() {
  if(lastKnownDirection == DIRECTION_FORWARD)
  {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Resuming Forward"));
    #endif
    forward(); // Resume forward movement after a stop
  }
  else if(lastKnownDirection == DIRECTION_BACKWARD)
  {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Resuming Backward"));
    #endif
    backward(); // Resume forward movement after a stop
  }
  else
  {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Resuming ... Stop"));
    #endif
    stopMotors();
  }
}

void reverseAwayFromObstacle()
{
    backward();
    delay(TURN_DELAY*4);
    stopMotors();
}


void setupLaserSensor()
{

  if (! vl.begin()) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Failed to find sensor"));
    #endif
    printMessageF(F("Laser failed"));
    while (1);
  }
  #ifdef SERIAL_DEBUG
  Serial.println("Sensor found!");
  #endif
  printMessageF(F("Laser ok"));
}

void laserDetect()
{
    float lux = vl.readLux(VL6180X_ALS_GAIN_5);
    #ifdef SERIAL_DEBUG
    Serial.print(F("Lux: ")); Serial.println(lux);
    #endif
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
      #ifdef SERIAL_DEBUG
      Serial.print(F("Range: ")); Serial.println(range);
      Serial.println();
      #endif
      printMessageF(F("Laser in Range"));
      if ( range < 15 )
        reverseAwayFromObstacle();
      else
        stopMotors();
      return;


    } else {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Error obtaining Range: ")); 
      #endif
      return;
    }

    // Some error occurred, print it out!
    
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("System error"));
      #endif
    }
    else if (status == VL6180X_ERROR_ECEFAIL) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("ECE failure"));
      #endif
    }
    else if (status == VL6180X_ERROR_NOCONVERGE) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("No convergence"));
      #endif
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Ignoring range"));
      #endif
    }
    else if (status == VL6180X_ERROR_SNR) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Signal/Noise error"));
      #endif
    }
    else if (status == VL6180X_ERROR_RAWUFLOW) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Raw reading underflow"));
      #endif
    }
    else if (status == VL6180X_ERROR_RAWOFLOW) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Raw reading overflow"));
      #endif
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Range reading underflow"));
      #endif
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Range reading overflow"));
      #endif
    }
}

void setup() {
  u8g2.begin();
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
  mainTimer = StensTimer::getInstance(); // the order of these next three instructions is important, set the static callback before setting timer intervals
  mainTimer->setStaticCallback(programCallback);
  programOne(100,200,2000);
  setupLaserSensor();
					// transfer internal memory to the display
}


void loop() {
  // Ensure the main timer processes all scheduled tasks
  mainTimer->run();

  // Adding a small delay to avoid continuous loop iteration without processing delays
  delay(10);

}





