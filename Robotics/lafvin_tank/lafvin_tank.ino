#include <Servo.h>
#include <IRremote.h>
// #include <Arduino.h>
#include <U8g2lib.h> // https://github.com/olikraus/u8g2/blob/master/doc/faq.txt#L167 how to reduce memory

//#define SERIAL_DEBUG // Uncomment this line to enable serial debugging

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);  // assumes I2C
//U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0);  // assumes I2C

#define RIGHT 180
#define MIDDLE 90
#define LEFT 0
#define ANTI_CLOCKWISE LOW
#define CLOCKWISE HIGH
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 4
#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_PWM 6
#define LEFT_IR_SENSOR A1
#define RIGHT_IR_SENSOR A1
#define ULTRA_SONIC_SERVO 10
#define MOTOR_SPEED 100
#define MOTOR_TURN_SPEED_HIGH 255
#define MOTOR_TURN_SPEED_LOW 100
#define SETTLING_DELAY 2000
#define DISABLE_MOTORS_DURING_DETECTION 1
#define MAIN_LOOP_DELAY_DEFAULT 1000
#define RECOVERY_TIME 3000
#define ULTRA_SONIC_TRIGGER_PIN 12
#define ULTRA_SONIC_ECHO_PIN 13


float max_valid_distance = 400.0;
bool movementEnabled = true;
float distance;
int main_loop_delay = MAIN_LOOP_DELAY_DEFAULT;
volatile int DL, DM, DR; // Left, Middle, and Right distances
volatile int left_IR_Sensor;
volatile int right_IR_Sensor;
int motor_speed = MOTOR_SPEED;
char bluetooth_data;
Servo myservo;

IRrecv irrecv(3);


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
    Serial.print("Received IR code: 0x");
    Serial.println(ir_rec, HEX);  // Print the received code in hex format
    #endif
    irrecv.resume(); // Receive the next value
  } 

  // Check which code was received and execute the corresponding function
  if (ir_rec == 0xFF629D) {
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Move Forward");
    #endif
    forward();
  }
  else if (ir_rec == 0xFFA857) {
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Move Backward");
    #endif
    backward();
  }
  else if (ir_rec == 0xFF22DD) { // left arrow
  #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Turn Left");
    #endif
    rotateLeft();
  }
  else if (ir_rec == 0xFFC23D) { // right arrow
  #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Turn Right");
    #endif
    rotateRight();
  }
  else if (ir_rec == 0xFF30CF) { // key 4
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Rotate Left");
    #endif
    rotateLeft();
  }
  else if (ir_rec == 0xFF7A85) { // key 6
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Rotate Right");
    #endif
    rotateRight();
  }
  else if (ir_rec == 0xFF02FD) {
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Stop");
    #endif
    stopMotors();
    

  }
  else if (ir_rec == 0xFF6897) { //Key number 1
    max_valid_distance = 1000;
  }
  else if (ir_rec == 0xFF9867) { //Key number 2
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Key: 2");
    #endif
    max_valid_distance = 2000;
    
  }
  else if (ir_rec == 0xFFB04F) { //Key number 3
    max_valid_distance = 3000;
  }
  else if (ir_rec == 0xFFB04F) { //Key number *
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Key: *");
    Serial.println("IR Command: Enabling Movement");
    #endif
    movementEnabled = true;

  }
  else if (ir_rec == 0xFF52AD) { //Key number #
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Key: #");
    Serial.println("IR Command: Disabling Movement");
    #endif
    movementEnabled = false;

  }
  else if (ir_rec == 0x0 || ir_rec == 0xFFFFFF) { //Key number #


  }
  else //  main_loop_delay
  {
    #ifdef SERIAL_DEBUG
    Serial.print("IR Command: Unknown Command: ");
    Serial.println(ir_rec, HEX);
    #endif
  }
  ir_rec = 0;  // Clear the last command
}

void printMessageR(char * str)
{
  u8g2.clearBuffer();	
	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR08_tr);
  u8g2.setCursor(4, 28);
  u8g2.print(str);
  u8g2.sendBuffer();
}


void printMessageF(const __FlashStringHelper* str)
{
  u8g2.clearBuffer();	
	// clear the internal memory
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.setCursor(4, 28);
  //u8g2.print(F("Snoop Dog"));	// write something to the internal memory at cursor location x=8,y=29
  u8g2.print(str);
  u8g2.sendBuffer();
}

void setup() {
  u8g2.begin();
  irrecv.enableIRIn();
  pinMode(ULTRA_SONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  left_IR_Sensor = 0;
  right_IR_Sensor = 0;
  DL = 0;
  DM = 0;
  DR = 0;
  myservo.attach(ULTRA_SONIC_SERVO);  // Attach the servo at startup
  myservo.write(90); // Center position
  delay(SETTLING_DELAY);
  Serial.begin(9600);
  printMessageF(F("Snoop Dog"));
					// transfer internal memory to the display
}


void Infrared_Obstacle_Avoidance() {
    left_IR_Sensor = digitalRead(LEFT_IR_SENSOR);
    right_IR_Sensor = digitalRead(RIGHT_IR_SENSOR);
    if (left_IR_Sensor == 0 && right_IR_Sensor == 0) { // both sensors are active
      printMessageF(F("IR: L & R"));
      stopMotors();
      delay(500);
      backward();
      delay(500);
      if (random(1, 10) > 5) {
        rotateLeft();;
      } else {
        rotateRight();
      }
      delay(500);

    } else if (left_IR_Sensor == 0 && right_IR_Sensor == 1) {
      printMessageF(F("IR: L"));
      backward();
      delay(500);
      rotateRight();
      delay(500);
    } else if (left_IR_Sensor == 1 && right_IR_Sensor == 0) {
      printMessageF(F("IR:  R"));
      backward();
      delay(500);
      rotateLeft();
      delay(500);
    } else {
      //printMessageF(F("IR: Clear"));
      forward();

    }
    if (Serial.available())
    {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {


      }
    }
}



// Average readings to get a reliable distance
float readUltrasonicDistance() {
  const int readings = 5;
  float total = 0;
  int validReadings = 0;
  if(DISABLE_MOTORS_DURING_DETECTION)
  {
    stopMotors();
    myservo.detach();
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
    myservo.attach(ULTRA_SONIC_SERVO);
  }
  return validReadings > 0 ? total / validReadings : 0;
}

void detectObstacleDistances() {
  char buffer[40];
  #ifdef SERIAL_DEBUG
  Serial.print("Scanning obstacle distances. ");
  #endif
  DL = getDistanceAtAngle(180); // Left
  DM = getDistanceAtAngle(90);  // Center
  DR = getDistanceAtAngle(0);  // Right
  sprintf(buffer,"DL=%d,DM=%d,DR=%d",DL,DM,DR);
  printMessageR(buffer);
  int maxDistance = max(DL, max(DM, DR));
  delay(SETTLING_DELAY);
  #ifdef SERIAL_DEBUG
  Serial.print("furthest obstacle is ");
  Serial.println(maxDistance);
  #endif
}



void moveServo(int angle)
{
    myservo.write(angle);
    delay(SETTLING_DELAY);
}

int getDistanceAtAngle(int angle) {
  //stopMotors();
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
  Serial.print("Scanning Obstacles on: "); 
  if (angle == 0) {
    Serial.print("Right");
  } else if (angle == 180) {
    Serial.print("Left");
  } else if (angle == 90) {
    Serial.print("Middle");
  } else {
    Serial.print(angle);  // Print the actual angle if it's not 0, 90, or 180
  }
  Serial.print(", distance: ");
  Serial.println(distance);
  #endif
  //resumeMotors();
  return distance;
}

void makeMovementDecision() {
  if (DM < 20 && DM > 0) {
    #ifdef SERIAL_DEBUG
    Serial.print("Obstacle detected straight ahead at distance: ");
    Serial.println(DM);
    #endif
    printMessageF(F("Object Detected"));
    stopMotors();
    delay(1000);
    if (DL < 50 || DR < 50) {
      chooseTurnDirection();
    } else {
      randomTurn();
    }
  } else {
    forward();
  }
}

void chooseTurnDirection() {
  if (DL > DR) {
    #ifdef SERIAL_DEBUG
    Serial.println("Turning left to avoid obstacle.");
    #endif
    rotateLeft();
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println("Turning right to avoid obstacle.");
    #endif
    rotateRight();
  }
  delay(500);
  forward();
}

void randomTurn() {
  if (random(1, 10) > 5) {
    #ifdef SERIAL_DEBUG
    Serial.println("Choosing random left turn.");
    #endif
    rotateLeft();
  } else {
    #ifdef SERIAL_DEBUG
    Serial.println("Choosing random right turn.");
    #endif
    rotateRight();
  }
  delay(500);
  forward();
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
    Serial.println("Moving forward...");
    #endif
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
    Serial.println("Moving backward...");
    #endif
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
    Serial.println("Rotating left...");
    #endif
  }
}

void turnRight() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_LOW);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    #ifdef SERIAL_DEBUG
    Serial.println("Rotating right...");
    #endif
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
    Serial.println("Rotating left...");
    #endif
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
    Serial.println("Rotating right...");
    #endif
  }
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    #ifdef SERIAL_DEBUG
    Serial.println("Stopping...");
    #endif
}

void resumeMotors() {
  forward(); // Resume forward movement after a stop
}

void loop() {
  processIR();
  moveServo(MIDDLE);
  distance = readUltrasonicDistance();
  while (distance == 0 || distance > max_valid_distance) {
    #ifdef SERIAL_DEBUG
    Serial.println("Invalid reading, trying again...");
    #endif
    distance = readUltrasonicDistance();
    delay(RECOVERY_TIME);
  } 
  #ifdef SERIAL_DEBUG
  Serial.print("Distance straight ahead: ");
  Serial.println(distance);
  #endif
  DM = distance;
  detectObstacleDistances();
  makeMovementDecision();
  processIR();
  Infrared_Obstacle_Avoidance();
  delay(MAIN_LOOP_DELAY_DEFAULT); // this is the amount we travel for
  u8g2.clearBuffer();	
  u8g2.sendBuffer();
}





