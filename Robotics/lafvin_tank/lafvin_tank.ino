#include <Servo.h>
#include <IRremote.h>
// #include <Arduino.h>
#include <U8g2lib.h>



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
  Serial.println("Checking IR Commands");
  if (irrecv.decode(&results)) {    
    ir_rec = results.value;
    Serial.print("Received IR code: 0x");
    Serial.println(ir_rec, HEX);  // Print the received code in hex format
    irrecv.resume(); // Receive the next value
  } 

  // Check which code was received and execute the corresponding function
  if (ir_rec == 0xFF629D) {
    Serial.println("IR Command: Move Forward");
    forward();
  }
  else if (ir_rec == 0xFFA857) {
    Serial.println("IR Command: Move Backward");
    backward();
  }
  else if (ir_rec == 0xFF22DD) { // left arrow
    Serial.println("IR Command: Turn Left");
    rotateLeft();
  }
  else if (ir_rec == 0xFFC23D) { // right arrow
    Serial.println("IR Command: Turn Right");
    rotateRight();
  }
  else if (ir_rec == 0xFF30CF) { // key 4
    Serial.println("IR Command: Rotate Left");
    rotateLeft();
  }
  else if (ir_rec == 0xFF7A85) { // key 6
    Serial.println("IR Command: Rotate Right");
    rotateRight();
  }
  else if (ir_rec == 0xFF02FD) {
    Serial.println("IR Command: Stop");
    stopMotors();

  }
  else if (ir_rec == 0xFF6897) { //Key number 1
    max_valid_distance = 1000;
  }
  else if (ir_rec == 0xFF9867) { //Key number 2
    Serial.println("IR Command: Key: 2");
    max_valid_distance = 2000;
  }
  else if (ir_rec == 0xFFB04F) { //Key number 3
    max_valid_distance = 3000;
  }
  else if (ir_rec == 0xFFB04F) { //Key number *
    Serial.println("IR Command: Key: *");
    Serial.println("IR Command: Enabling Movement");
    movementEnabled = true;

  }
  else if (ir_rec == 0xFF52AD) { //Key number #
    Serial.println("IR Command: Key: #");
    Serial.println("IR Command: Disabling Movement");
    movementEnabled = false;

  }
  else if (ir_rec == 0x0 || ir_rec == 0xFFFFFF) { //Key number #


  }
  else //  main_loop_delay
  {
    Serial.print("IR Command: Unknown Command: ");
    Serial.println(ir_rec, HEX);
  }
  ir_rec = 0;  // Clear the last command
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
					// transfer internal memory to the display
}


void Infrared_Obstacle_Avoidance() {
    left_IR_Sensor = digitalRead(LEFT_IR_SENSOR);
    right_IR_Sensor = digitalRead(RIGHT_IR_SENSOR);
    if (left_IR_Sensor == 0 && right_IR_Sensor == 0) {
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
      backward();
      delay(500);
      rotateRight();
      delay(500);
    } else if (left_IR_Sensor == 1 && right_IR_Sensor == 0) {
      backward();
      delay(500);
      rotateLeft();
      delay(500);
    } else {
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
  Serial.print("Scanning obstacle distances. ");
  DL = getDistanceAtAngle(180); // Left
  DM = getDistanceAtAngle(90);  // Center
  DR = getDistanceAtAngle(0);  // Right
  int maxDistance = max(DL, max(DM, DR));
  Serial.print("furthest obstacle is ");
  Serial.println(maxDistance);
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
  //resumeMotors();
  return distance;
}

void makeMovementDecision() {
  if (DM < 20 && DM > 0) {
    Serial.print("Obstacle detected straight ahead at distance: ");
    Serial.println(DM);
    stopMotors();
    delay(1000);
    detectObstacleDistances();
    if (DL < 50 || DR < 50) {
      chooseTurnDirection();
    } else {
      randomTurn();
    }
  } else {
    Infrared_Obstacle_Avoidance();
    forward();
  }
}

void chooseTurnDirection() {
  if (DL > DR) {
    Serial.println("Turning left to avoid obstacle.");
    rotateLeft();
  } else {
    Serial.println("Turning right to avoid obstacle.");
    rotateRight();
  }
  delay(500);
  forward();
}

void randomTurn() {
  if (random(1, 10) > 5) {
    Serial.println("Choosing random left turn.");
    rotateLeft();
  } else {
    Serial.println("Choosing random right turn.");
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
    Serial.println("Moving forward...");
  }
}

void backward() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_SPEED);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_SPEED);
    Serial.println("Moving backward...");
  }
}

void turnLeft() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_LOW);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    Serial.println("Rotating left...");
  }
}

void turnRight() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_LOW);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    Serial.println("Rotating right...");
  }
}



void rotateLeft() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    Serial.println("Rotating left...");
  }
}

void rotateRight() {
  if(movementEnabled)
  {
    digitalWrite(LEFT_MOTOR, LOW);
    analogWrite(LEFT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, MOTOR_TURN_SPEED_HIGH);
    Serial.println("Rotating right...");
  }
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR, CLOCKWISE);
    analogWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(RIGHT_MOTOR, ANTI_CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    Serial.println("Stopping...");
}

void resumeMotors() {
  forward(); // Resume forward movement after a stop
}

void loop() {
  processIR();
  moveServo(MIDDLE);
  distance = readUltrasonicDistance();
  while (distance == 0 || distance > max_valid_distance) {
    Serial.println("Invalid reading, trying again...");
    distance = readUltrasonicDistance();
    delay(RECOVERY_TIME);
  } 
  Serial.print("Distance straight ahead: ");
  Serial.println(distance);
  DM = distance;
  makeMovementDecision();
  processIR();
  //u8g2.clearBuffer();	
  delay(MAIN_LOOP_DELAY_DEFAULT); // this is the amount we travel for
	// clear the internal memory
  // //u8g2.setFont(u8g2_font_logisoso28_tr);  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  //  u8g2.setFont(u8g2_font_5x7_tr);
  //  u8g2.drawStr(8,29,"Snoop Dog");	// write something to the internal memory at cursor location x=8,y=29
  //  u8g2.sendBuffer();
}





