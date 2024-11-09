#include <Servo.h>
#include <IRremote.h>
// #include <Arduino.h>
#include <U8g2lib.h> // https://github.com/olikraus/u8g2/blob/master/doc/faq.txt#L167 how to reduce memory

//#define SERIAL_DEBUG // Uncomment this line to enable serial debugging

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);  // assumes I2C


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
#define SETTLING_DELAY 1000
#define DISABLE_MOTORS_DURING_DETECTION 1
#define MAIN_LOOP_DELAY_DEFAULT 1000
#define TURN_DELAY 500
#define RECOVERY_TIME 3000
#define ULTRA_SONIC_TRIGGER_PIN 12
#define ULTRA_SONIC_ECHO_PIN 13

#define SAFE_DISTANCE 30  // Example threshold in cm


float max_valid_distance = 400.0;
bool movementEnabled = true;
bool automationEnabled = true;
int main_loop_delay = MAIN_LOOP_DELAY_DEFAULT;
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
    Serial.println("IR Command: Enabling Automation");
    #endif
    printMessageF(F("Enablng Automation"));
    automationEnabled = true;

  }
  else if (ir_rec == 0xFF52AD) { //Key number #
    #ifdef SERIAL_DEBUG
    Serial.println("IR Command: Key: #");
    Serial.println("IR Command: Disabling Automation");
    #endif
    printMessageF(F("Disablng Automation"));
    automationEnabled = false;
    delay(SETTLING_DELAY);

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
  myservo.attach(ULTRA_SONIC_SERVO);  // Attach the servo at startup
  myservo.write(90); // Center position
  delay(SETTLING_DELAY);
  Serial.begin(9600);
  printMessageF(F("Snoop Dog"));
					// transfer internal memory to the display
}


int read_ir_sensor_left()
{
  return digitalRead(LEFT_IR_SENSOR);
}

int read_ir_sensor_right()
{
  return digitalRead(RIGHT_IR_SENSOR);
}

bool isObstacleDetectedByIR() {
  if((digitalRead(LEFT_IR_SENSOR) == 0 || digitalRead(RIGHT_IR_SENSOR) == 0))
  {
        printMessageF(F("Objected Detected IR"));
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
    // Read sensor values
    int DR = getDistanceAtAngle(180);    // Distance on the right (0 degrees)
    int DM = getDistanceAtAngle(90);   // Distance in the middle (90 degrees)
    int DL = getDistanceAtAngle(0);  // Distance on the left (180 degrees)
    int IRL = read_ir_sensor_left();   // IR sensor on the left (0 = object detected)
    int IRR = read_ir_sensor_right();  // IR sensor on the right (0 = object detected)

    // Check if the middle path is blocked
    if (DM < SAFE_DISTANCE) {
        stopMotors();  // Stop if an object is directly ahead
        printMessageF(F("Object in M"));

        // Decide direction based on side distances and IR sensors
        if (DL > DR && IRL == 1) {
            printMessageF(F("Left is clear"));
            rotateLeft();  // Prefer rotating left if left side is clear
        } else if (DR > DL && IRR == 1) {
            printMessageF(F("Right is clear"));
            rotateRight();  // Prefer rotating right if right side is clear
        } else if (IRL == 0 && IRR == 1) {
            printMessageF(F("IR: Object on L"));
            rotateRight();  // Object detected on the left, rotate right
        } else if (IRR == 0 && IRL == 1) {
            printMessageF(F("IR: Object on R"));
            rotateLeft();   // Object detected on the right, rotate left
        } else {
            printMessageF(F("Surrounded"));
            backward();     // If surrounded by obstacles, move backward
            randomTurn();   // Take a random turn to reassess
        }
    } else { // middle was clear so checking sides
        // If the middle path is clear, decide based on side sensors
        if (DL < SAFE_DISTANCE && DR < SAFE_DISTANCE) {
            printMessageF(F("Middle Clear")); 
            forward();  // Both sides are close but middle is clear, move forward cautiously
        } else if (DL < SAFE_DISTANCE) {
          printMessageF(F("Object on L")); 
            turnRight();  // Object on the left, turn right
        } else if (DR < SAFE_DISTANCE) {
          printMessageF(F("Object on R")); 
            turnLeft();   // Object on the right, turn left
        } else {
            printMessageF(F("Clear Ahead")); 
            forward();    // Clear path, move forward
        }
    }
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
    delay(TURN_DELAY);
    stopMotors();
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
    Serial.println("Rotating left...");
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
    Serial.println("Rotating right...");
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
    Serial.println("Stopping...");
    #endif
}

void resumeMotors() {
  forward(); // Resume forward movement after a stop
}

void reverseAwayFromObstacle()
{
    backward();
    delay(TURN_DELAY);
    stopMotors();
}


void loop() {
  if(isObstacleDetectedByIR())
  {
    reverseAwayFromObstacle();
  }
  processIR();
  //moveServo(MIDDLE);
  if(automationEnabled)
    makeMovementDecision();
  processIR();
  if(isObstacleDetectedByIR())
  {
    reverseAwayFromObstacle();
  }
  delay(MAIN_LOOP_DELAY_DEFAULT); // this is the amount we travel for
  if (Serial.available())
    {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {


      }
    }

}





