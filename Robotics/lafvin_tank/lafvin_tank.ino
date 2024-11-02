#include <Servo.h>
#include <IRremote.h>

#define RIGHT 180
#define MIDDLE 90
#define LEFT 0
#define ANTI_CLOCKWISE LOW
#define CLOCKWISE HIGH
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 4
#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_PWM 6
#define ULTRA_SONIC_SERVO 10
#define MOTOR_SPEED 100
#define MOTOR_TURN_SPEED_HIGH 255
#define MOTOR_TURN_SPEED_LOW 100
#define SETTLING_DELAY 2000
#define DISABLE_MOTORS_DURING_DETECTION 1
#define MAIN_LOOP_DELAY 5000
#define RECOVERY_TIME 3000

const int ultraSonicTriggerPin = 12;
const int ultraSonicEchoPin = 13;
const float max_valid_distance = 400.0;

bool movementEnabled = true;

float distance;
volatile int DL, DM, DR; // Left, Middle, and Right distances
int motor_speed = MOTOR_SPEED;


Servo myservo;

long ir_rec;

IRrecv irrecv(3);
decode_results results;

void processIR()
{
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
  if (ir_rec == 0xFFA857) {
    Serial.println("IR Command: Move Backward");
    backward();
  }
  if (ir_rec == 0xFF22DD) {
    Serial.println("IR Command: Turn Left");
    rotateLeft();
  }
  if (ir_rec == 0xFFC23D) {
    Serial.println("IR Command: Turn Right");
    rotateRight();
  }
  if (ir_rec == 0xFF30CF) {
    Serial.println("IR Command: Rotate Left");
    rotateLeft();
  }
  if (ir_rec == 0xFF7A85) {
    Serial.println("IR Command: Rotate Right");
    rotateRight();
  }
  if (ir_rec == 0xFF02FD) {
    Serial.println("IR Command: Stop");
    stopMotors();

  }
  if (ir_rec == 0xFF6897) {
    Serial.println("IR Command: Disabling Movement");
    movementEnabled = false;
  }
  ir_rec = 0;  // Clear the last command
}

void setup() {
  irrecv.enableIRIn();
  pinMode(ultraSonicTriggerPin, OUTPUT);
  pinMode(ultraSonicEchoPin, INPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  DL = 0;
  DM = 0;
  DR = 0;
  myservo.attach(ULTRA_SONIC_SERVO);  // Attach the servo at startup
  myservo.write(90); // Center position
  delay(SETTLING_DELAY);
  Serial.begin(9600);
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
    digitalWrite(ultraSonicTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultraSonicTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultraSonicTriggerPin, LOW);

    float temp_duration = pulseIn(ultraSonicEchoPin, HIGH);
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
    analogWrite(LEFT_MOTOR_PWM, 200);
    digitalWrite(RIGHT_MOTOR, CLOCKWISE);
    analogWrite(RIGHT_MOTOR_PWM, 200);
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
  delay(MAIN_LOOP_DELAY);
}





