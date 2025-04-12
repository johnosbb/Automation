/* Arduino code for using HC-020k rotary encoder on Arduino Mega2560
 Author: Revant Adlakha
 Lab: SVL
 Note: For other encoders use Pin 2,3,18,19 for interrupts on Arduino Mega2560
 */

int encoder_pin1=2;
unsigned int rpm;
volatile byte pulses;
unsigned long time_old;
unsigned int pulses_per_turn= 20; // Depends on the number of spokes on the encoder wheel

void count() // Counting the number of pulses for calculation of rpm
{
  pulses++;  
}

void setup() {
  rpm=0;
  pulses=0;
  time_old=0;
  
  Serial.begin(9600);
  pinMode(encoder_pin1,INPUT); // Setting encoder-1 pin as Input
  attachInterrupt(digitalPinToInterrupt(encoder_pin1), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0
 
}

void loop() {
 if(millis()-time_old >=100){ // Updating every 0.1 seconds
  detachInterrupt(digitalPinToInterrupt(encoder_pin1));
  rpm = (60 * 100 / pulses_per_turn )/ (millis() - time_old)* pulses;
  time_old=millis();
  pulses=0;
  Serial.print("RPM= ");
  Serial.println(rpm);
  attachInterrupt(digitalPinToInterrupt(encoder_pin1), count, FALLING ); // Triggering count function everytime the encoder pin1 turns from 1 to 0
  
 }

}