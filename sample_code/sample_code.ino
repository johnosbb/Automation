#include <Arduino.h>
#include <IRremote.h>

#define IR_RECEIVE_PIN 4

IRrecv IRReceiver(IR_RECEIVE_PIN);
decode_results results;

void setup()
{
   Serial.begin(115200);
   IRReceiver.enableIRIn();


}

void loop() {
  Serial.println("Decoding ... main loop");
   if(IRReceiver.decode(&results)) {
      //Serial.println(results.value, HEX);
      Serial.println("Got code ... main loop");
      IRReceiver.resume();
   }
   delay(3000);

}