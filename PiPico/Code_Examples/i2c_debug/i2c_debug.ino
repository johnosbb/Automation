#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


MbedI2C myi2c(4,5);
// Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &myi2c);


// Create an instance of the SHT4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
// Create an instance of the Adafruit_ADXL34 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


void scanI2CDevices(long frequency) {
  Serial.print("Scanning at ");
  Serial.print(frequency);
  Serial.println(" Hz");

  Wire.setClock(frequency);

  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}





void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while(!Serial);
  myi2c.begin(); // Use default I2C pins (GPIO 4 -> SDA, GPIO 5 -> SCL)
  Serial.print("Initialization completed\n");

  // Initialize the I2C communication
  if (!sht4.begin(&myi2c)) {
    Serial.println("Couldn't find SHT4x sensor!");
    while (1) delay(10);
  }
  Serial.println("SHT4x Found!");

  // Initialize ADXL345
  if (!accel.begin()) {
    Serial.println("Couldn't find ADXL345 sensor!");
    while (1);
  }
  Serial.println("Found ADXL345 sensor!");

  // Set ADXL345 range to +/- 4G 
  accel.setRange(ADXL345_RANGE_4_G);

  // Set data rate to 800 Hz (adjust based on your application)
  accel.setDataRate(ADXL345_DATARATE_800_HZ);

}

void loop() {
  // put your main code here, to run repeatedly:

}
