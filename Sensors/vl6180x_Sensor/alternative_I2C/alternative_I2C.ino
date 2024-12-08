#include <Wire.h>            // For hardware I2C
#include <SoftwareWire.h>    // For software I2C (on custom pins)
#include <Adafruit_VL6180X.h>

// Configuration: Uncomment one of these lines to select the I2C mode
//#define USE_HARDWARE_I2C
#define USE_SOFTWARE_I2C

#ifdef USE_HARDWARE_I2C
Adafruit_VL6180X vl;          // Use default hardware I2C
#else
#define SDA_PIN 8             // SDA for software I2C
#define SCL_PIN 9             // SCL for software I2C
SoftwareWire myWire(SDA_PIN, SCL_PIN);
Adafruit_VL6180X vl = Adafruit_VL6180X(&myWire); // Use software I2C
#endif

void setup() {
  Serial.begin(9600);
  Serial.println("VL6180X test with configurable I2C");

#ifdef USE_HARDWARE_I2C
  Wire.begin();               // Initialize hardware I2C
#else
  myWire.begin();             // Initialize software I2C
#endif

  // Initialize the sensor
  if (!vl.begin()) {
    Serial.println("Failed to initialize sensor!");
    while (1); // Halt execution
  }
  Serial.println("Sensor initialized successfully!");
}

void loop() {
  // Read and print range data
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: ");
    Serial.println(range);
  } else {
    Serial.print("Error: ");
    Serial.println(status);
  }

  delay(500);
}
