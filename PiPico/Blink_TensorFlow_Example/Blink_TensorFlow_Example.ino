
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>




// Create an instance of the SHT4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
// Create an instance of the Adafruit_ADXL34 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Define the pins for RP2040
const int switchPin = 2;         // GPIO2 for the switch
const int analogInPin = 26;      // GPIO26 for analog input (ADC)

// Sensitivity value for ACS712-05B (±5A version)
const float sensitivity = 0.185; // V/A for 5A version

// Voltage divider scaling factor
const float scale = 4.3;  // Scaling factor for 330kΩ / 100kΩ divider

// Define the mean and standard deviation for RPM
const float meanRPM = 1600.0;
const float stdDevRPM = 200.0;

float threshold = 0.1;  // Vibration threshold in g
float baseReading = 0.0;
bool firstRun = true;

float generateRandomRPM(float mean, float stddev) {
  float u1 = random(0, 10000) / 10000.0;
  float u2 = random(0, 10000) / 10000.0;
  
  // Box-Muller transform to generate normally distributed values
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
  float rpm = mean + z0 * stddev;

  return rpm;
}

float getRPM() {
  int switchState = digitalRead(switchPin);

  if (switchState == HIGH) {
    // Generate an RPM below 1500
    return generateRandomRPM(1400, 50); // Centered around 1400 with a smaller deviation
  } else {
    // Generate an RPM centered around 1600 with std dev of 200
    return generateRandomRPM(meanRPM, stdDevRPM);
  }
}

// We will measure vibration in just the X-axis for the purpose of illustration
float measure_vibration(sensors_event_t event)
{
    // Get the X-axis acceleration in g
  float xReading = event.acceleration.x;

  // On first run, set the base reading
  if (firstRun) {
    baseReading = xReading;
    firstRun = false;
  }

  // Calculate the change in acceleration from the base reading
  float deltaX = abs(xReading - baseReading);

  // Check if the change exceeds the threshold
  if (deltaX > threshold) {
    Serial.println("Vibration detected!");
  } else {
    Serial.println("No significant vibration.");
  }

  // Output the current acceleration and delta for debugging
  Serial.print("X-Acceleration: ");
  Serial.print(xReading);
  Serial.print(" g, Delta: ");
  Serial.println(deltaX);
  return deltaX;
}

void setup() {
  Wire.begin(); // Use default I2C pins (GPIO 4 -> SDA, GPIO 5 -> SCL)
  
  pinMode(switchPin, INPUT);
  
  // Initialize serial communication
  Serial.begin(115200);
  while(!Serial);

  Serial.print("Initialization completed\n");

  // Initialize the I2C communication
  if (!sht4.begin()) {
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

void read_current()
{
  // Read the analog value from the sensor
  int sensorValue = analogRead(analogInPin);

  // Convert the analog value to voltage (account for scaling)
  float voltage = (sensorValue * (3.3 / 4095.0)) * scale; // 12-bit ADC

  // Convert the voltage to current
  float current = (voltage - 1.65) / sensitivity; // Center voltage is 1.65V for 3.3V supply

  // Print the current to the Serial Monitor
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" A");
}

void show_accelerator_data(sensors_event_t event)
{
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print(" m/s^2 ");
  Serial.println();
}

// The loop function runs over and over again forever
void loop() {
  sensors_event_t humidity, temp;

  // Perform the measurement
  sht4.getEvent(&humidity, &temp);

  // Print the results
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  // Reading from ADXL345
  sensors_event_t event;
  accel.getEvent(&event);
  // Call the measure_vibration function and pass the event
  float deltaX = measure_vibration(event);
  Serial.print("Vibration: ");
  Serial.println(deltaX);

  float rpm = getRPM();
  Serial.print("RPM: ");
  Serial.println(rpm);

  read_current();

  delay(1000);  // Wait for 1 second before the next loop
}

