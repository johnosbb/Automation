
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>



#define DEBUG

// Circular buffer for values
constexpr int num_hours = 3;
int8_t temperature_vals [num_hours] = {0};
int8_t current_vals [num_hours] = {0};
int8_t rpm_vals [num_hours] = {0};
int8_t vibrartion_vals [num_hours] = {0};
int cur_idx = 0;



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

  #ifdef DEBUG
  // Check if the change exceeds the threshold
  if (deltaX > threshold) {
    Serial.println("Vibration detected!");
  } else {
    Serial.println("No significant vibration.");
  }
  #endif

  #ifdef DEBUG
  // Output the current acceleration and delta for debugging
  Serial.print("X-Acceleration: ");
  Serial.print(xReading);
  Serial.print(" g, Delta: ");
  Serial.println(deltaX);
  #endif
  return deltaX;
}


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

float read_current()
{
  // Read the analog value from the sensor
  int sensorValue = analogRead(analogInPin);
#ifdef DEBUG
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
#endif

   // Map the sensorValue to the range 0 to 15 amps
  float current = map(sensorValue, 5, 1021, 0, 15000) / 1000.0; // Convert milliamps to amps

#ifdef DEBUG
  // Print the current to the Serial Monitor
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" A");
#endif
  return current;
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
#ifdef DEBUG
  // Print the results
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");
#endif
  // Reading from ADXL345
  sensors_event_t event;
  accel.getEvent(&event);
  // Call the measure_vibration function and pass the event
  float deltaX = measure_vibration(event);
#ifdef DEBUG
  Serial.print("Vibration: ");
  Serial.println(deltaX);
#endif
  float rpm = getRPM();
#ifdef DEBUG  
  Serial.print("RPM: ");
  Serial.println(rpm);
#endif
  float current = read_current();

  delay(10000);  // Wait for 1 second before the next loop
  Serial.println("I2C Scanner starting...");
  scanI2CDevices(100000); // Scan at 100kHz

  delay(1000);

  scanI2CDevices(400000); // Scan at 400kHz (if supported by the device)
}

