
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


//#define DEBUG
//#define CIRCULAR_BUFFER_DEBUG

#define NUM_HOURS 3
// RPM - Mean: 1603.866, Standard Deviation: 195.843
// Temperature (°C) - Mean: 24.354, Standard Deviation: 4.987
// Vibration (g) - Mean: 0.120, Standard Deviation: 0.020
// Current (A) - Mean: 3.494, Standard Deviation: 0.308

constexpr int num_reads = 3;
constexpr float t_mean  = 24.354f;
constexpr float rpm_mean  = 1603.866;
constexpr float v_mean  = 0.120f;
constexpr float c_mean  = 3.494;
constexpr float t_std   = 4.987f;
constexpr float rpm_std   = 195.843f;
constexpr float v_std   = 0.020f;
constexpr float c_std   = 0.308f;


// Circular buffer for values
constexpr int num_hours = 3;
int8_t temperature_vals [num_hours] = {0};
int8_t current_vals [num_hours] = {0};
int8_t rpm_vals [num_hours] = {0};
int8_t vibration_vals [num_hours] = {0};
int cur_idx = 0;

float   tflu_i_scale      = 1.0f;
int32_t tflu_i_zero_point = 0;

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

// Define the emulated mean and standard deviation for RPM generation
const float meanRPM = 1600.0;
const float stdDevRPM = 200.0;

float threshold = 0.1;  // Vibration threshold in g
float baseReading = 0.0;
bool firstRun = true;

inline int8_t quantize(float x, float scale, float zero_point)
{

    // Compute the quantized value
    float quantized_float = (x / scale) + zero_point;

    // Cast to int8_t
    int8_t quantized_value = static_cast<int8_t>(quantized_float);

    // Print debug information
    Serial.print("Input Value: ");
    Serial.print(x);
    Serial.print(" | Scale: ");
    Serial.print(scale);
    Serial.print(" | Zero Point: ");
    Serial.print(zero_point);
    Serial.print(" | Quantized Float: ");
    Serial.print(quantized_float);
    Serial.print(" | Quantized Int8_t: ");
    Serial.println(quantized_value);

    return quantized_value;
}

float generateRandomRPM(float mean, float stddev) {
  float u1 = random(0, 10000) / 10000.0;
  float u2 = random(0, 10000) / 10000.0;
  
  // Box-Muller transform to generate normally distributed values
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
  float rpm = mean + z0 * stddev;

  return rpm;
}

float readRPM() {
  int switchState = digitalRead(switchPin);
  float rpm;

  if (switchState == HIGH) {
    // Generate an RPM below 1500
    rpm =  generateRandomRPM(1400, 50); // Centered around 1400 with a smaller deviation
  } else {
    // Generate an RPM centered around 1600 with std dev of 200
    rpm =  generateRandomRPM(meanRPM, stdDevRPM);
  }
  #ifdef DEBUG  
    Serial.print("RPM: ");
    Serial.println(rpm);
  #endif
  return rpm;
}

// We will measure vibration in just the X-axis for the purpose of illustration
float readVibration()
{

  sensors_event_t event;
  accel.getEvent(&event);


#ifdef DEBUG
  Serial.print("Vibration: ");
  Serial.println(deltaX);
#endif
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

float readCurrent()
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


#ifdef DEBUG
void show_accelerator_data(sensors_event_t event)
{
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print(" m/s^2 ");
  Serial.println();
}

#endif

float readTemperature()
{
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
  return temp.temperature;
}

#ifdef CIRCULAR_BUFFER_DEBUG
void printCircularBuffer(int8_t* buffer, int cur_idx, int num_hours, const char* var_name) {
  Serial.print(var_name);
  Serial.print(" Circular Buffer: ");

  for (int i = 0; i < num_hours; i++) {
    int idx = (cur_idx + i) % num_hours;
    Serial.print(buffer[idx]);
    if (i < num_hours - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}
#endif

// The loop function runs over and over again forever
void loop() {
  float rpm = 0.0f;
  float temperature = 0.0f;
  float current = 0.0f;
  float vibration = 0.0f;

  for(int i = 0; i < num_reads; ++i) {
    temperature += readTemperature();
    rpm += readRPM();    
    current += readCurrent();
    vibration += readVibration();    
    delay(3000);
  }

  // Take the average
  temperature /= (float)num_reads;
  current /= (float)num_reads;
  rpm /= (float)num_reads;
  vibration /= (float)num_reads;


#ifdef DEBUG

  // Print the new data
  Serial.print("RPM: ");
  Serial.print(rpm);
  Serial.print(" | Temperature: ");
  Serial.print(temperature);
  Serial.print(" C");
  Serial.print(" | Current: ");
  Serial.print(current);
  Serial.print(" A");
  Serial.print(" | Vibration: ");
  Serial.print(vibration);
  Serial.print(" m/s^2\n");

  // Ensure to flush the Serial output
  Serial.flush();
#endif
    // Z-score scaling
  temperature = (temperature - t_mean) / t_std;
  rpm = (rpm - rpm_mean) / rpm_std;
  current = (current - c_mean) / c_std;
  vibration = (vibration - v_mean) / v_std;

  // Debug print to check normalized values
  Serial.print("Normalized Temperature: ");
  Serial.println(temperature);
  Serial.print("Normalized RPM: ");
  Serial.println(rpm);
  Serial.print("Normalized Current: ");
  Serial.println(current);
  Serial.print("Normalized Vibration: ");
  Serial.println(vibration);


  // Store the normalized and quantized samples in the circular buffers
  temperature_vals[cur_idx] = quantize(temperature, tflu_i_scale, tflu_i_zero_point);
  rpm_vals[cur_idx] = quantize(rpm, tflu_i_scale, tflu_i_zero_point);
  vibration_vals[cur_idx] = quantize(vibration, tflu_i_scale, tflu_i_zero_point);
  current_vals[cur_idx] = quantize(current, tflu_i_scale, tflu_i_zero_point);


#ifdef CIRCULAR_BUFFER_DEBUG  
  Serial.print("Current Index: ");
  Serial.println(cur_idx);
  for (int i = 0; i < NUM_HOURS; i++) {
      Serial.print("Iteration ");
      Serial.print(i);
      Serial.print(": ");
      printCircularBuffer(temperature_vals, cur_idx, NUM_HOURS, "Temperature");
      printCircularBuffer(rpm_vals, cur_idx, NUM_HOURS, "RPM");
      printCircularBuffer(vibration_vals, cur_idx, NUM_HOURS, "Vibration");
      printCircularBuffer(current_vals, cur_idx, NUM_HOURS, "Current");
      delay(1000); // Adjust delay as needed
  }
#endif
  // Update the circular buffer index
  cur_idx = (cur_idx + 1) % NUM_HOURS;



  delay(10000);  // Wait for 10 second before the next loop
}

