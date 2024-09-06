#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include "model.h"

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>

// Define the DebugLog function for logging
extern "C" void DebugLog(const char *s) {
  Serial.println(s);  // Use Serial to print logs to the Serial Monitor
}

//#define DEBUG
//#define CIRCULAR_BUFFER_DEBUG

// RPM - Mean: 1603.866, Standard Deviation: 195.843
// Temperature (°C) - Mean: 24.354, Standard Deviation: 4.987
// Vibration (g) - Mean: 0.120, Standard Deviation: 0.020
// Current (A) - Mean: 3.494, Standard Deviation: 0.308

bool is_valid              = false;
bool show_statistics = false;
// Define thresholds for failure conditions
const float high_temp_threshold = 30.0;           // High temperature threshold in °C
const float low_rpm_threshold = 1500.0;           // Low RPM threshold
const float high_vibration_threshold = 0.60;      // High vibration threshold in g
const float abnormal_current_low_threshold = 0.2; // Low current threshold in A
const float abnormal_current_high_threshold = 10.8;// High current threshold in A

constexpr int num_reads = 3;

// Statistical Summary of Features Before Scaling and Balancing:
// RPM - Mean: 1603.866, Standard Deviation: 195.843
// Temperature (°C) - Mean: 24.354, Standard Deviation: 4.987
// Vibration (g) - Mean: 0.120, Standard Deviation: 0.020
// Current (A) - Mean: 3.494, Standard Deviation: 0.308

constexpr float t_mean  = 24.354f;
constexpr float rpm_mean  = 1603.866;
constexpr float v_mean  = 0.120f;
constexpr float c_mean  = 3.494;
constexpr float t_std   = 4.987f;
constexpr float rpm_std   = 195.843f;
constexpr float v_std   = 0.020f;
constexpr float c_std   = 0.308f;

// TensorFlow Lite for Microcontroller global variables
const tflite::Model* tflu_model            = nullptr;
tflite::MicroInterpreter* tflu_interpreter = nullptr;
TfLiteTensor* tflu_i_tensor                = nullptr;
TfLiteTensor* tflu_o_tensor                = nullptr;
tflite::MicroErrorReporter tflu_error;

constexpr int tensor_arena_size = 4 * 1024;
byte tensor_arena[tensor_arena_size] __attribute__((aligned(16)));
float   tflu_i_scale      = 0.0f;
float   tflu_o_scale      = 0.0f;
int32_t tflu_i_zero_point = 0;
int32_t tflu_o_zero_point = 0;

const float temperature_conversion_factor = 3.3f / (1 << 12);

// Statistics counters
int total_predictions = 0;
int true_positives = 0;
int false_positives = 0;
int true_negatives = 0;
int false_negatives = 0;
float rolling_accuracy = 0.0;

#ifdef I2C_EXTERNAL
// Create an instance of the SHT4x sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
// Create an instance of the Adafruit_ADXL34 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif

// Define the pins for RP2040
const int switchPin = 2;         // GPIO2 for the switch
const int analogInPin = 26;      // GPIO26 for analog input (ADC)

// Sensitivity value for ACS712-05B (±5A version)
const float sensitivity = 0.185; // V/A for 5A version

// Voltage divider scaling factor
const float scale = 4.3;  // Scaling factor for 330kΩ / 100kΩ divider



float threshold = 0.1;  // Vibration threshold in g
float baseReading = 0.0;
bool firstRun = true;

inline int8_t quantize(float x, float scale, float zero_point)
{
    // Compute the quantized value
    float quantized_float = (x / scale) + zero_point;
    // Cast to int8_t
    int8_t quantized_value = static_cast<int8_t>(quantized_float);
    return quantized_value;
}

inline float dequantize(int8_t x, float scale, float zero_point)
{
  return ((float)x - zero_point) * scale;
}

void tflu_initialization()
{
  Serial.println("TFLu initialization - start");

  // Load the TFLITE model
  tflu_model = tflite::GetModel(preventive_forecast_tflite);
  if (tflu_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.print(tflu_model->version());
    Serial.println("");
    Serial.print(TFLITE_SCHEMA_VERSION);
    Serial.println("");
    while(1);
  }

  tflite::AllOpsResolver tflu_ops_resolver;

  // Initialize the TFLu interpreter
  tflu_interpreter = new tflite::MicroInterpreter(
      tflu_model,             // The model pointer
      tflu_ops_resolver,      // The op resolver
      tensor_arena,           // The tensor arena
      tensor_arena_size,      // The size of the tensor arena
      nullptr,                // No resource variables, so set this to nullptr
      nullptr                 // No profiler, so set this to nullptr
  );

  // Allocate TFLu internal memory
  tflu_interpreter->AllocateTensors();

  // Get the pointers for the input and output tensors
  tflu_i_tensor = tflu_interpreter->input(0);
  tflu_o_tensor = tflu_interpreter->output(0);

  const auto* i_quantization = reinterpret_cast<TfLiteAffineQuantization*>(tflu_i_tensor->quantization.params);
  const auto* o_quantization = reinterpret_cast<TfLiteAffineQuantization*>(tflu_o_tensor->quantization.params);

  // Get the quantization parameters (per-tensor quantization)
  tflu_i_scale      = i_quantization->scale->data[0];
  tflu_i_zero_point = i_quantization->zero_point->data[0];
  tflu_o_scale      = o_quantization->scale->data[0];
  tflu_o_zero_point = o_quantization->zero_point->data[0];

  Serial.println("TFLu initialization - completed");
}

// Function to generate normally distributed random values using Box-Muller transform
float generateRandomValue(float mean, float stddev) {
  float u1 = random(0, 10000) / 10000.0;
  float u2 = random(0, 10000) / 10000.0;
  
  // Box-Muller transform to generate normally distributed values
  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
  float value = mean + z0 * stddev;

  return value;
}

float readRPM() {
  int switchState = digitalRead(switchPin);
  float rpm;

  if (switchState == LOW) {
    rpm =  generateRandomValue(1400, 50); // Centered around 1400 with a smaller deviation
    show_statistics = true;
  } else {
    rpm =  generateRandomValue(rpm_mean, rpm_std);
  }
  return rpm;
}

#ifdef I2C_EXTERNAL
float readVibration()
{
  sensors_event_t event;
  accel.getEvent(&event);
  float xReading = event.acceleration.x;

  if (firstRun) {
    baseReading = xReading;
    firstRun = false;
  }
  float deltaX = abs(xReading - baseReading);
  return deltaX;
}
#else
float readVibration()
{
  float vibration = generateRandomValue(v_mean, v_std);
  return vibration;
}
#endif

void setup() {
  Wire.begin();
  pinMode(switchPin, INPUT_PULLUP);
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Serial Initialization completed");

#ifdef I2C_EXTERNAL
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x sensor!");
    while (1) delay(10);
  }
  if (!accel.begin()) {
    Serial.println("Couldn't find ADXL345 sensor!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_800_HZ);
#endif  
  adc_init();
  adc_set_temp_sensor_enabled(true);
  tflu_initialization();
}

#ifdef I2C_EXTERNAL
float readTemperature() {
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);
  return temp.temperature;
}
#else
float readTemperature()
{
  return generateRandomValue(t_mean, t_std);
}
#endif

float readCurrent() {
  return generateRandomValue(c_mean, c_std);
}

// Check failure conditions and return true if any are met
bool checkFailureConditions(float temperature, float rpm, float vibration, float current) {
  bool condition_met = false;
  String failure_reason = "";

  if (temperature > high_temp_threshold) {
    condition_met = true;
    failure_reason += "High Temperature; ";
  }
  if (rpm < low_rpm_threshold) {
    condition_met = true;
    failure_reason += "Low RPM; ";
  }
  if (vibration > high_vibration_threshold) {
    condition_met = true;
    failure_reason += "High Vibration; ";
  }
  if (current < abnormal_current_low_threshold) {
    condition_met = true;
    failure_reason += "Low Current; ";
  }
  if (current > abnormal_current_high_threshold) {
    condition_met = true;
    failure_reason += "High Current; ";
  }

  if (condition_met) {
    Serial.print("Warning: Sensor readings indicate a potential failure condition. Reasons: ");
    Serial.println(failure_reason);
  }

  return condition_met;  // Return true if failure conditions are met, false otherwise
}

void loop() {
  float rpm = 0.0f;
  float temperature = 0.0f;
  float current = 0.0f;
  float vibration = 0.0f;

  // Read sensor values
  temperature += readTemperature();
  rpm += readRPM();    
  current += readCurrent();
  vibration += readVibration();




  // Check if actual failure conditions are met
  bool actual_failure = checkFailureConditions(temperature, rpm, vibration, current);

  // Normalize the values
  float temperature_n = (temperature - t_mean) / t_std;
  float rpm_n = (rpm - rpm_mean) / rpm_std;
  float current_n = (current - c_mean) / c_std;
  float vibration_n = (vibration - v_mean) / v_std;

  // Quantize the values
  int8_t quantized_temperature = quantize(temperature_n, tflu_i_scale, tflu_i_zero_point);
  int8_t quantized_rpm = quantize(rpm_n, tflu_i_scale, tflu_i_zero_point);
  int8_t quantized_vibration = quantize(vibration_n, tflu_i_scale, tflu_i_zero_point);
  int8_t quantized_current = quantize(current_n, tflu_i_scale, tflu_i_zero_point);

  // Initialize the input tensor
  tflu_i_tensor->data.int8[0] = quantized_temperature;
  tflu_i_tensor->data.int8[1] = quantized_rpm;
  tflu_i_tensor->data.int8[2] = quantized_vibration;
  tflu_i_tensor->data.int8[3] = quantized_current;

  // Run inference
  TfLiteStatus invoke_status = tflu_interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Error invoking the TFLu interpreter");
    return;
  }

  // Get the model prediction
  int8_t out_int8 = tflu_o_tensor->data.int8[0];
  float out_f = dequantize(out_int8, tflu_o_scale, tflu_o_zero_point);
  
  bool predicted_failure = (out_f > 0.50);  // Model predicts failure if out_f > 0.5

  // Update prediction statistics
  if (predicted_failure && actual_failure) {
    true_positives++;
  } else if (predicted_failure && !actual_failure) {
    false_positives++;
    show_statistics = true;
  } else if (!predicted_failure && actual_failure) {
    false_negatives++;
    show_statistics = true;
  } else if (!predicted_failure && !actual_failure) {
    true_negatives++;
  }

  total_predictions++;

  // Calculate rolling accuracy (percentage of correct predictions)
  rolling_accuracy = (float)(true_positives + true_negatives) / total_predictions * 100.0;
  show_statistics = true;
  if(show_statistics)
  {
    Serial.println("-----------------------------");
    Serial.print("Confidence: ");
    Serial.println(out_f);
    Serial.print("Predicted failure: ");
    Serial.println(predicted_failure);
    Serial.print("Actual failure: ");
    Serial.println(actual_failure);
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
    // Print statistics

    Serial.print("Total Predictions: ");
    Serial.println(total_predictions);
    Serial.print("True Positives: ");
    Serial.println(true_positives);
    Serial.print("False Positives: ");
    Serial.println(false_positives);
    Serial.print("True Negatives: ");
    Serial.println(true_negatives);
    Serial.print("False Negatives: ");
    Serial.println(false_negatives);
    Serial.print("Rolling Accuracy (%): ");
    Serial.println(rolling_accuracy);
      // Ensure to flush the Serial output
    //Serial.flush();
    show_statistics = false;

  }
  delay(10000);  // Wait for 10 seconds before the next loop
}
