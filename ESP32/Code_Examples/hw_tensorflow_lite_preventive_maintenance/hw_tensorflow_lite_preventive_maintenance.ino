/* Copyright 2020 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

//https://github.com/spaziochirale/Chirale_TensorFlowLite

// Sine Wave Prediction: The "Hello World" model itself is a neural network trained to approximate the sine function.
// It takes an input value x and predicts the corresponding y value, where y = sin(x).
// The input values range between 0 and 2π (which is a full sine wave cycle), and the model is expected to output predictions that closely match the true sine wave.

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "model.h"
#include "constants.h"
#include "output_handler.h"

// Globals, used for compatibility with Arduino-style sketches.
namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;
int inference_count = 0;

constexpr int kTensorArenaSize =  4 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}  // namespace


constexpr float t_mean  = 24.354f;
constexpr float rpm_mean  = 1603.866;
constexpr float v_mean  = 0.120f;
constexpr float c_mean  = 3.494;
constexpr float t_std   = 4.987f;
constexpr float rpm_std   = 195.843f;
constexpr float v_std   = 0.020f;
constexpr float c_std   = 0.308f;

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

  float rpm =  generateRandomValue(rpm_mean, rpm_std);
  return rpm;
}


float readVibration()
{
  float vibration = generateRandomValue(v_mean, v_std);
  return vibration;
}

float readTemperature()
{
  return generateRandomValue(t_mean, t_std);
}


float readCurrent() {
  return generateRandomValue(c_mean, c_std);
}



// The name of this function is important for Arduino compatibility.
void setup() {
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
      "Model provided is schema version %d not equal to supported "
      "version %d.",
      model->version(), TFLITE_SCHEMA_VERSION
    );
    return;
  }

  // Pull in only the operation implementations we need.
  static tflite::MicroMutableOpResolver<2> resolver;
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }
  if (resolver.AddLogistic() != kTfLiteOk) {
    return;
  } 

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    while(1)
      delay(1000);
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

      // Print tensor parameters
  printTensorParams();

  // Keep track of how many inferences we have performed.
  inference_count = 0;
}

inline int8_t quantize(float x, float scale, float zero_point)
{
    // Compute the quantized value
    float quantized_float = (x / scale) + zero_point;
    // Cast to int8_t
    int8_t quantized_value = static_cast<int8_t>(quantized_float);
    MicroPrintf("input_value: %.6f, quantized_value: %d\n", x, quantized_value);

    return quantized_value;
}

inline float dequantize(int8_t x, float scale, float zero_point)
{
  return ((float)x - zero_point) * scale;
}

void printTensorParams() {
  // Print input scale and zero_point
  MicroPrintf("Input Scale: %.6f, Input Zero Point: %d\n", input->params.scale, input->params.zero_point);

  // Print output scale and zero_point
  MicroPrintf("Output Scale: %.6f, Output Zero Point: %d\n", output->params.scale, output->params.zero_point);

}

// The name of this function is important for Arduino compatibility.
void loop() {
  float rpm = 0.0f;
  float temperature = 0.0f;
  float current = 0.0f;
  float vibration = 0.0f;
  bool input_set_failed = false;
  // Read sensor values
  temperature += readTemperature();
  rpm += readRPM();    
  current += readCurrent();
  vibration += readVibration();

  // Normalize the values
  float temperature_n = (temperature - t_mean) / t_std;
  float rpm_n = (rpm - rpm_mean) / rpm_std;
  float current_n = (current - c_mean) / c_std;
  float vibration_n = (vibration - v_mean) / v_std;


  // Quantize the values
  int8_t quantized_temperature = quantize(temperature_n, input->params.scale, input->params.zero_point);
  int8_t quantized_rpm = quantize(rpm_n, input->params.scale, input->params.zero_point);
  int8_t quantized_vibration = quantize(vibration_n, input->params.scale, input->params.zero_point);
  int8_t quantized_current = quantize(current_n, input->params.scale, input->params.zero_point);


  // Place the quantized input in the model's input tensor
  input->data.int8[0] = quantized_temperature;
  input->data.int8[1] = quantized_rpm;
  input->data.int8[2] = quantized_vibration;
  input->data.int8[3] = quantized_current;
  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    MicroPrintf("Invoke failed!\n");
    return;
  }

  // Obtain the quantized output from model's output tensor
  int8_t y_quantized = output->data.int8[0];
  // Dequantize the output from integer to floating-point
  //float y = (y_quantized - output->params.zero_point) * output->params.scale;
  float prediction = dequantize(y_quantized, output->params.scale, output->params.zero_point);

  // Output the results. A custom HandleOutput function can be implemented
  // for each supported hardware target.
  HandleOutput(rpm,temperature, current, vibration, prediction);

  delay(10000);
}
