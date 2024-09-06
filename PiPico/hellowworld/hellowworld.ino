#include <Arduino.h>
#include "TensorFlowLite.h"
#include "model.h"  // Include your model

#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/schema/schema_generated.h>


// Define the DebugLog function for logging
extern "C" void DebugLog(const char *s) {
  Serial.println(s);  // Use Serial to print logs to the Serial Monitor
}

// Memory allocation
constexpr int kTensorArenaSize = 2 * 1024;  // Define the tensor arena size based on your model's requirements
uint8_t tensor_arena[kTensorArenaSize];

// Initialize necessary objects
tflite::MicroErrorReporter micro_error_reporter;
tflite::ErrorReporter* error_reporter = &micro_error_reporter;

const tflite::Model* model;
tflite::MicroInterpreter* interpreter;
tflite::AllOpsResolver resolver;
TfLiteTensor* input = NULL;
TfLiteTensor* output = NULL;

void setup() {
  Serial.begin(115200);
  
  // Load the model
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report("Model provided is schema version %d not equal "
                           "to supported version %d.",
                           model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Initialize the TFLu interpreter using the appropriate constructor
  interpreter = new tflite::MicroInterpreter(
      model,             // The model pointer
      resolver,          // The op resolver
      tensor_arena,      // The tensor arena
      kTensorArenaSize,  // The size of the tensor arena
      NULL,              // No resource variables, so set this to NULL
      NULL               // No profiler, so set this to NULL
  );

  // Allocate memory for tensors
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors
  input = interpreter->input(0);
  output = interpreter->output(0);
}

void loop() {
  // Populate input tensor with some data
  for (int i = 0; i < input->dims->data[0]; i++) {
    input->data.f[i] = 0.5;
  }

  // Run the model on this input
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    error_reporter->Report("Invoke failed");
    return;
  }

  // Get the output from the model
  float output_value = output->data.f[0];
  Serial.println(output_value);

  // Add a delay to make the output readable
  delay(1000);
}
