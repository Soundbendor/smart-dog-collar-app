#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <TensorFlowLite.h>

#include <cmath>

#include "smart_dog_collar_model_data.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#define SMART_DOG_COLLAR_DEBUG
// #undef SMART_DOG_COLLAR_DEBUG
// TODO LIST
// Double check data FIFO
// output handler
// tensor input/output checks
// ml model input

namespace {
  const int VERSION = 0x00000001;

  /*************/
  /* Variables */
  /*************/
  constexpr int input_count = 6;

  // A buffer holding the last 600 sets of 3-channel values from the accelerometer.
  constexpr int acceleration_data_length = 600 * 3;
  float acceleration_data[acceleration_data_length] = {};
  int acceleration_data_index = 0;  // The next free entry in the data array.
  float acceleration_sample_rate = 0.0f;

  // A buffer holding the last 600 sets of 3-channel values from the gyroscope.
  constexpr int gyroscope_data_length = 600 * 3;
  float gyroscope_data[gyroscope_data_length] = {};
  int gyroscope_data_index = 0;  // The next free entry in the data array.
  float gyroscope_sample_rate = 0.0f;

  // TODO: Idk if these are needed
  int feature_buffer[6] = {0, 0, 0, 0, 0, 0};
  float current_acceleration[3] = { 0.0f, 0.0f, 0.0f };
  float current_rotation[3] = { 0.0f, 0.0f, 0.0f };

  // Create an area of memory to use for input, output, and intermediate arrays.
  // The size of this will depend on the model you're using, and may need to be
  // determined by experimentation.
  constexpr int kTensorArenaSize = 60 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];

  tflite::ErrorReporter *error_reporter = nullptr;
  const tflite::Model *model = nullptr;
  tflite::MicroInterpreter *interpreter = nullptr;
  TfLiteTensor *model_input = nullptr;
  TfLiteTensor *model_output = nullptr;

  constexpr int label_count = 7;
  constexpr int seizure = 4;
  const char *labels[label_count] = { "car", "leisure", "play", "run_jog",
                                      "seizure", "sleep", "walk" };

  /***********/
  /* Methods */
  /***********/

  void setupIMU() 
  {
    // Make sure we are pulling measurements into a FIFO.
    // If you see an error on this line, make sure you have at least v1.1.0 of the
    // Arduino_LSM9DS1 library installed.
    IMU.setContinuousMode();

    acceleration_sample_rate = IMU.accelerationSampleRate();
    gyroscope_sample_rate = IMU.gyroscopeSampleRate();

    #ifdef SMART_DOG_COLLAR_DEBUG
    float rate_frac;
    float rate_int;
    rate_frac = modf(acceleration_sample_rate, &rate_int);
    TF_LITE_REPORT_ERROR(error_reporter, "Acceleration sample rate %d.%d Hz",
                        static_cast<int32_t>(rate_int),
                        static_cast<int32_t>(rate_frac * 100));
    rate_frac = modf(gyroscope_sample_rate, &rate_int);
    TF_LITE_REPORT_ERROR(error_reporter, "Gyroscope sample rate %d.%d Hz",
                        static_cast<int32_t>(rate_int),
                        static_cast<int32_t>(rate_frac * 100));
    #endif
  }

  void readAccelerometerAndGyroscope() 
  {
    int num_parameters = 3;

    // Get new samples
    while (IMU.accelerationAvailable()) 
    {
      const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
      gyroscope_data_index += 3;
      float* current_gyroscope_data = &gyroscope_data[gyroscope_index];

      // Read a sample
      if (!IMU.readGyroscope(current_gyroscope_data[0],
                            current_gyroscope_data[1],
                            current_gyroscope_data[2])) 
      {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to read gyroscope data");
        break;
      }

      const int acceleration_index = (acceleration_data_index % acceleration_data_length);
      acceleration_data_index += 3;
      float* current_acceleration_data = &acceleration_data[acceleration_index];

      // Read a sample
      if (!IMU.readAcceleration(current_acceleration_data[0],
                                current_acceleration_data[1],
                                current_acceleration_data[2])) 
      {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to read acceleration data");
        break;
      }
      #ifdef SMART_DOG_COLLAR_DEBUG
      Serial.print(current_acceleration_data[0]);
      Serial.print('\t');
      Serial.print(current_acceleration_data[1]);
      Serial.print('\t');
      Serial.print(current_acceleration_data[2]);
      Serial.print('\t');
      Serial.print(current_gyroscope_data[0]);
      Serial.print('\t');
      Serial.print(current_gyroscope_data[1]);
      Serial.print('\t');
      Serial.println(current_gyroscope_data[2]);
      #endif
    }
  }

  // TODO: handle output in some way
  void handleOutput(int activity)
  {
    #ifdef SMART_DOG_COLLAR_DEBUG
    Serial.print(labels[activity]);
    #endif
    
    // Handle seizure
    if(activity == seizure)
    {
      // Send push notification?

    }

    // Everything that must be done
    // Send status to phone
  }
}

void setup() {
  // Setup Serial
  tflite::InitializeTarget();

  // Setup logging
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  TF_LITE_REPORT_ERROR(error_reporter, "Started");

  // Try to start up the IMU
  if (!IMU.begin()) {
    TF_LITE_REPORT_ERROR(error_reporter, "Failed to initialize IMU");
    while (true) {
      // Never return due to error
    }
  }
  setupIMU();

  // Map the model into a usable data structure.
  model = tflite::GetModel(sdc_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);

    return;
  }

  // Pulls in all the operation implementations we need
  static tflite::AllOpsResolver resolver;

  // Build an intepreter to run the model with
  static tflite::MicroInterpreter static_interpreter(model,
                                                     resolver,
                                                     tensor_arena,
                                                     kTensorArenaSize,
                                                     error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if(allocate_status != kTfLiteOk)
  {
    TF_LITE_REPORT_ERROR(error_reporter, "Memory allocations failed");
    return;
  }

  // TODO: Check if setup correctly
  // Obtain pointer to model's input and check model input parameters
  model_input = interpreter->input(0);
  if ((model_input->dims->size != input_count) || (model_input->dims->data[0] != 1)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Bad input tensor parameters in model");
    return;
  }

  // TODO: Check if setup correctly
  // Obtain pointer to model's output and check model output parameters
  model_output = interpreter->output(0);
  if ((model_output->dims->size != label_count) || (model_output->dims->data[0] != 1) || (model_output->dims->data[1] != label_count)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Bad output tensor parameters in model");
    return;
  }
}

void loop() {
  // Check if data is avaliable
  const bool data_available = IMU.accelerationAvailable() || IMU.gyroscopeAvailable();
  if (!data_available) {
    return;
  }

  // Read data from sensors
  readAccelerometerAndGyroscope();

  // Give gyroscope data to model
  for (int i = 0; i < 3; i++) {
    model_input->data.int8[i] = feature_buffer[i];
  }
  //Give accelerometer data to model
  for (int i = 0; i < 3; i++) {
    model_input->data.int8[i + 3] = feature_buffer[i + 3];
  }

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
    return;
  }

  // Read the results of the ml model
  int8_t max_score = 0;
  int max_index;
  for (int i = 0; i < label_count; i++) {
    const int8_t score = model_output->data.int8[i];
    if ((i == 0) || (score > max_score)) {
      max_score = score;
      max_index = i;
    }
  }

  // Handle the results of the ml model
  handleOutput(max_index);

  #ifdef SMART_DOG_COLLAR_DEBUG
  Serial.print(feature_buffer[0]);
  Serial.print('\t');
  Serial.print(feature_buffer[1]);
  Serial.print('\t');
  Serial.print(feature_buffer[2]);
  Serial.print('\t');
  Serial.print(feature_buffer[3]);
  Serial.print('\t');
  Serial.print(feature_buffer[4]);
  Serial.print('\t');
  Serial.println(feature_buffer[5]);
  #endif
}
