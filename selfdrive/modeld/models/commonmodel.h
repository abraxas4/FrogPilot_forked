#pragma once

// Standard libraries for floating-point limits and standard utilities
#include <cfloat>
#include <cstdlib>

// For smart pointers
#include <memory>

// Define to use deprecated OpenCL 1.2 APIs for compatibility
#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
// Conditional compilation for Apple's OpenCL framework
#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

// Included headers for matrix operations, messaging between processes, and image transformations
#include "common/mat.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/modeld/transforms/loadyuv.h"
#include "selfdrive/modeld/transforms/transform.h"

// Check if the SEND_RAW_PRED environment variable is set; used to control the output of raw predictions
const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;

// Utility functions for applying softmax and sigmoid operations
void softmax(const float* input, float* output, size_t len);
float sigmoid(float input);

// Template function to convert a std::array to a kj::ArrayPtr, facilitating interoperation with Cap'n Proto
template<class T, size_t size>
constexpr const kj::ArrayPtr<const T> to_kj_array_ptr(const std::array<T, size> &arr) {
  return kj::ArrayPtr(arr.data(), arr.size());
}

// Class definition for handling frames as input to the model
class ModelFrame {
public:
  // Constructor to initialize OpenCL resources
  ModelFrame(cl_device_id device_id, cl_context context);
  // Destructor to release OpenCL resources
  ~ModelFrame();
  // Prepare the frame for input into the neural network, including format conversion and transformation
  float* prepare(cl_mem yuv_cl, int width, int height, int frame_stride, int frame_uv_offset, const mat3& transform, cl_mem *output);

  // Constants defining the dimensions of the input expected by the model
  const int MODEL_WIDTH = 512;
  const int MODEL_HEIGHT = 256;
  // Calculate the size of the frame buffer needed, taking into account the model's input dimensions and format (YUV420)
  const int MODEL_FRAME_SIZE = MODEL_WIDTH * MODEL_HEIGHT * 3 / 2;
  // Buffer size allocation, doubled to handle potential overflow or additional processing needs
  const int buf_size = MODEL_FRAME_SIZE * 2;

private:
  // Transformation utility for image data
  Transform transform;
  // State for loading YUV-formatted image data
  LoadYUVState loadyuv;
  // OpenCL command queue for issuing commands to the device
  cl_command_queue q;
  // OpenCL memory buffers for different image planes and network input
  cl_mem y_cl, u_cl, v_cl, net_input_cl;
  // Buffer to hold input frames before they are processed
  std::unique_ptr<float[]> input_frames;
};
