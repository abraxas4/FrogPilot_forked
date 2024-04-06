// Includes the header for the common model functionalities
#include "selfdrive/modeld/models/commonmodel.h"

// Standard library headers for algorithms, assertions, mathematical operations, and memory operations
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

// Include headers for OpenCL utilities, matrix operations, and timing functions
#include "common/clutil.h"
#include "common/mat.h"
#include "common/timing.h"

// Constructor for the ModelFrame class: initializes OpenCL buffers and resources for image processing
ModelFrame::ModelFrame(cl_device_id device_id, cl_context context) {
  // Allocate memory for storing input frames
  input_frames = std::make_unique<float[]>(buf_size);

  // Create an OpenCL command queue for issuing commands to the specified device
  q = CL_CHECK_ERR(clCreateCommandQueue(context, device_id, 0, &err));
  // Allocate OpenCL memory buffers for storing the Y channel of the image
  y_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_WIDTH * MODEL_HEIGHT, NULL, &err));
  // Allocate OpenCL memory buffers for storing the U channel of the image
  u_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  // Allocate OpenCL memory buffers for storing the V channel of the image
  v_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2), NULL, &err));
  // Allocate an OpenCL memory buffer for the neural network's input data
  net_input_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, MODEL_FRAME_SIZE * sizeof(float), NULL, &err));

  // Initialize the transform and YUV loading operations
  transform_init(&transform, context, device_id);
  loadyuv_init(&loadyuv, context, device_id, MODEL_WIDTH, MODEL_HEIGHT);
}

// Prepares the image frame for processing by transforming and loading it into the appropriate format
float* ModelFrame::prepare(cl_mem yuv_cl, int frame_width, int frame_height, int frame_stride, int frame_uv_offset, const mat3 &projection, cl_mem *output) {
  // Queue the transformation operation for the input YUV frame
  transform_queue(&this->transform, q,
                  yuv_cl, frame_width, frame_height, frame_stride, frame_uv_offset,
                  y_cl, u_cl, v_cl, MODEL_WIDTH, MODEL_HEIGHT, projection);

  // Check if the output buffer is provided or if internal storage should be used
  if (output == NULL) {
    // Queue the operation to load the transformed YUV data into the neural network's input buffer
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, net_input_cl);

    // Move the previous frame's data to make space for new data
    std::memmove(&input_frames[0], &input_frames[MODEL_FRAME_SIZE], sizeof(float) * MODEL_FRAME_SIZE);
    // Read the processed frame data from the OpenCL buffer into the internal storage
    CL_CHECK(clEnqueueReadBuffer(q, net_input_cl, CL_TRUE, 0, MODEL_FRAME_SIZE * sizeof(float), &input_frames[MODEL_FRAME_SIZE], 0, nullptr, nullptr));
    // Ensure all previously queued OpenCL commands are finished
    clFinish(q);
    // Return a pointer to the internal storage of frame data
    return &input_frames[0];
  } else {
    // Queue the operation to load the transformed YUV data directly into the provided output buffer
    loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, *output, true);
    // NOTE: Since thneed is using a different command queue, this clFinish is needed to ensure the image is ready.
    // Ensure all previously queued OpenCL commands are finished, necessary for synchronization with different command queues
    clFinish(q);
    // No internal storage is used, return NULL
    return NULL;
  }
}

// Destructor for the ModelFrame class: releases all allocated OpenCL resources
ModelFrame::~ModelFrame() {
  // Destroy the transformation and YUV loading operations
  transform_destroy(&transform);
  loadyuv_destroy(&loadyuv);
  // Release the OpenCL memory objects for the neural network's input data and image channels
  CL_CHECK(clReleaseMemObject(net_input_cl));
  CL_CHECK(clReleaseMemObject(v_cl));
  CL_CHECK(clReleaseMemObject(u_cl));
  CL_CHECK(clReleaseMemObject(y_cl));
  // Release the OpenCL command queue
  CL_CHECK(clReleaseCommandQueue(q));
}

// Defines the softmax function, which is used to normalize an array of logits to probabilities
void softmax(const float* input, float* output, size_t len) {
  // Find the maximum value in the input array for numerical stability
  const float max_val = *std::max_element(input, input + len);
  
  // Initialize the denominator of the softmax formula to zero
  float denominator = 0;
  
  // Iterate over the input array to compute the exponential of each value shifted by max_val, 
  // contributing to the computation of both the numerator and the denominator of the softmax function
  for (int i = 0; i < len; i++) {
    // Calculate exp(value - max_val) for numerical stability
    float const v_exp = expf(input[i] - max_val); 
    // Accumulate the sum of exponentials for the denominator
    denominator += v_exp; 
    // Store the unnormalized value in the output array
    output[i] = v_exp; 
  }

  // Calculate the inverse of the denominator (1 / sum of exponentials) for normalization
  const float inv_denominator = 1. / denominator;
  
  // Normalize each output value by multiplying by the inverse of the denominator 
  // to convert logits to probabilities that sum to 1
  for (int i = 0; i < len; i++) {
    output[i] *= inv_denominator;
  }
}

// Defines the sigmoid function, which is commonly used as an activation function in neural networks
float sigmoid(float input) {
  // Calculate and return the sigmoid of the input: 1 / (1 + exp(-input))
  // The sigmoid function maps any real value to the (0, 1) interval, acting as a nonlinear squashing function
  return 1 / (1 + expf(-input));
}
