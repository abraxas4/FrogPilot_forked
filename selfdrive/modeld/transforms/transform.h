#pragma once

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "common/mat.h"

// Structure to hold the OpenCL kernel and memory objects for transformation
typedef struct {
  // Kernel for performing the transformation
  cl_kernel krnl;
  // Memory objects for Y and UV channels of the image
  cl_mem m_y_cl, m_uv_cl;
} Transform;

// Initializes the Transform structure with necessary OpenCL context and device
void transform_init(Transform* s, cl_context ctx, cl_device_id device_id);

// Frees the resources associated with the Transform structure
void transform_destroy(Transform* transform);

// Queues the transformation operation on an OpenCL command queue
void transform_queue(Transform* s, cl_command_queue q,
                     cl_mem yuv, int in_width, int in_height, int in_stride, int in_uv_offset,
                     cl_mem out_y, cl_mem out_u, cl_mem out_v,
                     int out_width, int out_height,
                     const mat3& projection);
