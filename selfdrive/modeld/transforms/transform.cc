#include "selfdrive/modeld/transforms/transform.h"

#include <cassert>
#include <cstring>

#include "common/clutil.h"

// Initializes the transformation settings and allocates required resources
void transform_init(Transform* s, cl_context ctx, cl_device_id device_id) {
  // Zeroes out the Transform structure
  memset(s, 0, sizeof(*s));

  // Compiles the OpenCL program from file
  cl_program prg = cl_program_from_file(ctx, device_id, TRANSFORM_PATH, "");
  // Creates an OpenCL kernel from the compiled program
  s->krnl = CL_CHECK_ERR(clCreateKernel(prg, "warpPerspective", &err));
  // done with this
  // Releases the compiled OpenCL program as it's no longer needed
  CL_CHECK(clReleaseProgram(prg));

  // Allocates OpenCL memory for Y channel transformation matrix
  s->m_y_cl = CL_CHECK_ERR(clCreateBuffer(ctx, CL_MEM_READ_WRITE, 3*3*sizeof(float), NULL, &err));
  // Allocates OpenCL memory for UV channel transformation matrix
  s->m_uv_cl = CL_CHECK_ERR(clCreateBuffer(ctx, CL_MEM_READ_WRITE, 3*3*sizeof(float), NULL, &err));
}

// Frees the resources allocated for the transformation
void transform_destroy(Transform* s) {
  // Releases the OpenCL memory objects and kernel
  CL_CHECK(clReleaseMemObject(s->m_y_cl));
  CL_CHECK(clReleaseMemObject(s->m_uv_cl));
  CL_CHECK(clReleaseKernel(s->krnl));
}

// Enqueues transformation tasks to the OpenCL command queue
void transform_queue(Transform* s,
                     cl_command_queue q,
                     cl_mem in_yuv, int in_width, int in_height, int in_stride, int in_uv_offset,
                     cl_mem out_y, cl_mem out_u, cl_mem out_v,
                     int out_width, int out_height,
                     const mat3& projection) {
  // Sets the origin for sampling to pixel center
  const int zero = 0;

  // sampled using pixel center origin
  // (because that's how fastcv and opencv does it)
  // Projection matrix for Y channel remains unchanged
  mat3 projection_y = projection;

  // in and out uv is half the size of y.
  // Scales the projection matrix for UV channels since they are half the size of Y
  mat3 projection_uv = transform_scale_buffer(projection, 0.5);

  // Writes the Y channel projection matrix to the OpenCL buffer
  CL_CHECK(clEnqueueWriteBuffer(q, s->m_y_cl, CL_TRUE, 0, 3*3*sizeof(float), (void*)projection_y.v, 0, NULL, NULL));
  // Writes the UV channel projection matrix to the OpenCL buffer
  CL_CHECK(clEnqueueWriteBuffer(q, s->m_uv_cl, CL_TRUE, 0, 3*3*sizeof(float), (void*)projection_uv.v, 0, NULL, NULL));

  // Input image dimensions for Y channel
  const int in_y_width = in_width;
  const int in_y_height = in_height;
  const int in_y_px_stride = 1;
  // Input image dimensions for UV channels are half that of Y
  const int in_uv_width = in_width/2;
  const int in_uv_height = in_height/2;
  const int in_uv_px_stride = 2;
  const int in_u_offset = in_uv_offset;
  const int in_v_offset = in_uv_offset + 1;

  // Output image dimensions for Y channel
  const int out_y_width = out_width;
  const int out_y_height = out_height;
  // Output image dimensions for UV channels are half that of Y
  const int out_uv_width = out_width/2;
  const int out_uv_height = out_height/2;

  // Setting kernel arguments for processing Y channel
  CL_CHECK(clSetKernelArg(s->krnl, 0, sizeof(cl_mem), &in_yuv));  // src
  CL_CHECK(clSetKernelArg(s->krnl, 1, sizeof(cl_int), &in_stride));  // src_row_stride
  CL_CHECK(clSetKernelArg(s->krnl, 2, sizeof(cl_int), &in_y_px_stride));  // src_px_stride
  CL_CHECK(clSetKernelArg(s->krnl, 3, sizeof(cl_int), &zero));  // src_offset
  CL_CHECK(clSetKernelArg(s->krnl, 4, sizeof(cl_int), &in_y_height));  // src_rows
  CL_CHECK(clSetKernelArg(s->krnl, 5, sizeof(cl_int), &in_y_width));  // src_cols
  CL_CHECK(clSetKernelArg(s->krnl, 6, sizeof(cl_mem), &out_y));  // dst
  CL_CHECK(clSetKernelArg(s->krnl, 7, sizeof(cl_int), &out_y_width));  // dst_row_stride
  CL_CHECK(clSetKernelArg(s->krnl, 8, sizeof(cl_int), &zero));  // dst_offset
  CL_CHECK(clSetKernelArg(s->krnl, 9, sizeof(cl_int), &out_y_height));  // dst_rows
  CL_CHECK(clSetKernelArg(s->krnl, 10, sizeof(cl_int), &out_y_width));  // dst_cols
  CL_CHECK(clSetKernelArg(s->krnl, 11, sizeof(cl_mem), &s->m_y_cl));  // M

  // Determine the global work size for the Y channel
  const size_t work_size_y[2] = {(size_t)out_y_width, (size_t)out_y_height};

  // Enqueue the OpenCL kernel for execution with work size for the Y channel
  CL_CHECK(clEnqueueNDRangeKernel(q, s->krnl, 2, NULL,
                                (const size_t*)&work_size_y, NULL, 0, 0, NULL));

  // Determine the global work size for the UV channels
  const size_t work_size_uv[2] = {(size_t)out_uv_width, (size_t)out_uv_height};

  // Set the OpenCL kernel arguments for the UV channel transformation: pixel stride, source offset, and image dimensions
  CL_CHECK(clSetKernelArg(s->krnl, 2, sizeof(cl_int), &in_uv_px_stride));  // src_px_stride
  CL_CHECK(clSetKernelArg(s->krnl, 3, sizeof(cl_int), &in_u_offset));  // src_offset
  CL_CHECK(clSetKernelArg(s->krnl, 4, sizeof(cl_int), &in_uv_height));  // src_rows
  CL_CHECK(clSetKernelArg(s->krnl, 5, sizeof(cl_int), &in_uv_width));  // src_cols
  CL_CHECK(clSetKernelArg(s->krnl, 6, sizeof(cl_mem), &out_u));  // dst
  CL_CHECK(clSetKernelArg(s->krnl, 7, sizeof(cl_int), &out_uv_width));  // dst_row_stride
  CL_CHECK(clSetKernelArg(s->krnl, 8, sizeof(cl_int), &zero));  // dst_offset
  CL_CHECK(clSetKernelArg(s->krnl, 9, sizeof(cl_int), &out_uv_height));  // dst_rows
  CL_CHECK(clSetKernelArg(s->krnl, 10, sizeof(cl_int), &out_uv_width));  // dst_cols
  CL_CHECK(clSetKernelArg(s->krnl, 11, sizeof(cl_mem), &s->m_uv_cl));  // M

  // Enqueue the OpenCL kernel for execution with work size for the UV channel (U plane)
  CL_CHECK(clEnqueueNDRangeKernel(q, s->krnl, 2, NULL,
                                (const size_t*)&work_size_uv, NULL, 0, 0, NULL));
                                
  // Update the kernel argument for the V channel source offset and destination buffer
  CL_CHECK(clSetKernelArg(s->krnl, 3, sizeof(cl_int), &in_v_offset));  // src_ofset
  CL_CHECK(clSetKernelArg(s->krnl, 6, sizeof(cl_mem), &out_v));  // dst

  // Enqueue the kernel again for the V channel with the updated arguments
  CL_CHECK(clEnqueueNDRangeKernel(q, s->krnl, 2, NULL,
                                (const size_t*)&work_size_uv, NULL, 0, 0, NULL));
}
