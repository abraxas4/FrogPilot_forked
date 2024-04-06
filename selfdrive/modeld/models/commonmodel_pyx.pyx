# Specify the compilation language as C++ for this Cython module
# distutils: language = c++
# Define the string encoding used in this Cython file as ASCII
# cython: c_string_encoding=ascii

# Import the NumPy library for array manipulation in Python
import numpy as np
# cimport numpy for direct access to NumPy's C-API, enabling efficient array manipulations
cimport numpy as cnp
# Import the memcpy function from libc for memory copying operations at the C level
from libc.string cimport memcpy

# cimport cl_mem from visionipc to use OpenCL memory objects in this Cython module
from cereal.visionipc.visionipc cimport cl_mem
# cimport VisionBuf and a base CLContext class for handling vision buffers and OpenCL contexts
from cereal.visionipc.visionipc_pyx cimport VisionBuf, CLContext as BaseCLContext
# cimport various utilities for model handling, including device type constants, device and context creation functions, and mat3 type for matrices
from .commonmodel cimport CL_DEVICE_TYPE_DEFAULT, cl_get_device_id, cl_create_context
from .commonmodel cimport mat3, sigmoid as cppSigmoid, ModelFrame as cppModelFrame

# Define a Python wrapper for the C++ sigmoid function for use in Python code
def sigmoid(x):
  return cppSigmoid(x)

# Define a Cython class CLContext that extends the BaseCLContext with OpenCL specific initializations
cdef class CLContext(BaseCLContext):
  # Initialize the CLContext with a device ID and an OpenCL context
  def __cinit__(self):
    self.device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT)
    self.context = cl_create_context(self.device_id)

# Define a Cython class for managing OpenCL memory objects
cdef class CLMem:
  # Static method to create and return a new CLMem instance from a given OpenCL memory pointer
  @staticmethod
  cdef create(void * cmem):
    mem = CLMem()
    mem.mem = <cl_mem*> cmem
    return mem

# Define a Cython class ModelFrame to encapsulate operations on model frames, interfacing with the corresponding C++ class
cdef class ModelFrame:
  # Declare a pointer to the C++ ModelFrame class to use its methods
  cdef cppModelFrame * frame

  # Initialize the ModelFrame with a given CLContext, creating a new instance of the C++ ModelFrame
  def __cinit__(self, CLContext context):
    self.frame = new cppModelFrame(context.device_id, context.context)

  # Clean up the allocated ModelFrame when the Cython object is deallocated
  def __dealloc__(self):
    del self.frame

  # Prepare the frame for model processing, converting the projection to the required C++ format and invoking the C++ prepare method
  def prepare(self, VisionBuf buf, float[:] projection, CLMem output):
    # Create a mat3 instance for the projection matrix to be passed to C++
    cdef mat3 cprojection
    # Copy the projection data into the mat3 instance
    memcpy(cprojection.v, &projection[0], 9*sizeof(float))
    # Call the C++ ModelFrame's prepare method and convert the result to a NumPy array
    cdef float * data = self.frame.prepare(buf.buf.buf_cl, buf.width, buf.height, buf.stride, buf.uv_offset, cprojection, output.mem)
    # Check if data is not null, then convert to a NumPy array; else, return None
    if not data:
      return None
    return np.asarray(<cnp.float32_t[:self.frame.buf_size]> data)
