# Specify that the C++ language should be used for compiling this Cython module
# distutils: language = c++

# Import specific types from the visionipc module to be used with OpenCL operations
from cereal.visionipc.visionipc cimport cl_device_id, cl_context, cl_mem

# Extern block to declare mat3 structure from "common/mat.h". This structure is used for 3x3 matrices, common in transformation operations.
cdef extern from "common/mat.h":
  cdef struct mat3:
    # Declare a 3x3 matrix as a single-dimensional array of 9 floats. This is a common way to handle matrices in C for efficiency.
    float v[9]

# Extern block to import functions and constants from "common/clutil.h" related to OpenCL utilities.
cdef extern from "common/clutil.h":
  # Define a constant representing the default device type for OpenCL operations. This is typically used to select the GPU or CPU as the OpenCL device.
  cdef unsigned long CL_DEVICE_TYPE_DEFAULT
  # Declare a function to retrieve an OpenCL device ID given a device type. This function wraps OpenCL calls to simplify device selection in Cython.
  cl_device_id cl_get_device_id(unsigned long)
  # Declare a function to create an OpenCL context given a device ID. An OpenCL context is a central piece of OpenCL architecture, managing objects like command-queues, memory, program and kernel objects.
  cl_context cl_create_context(cl_device_id)

# Extern block to import specific functions and classes from "selfdrive/modeld/models/commonmodel.h" for model operations.
cdef extern from "selfdrive/modeld/models/commonmodel.h":
  # Declare a sigmoid function that operates on a float and returns a float. This function is commonly used in neural networks as an activation function.
  float sigmoid(float)

  # Declare a C++ class ModelFrame. This class is responsible for preparing the frames (images) for the neural network model, including transformations and memory management for GPU processing.
  cppclass ModelFrame:
    # Declare an integer to hold the buffer size. This size is necessary for managing memory allocations for the model's input frames.
    int buf_size
    # Constructor declaration for ModelFrame, requiring an OpenCL device ID and context. This sets up the necessary OpenCL resources for frame processing.
    ModelFrame(cl_device_id, cl_context)
    # Declare a prepare function that processes an image frame and prepares it for the model. This function takes parameters for the image buffer, its dimensions, stride, UV offset, a transformation matrix, and an output memory object.
    float * prepare(cl_mem, int, int, int, int, mat3, cl_mem*)
