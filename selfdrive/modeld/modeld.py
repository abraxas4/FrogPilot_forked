#!/usr/bin/env python3
import os
import time
import pickle
import numpy as np
import cereal.messaging as messaging
from cereal import car, log
from pathlib import Path
from typing import Dict, Optional
from setproctitle import setproctitle
from cereal.messaging import PubMaster, SubMaster
from cereal.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive import sentry
from openpilot.selfdrive.car.car_helpers import get_demo_car_params
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.models.commonmodel_pyx import ModelFrame, CLContext

from openpilot.selfdrive.frogpilot.functions.frogpilot_functions import DEFAULT_MODEL

PROCESS_NAME = "selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

MODEL_NAME = Params().get("Model", encoding='utf-8')

MODEL_PATHS = {
  ModelRunner.THNEED: Path(__file__).parent / ('models/supercombo.thneed' if MODEL_NAME == DEFAULT_MODEL else f'models/models/{MODEL_NAME}.thneed'),
  ModelRunner.ONNX: Path(__file__).parent / 'models/supercombo.onnx'}

METADATA_PATH = Path(__file__).parent / 'models/supercombo_metadata.pkl'

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

# Class to manage the model's state including inputs, outputs, and frames
class ModelState:
  # ModelFrame instances for regular and wide angle processing
  frame: ModelFrame
  wide_frame: ModelFrame
  # Dictionary to hold input arrays for the neural network
  inputs: Dict[str, np.ndarray]
  # Numpy array to hold the output predictions from the model
  output: np.ndarray
  # Previous desire state array to track transitions
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse
  # ModelRunner object to manage the execution of the neural network
  model: ModelRunner

  # Constructor of the ModelState class
  def __init__(self, context: CLContext):
    # Initialize regular and wide angle frames
    self.frame = ModelFrame(context)
    self.wide_frame = ModelFrame(context)
    # Set up initial state for previous desires as zeros
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    # Define and initialize all input arrays for the model with zeros
    self.inputs = {
      'desire': np.zeros(ModelConstants.DESIRE_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'traffic_convention': np.zeros(ModelConstants.TRAFFIC_CONVENTION_LEN, dtype=np.float32),
      'lateral_control_params': np.zeros(ModelConstants.LATERAL_CONTROL_PARAMS_LEN, dtype=np.float32),
      'prev_desired_curv': np.zeros(ModelConstants.PREV_DESIRED_CURV_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'nav_features': np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32),
      'nav_instructions': np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32),
      'features_buffer': np.zeros(ModelConstants.HISTORY_BUFFER_LEN * ModelConstants.FEATURE_LEN, dtype=np.float32),
    }

    # Load model metadata from the predefined metadata path
    with open(METADATA_PATH, 'rb') as f:
      model_metadata = pickle.load(f)

    # Extract the slices of the model output for parsing
    self.output_slices = model_metadata['output_slices']
    # Determine the output size from the model metadata
    net_output_size = model_metadata['output_shapes']['outputs'][1]
    # Initialize the output array to hold model predictions
    self.output = np.zeros(net_output_size, dtype=np.float32)
    # Create a parser instance to handle output data
    self.parser = Parser()

    # Set up the model runner with paths and context
    self.model = ModelRunner(MODEL_PATHS, self.output, Runtime.GPU, False, context)
    # Prepare the model inputs for image data
    self.model.addInput("input_imgs", None)
    self.model.addInput("big_input_imgs", None)
    # Prepare all other model inputs
    for k, v in self.inputs.items():
      self.model.addInput(k, v)

  # Function to slice model outputs into a structured format for further processing
  def slice_outputs(self, model_outputs: np.ndarray) -> Dict[str, np.ndarray]:
    # Parse model outputs according to the slices defined in the metadata
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k, v in self.output_slices.items()}
    # If configured, add the raw model predictions to the parsed outputs
    if SEND_RAW_PRED:
      parsed_model_outputs['raw_pred'] = model_outputs.copy()
    # Return the structured model outputs
    return parsed_model_outputs

  # Function to run the model for a given set of inputs and provide the outputs
  def run(self, buf: VisionBuf, wbuf: VisionBuf, transform: np.ndarray, transform_wide: np.ndarray,
                inputs: Dict[str, np.ndarray], prepare_only: bool) -> Optional[Dict[str, np.ndarray]]:
    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    # Set the first desire input to zero as a pulse trigger on rising edge
    inputs['desire'][0] = 0
    # Shift the desire inputs to prepare for the next model run
    self.inputs['desire'][:-ModelConstants.DESIRE_LEN] = self.inputs['desire'][ModelConstants.DESIRE_LEN:]
    # Update the last desire inputs based on rising edge detection
    self.inputs['desire'][-ModelConstants.DESIRE_LEN:] = np.where(inputs['desire'] - self.prev_desire > .99, inputs['desire'], 0)
    # Remember the last desire input for the next run
    self.prev_desire[:] = inputs['desire']

    # Update the inputs with the current traffic convention, lateral control params, and navigational features
    self.inputs['traffic_convention'][:] = inputs['traffic_convention']
    self.inputs['lateral_control_params'][:] = inputs['lateral_control_params']
    self.inputs['nav_features'][:] = inputs['nav_features']
    self.inputs['nav_instructions'][:] = inputs['nav_instructions']

    # if getCLBuffer is not None, frame will be None
    # Prepare the input images buffer for the model
    self.model.setInputBuffer("input_imgs", self.frame.prepare(buf, transform.flatten(), self.model.getCLBuffer("input_imgs")))
    # Prepare the wide angle input images buffer if a wide buffer is provided
    if wbuf is not None:
      self.model.setInputBuffer("big_input_imgs", self.wide_frame.prepare(wbuf, transform_wide.flatten(), self.model.getCLBuffer("big_input_imgs")))

    # If only preparing the model inputs, return here without executing the model
    if prepare_only:
      return None

    # Execute the model with the prepared inputs
    self.model.execute()
    # Parse and slice the outputs from the model execution
    outputs = self.parser.parse_outputs(self.slice_outputs(self.output))

    # Update the feature buffer with the latest hidden state from the model outputs
    self.inputs['features_buffer'][:-ModelConstants.FEATURE_LEN] = self.inputs['features_buffer'][ModelConstants.FEATURE_LEN:]
    self.inputs['features_buffer'][-ModelConstants.FEATURE_LEN:] = outputs['hidden_state'][0, :]
    # Update the previous desired curvature buffer with the model's desired curvature output
    self.inputs['prev_desired_curv'][:-ModelConstants.PREV_DESIRED_CURV_LEN] = self.inputs['prev_desired_curv'][ModelConstants.PREV_DESIRED_CURV_LEN:]
    self.inputs['prev_desired_curv'][-ModelConstants.PREV_DESIRED_CURV_LEN:] = outputs['desired_curvature'][0, :]
    # Return the final parsed outputs for use elsewhere in the system
    return outputs

def main(demo=False):
  # Log the start of the model daemon
  cloudlog.warning("modeld init")

  # Set daemon name tag for Sentry logging
  sentry.set_tag("daemon", PROCESS_NAME)
  # Bind the process name to cloudlog for easier identification in logs
  cloudlog.bind(daemon=PROCESS_NAME)
  # Set the process title for visibility in system process tables
  setproctitle(PROCESS_NAME)
  # Configure this process for real-time execution with specified priorities
  config_realtime_process(7, 54)

  # Log the start of setting up the OpenCL context
  cloudlog.warning("setting up CL context")
  # Create OpenCL context to use with the model
  cl_context = CLContext()
  # Log that the OpenCL context is ready and the model will be loaded next
  cloudlog.warning("CL context ready; loading model")
  # Load the model with the given OpenCL context
  model = ModelState(cl_context)
  # Log that model loading is complete and modeld is starting
  cloudlog.warning("models loaded, modeld starting")

  # visionipc clients
  # Continuously check for available vision IPC streams until found
  while True:
    # Check for available streams from camerad without blocking
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      # Determine if both wide road and road streams are available
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      # Use the main wide camera stream if road stream is not available
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    # Sleep a bit before checking again
    time.sleep(.1)

  # Determine the main stream to use based on available streams
  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  # Initialize the main vision IPC client with the selected stream
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  # Initialize an extra vision IPC client for the wide road stream if needed
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False, cl_context)
  # Log the vision stream setup details
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  # Attempt to connect the main vision IPC client until successful
  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  # Attempt to connect the extra vision IPC client if it's being used
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  # Log successful connection details for the main camera
  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    # Log successful connection details for the extra camera if used
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  # messaging
  # Initialize publishers and subscribers for inter-process messaging
  pm = PubMaster(["modelV2", "cameraOdometry"])
  sm = SubMaster(["carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "navModel", "navInstruction", "carControl", "frogpilotPlan"])

  # Initialize a structure to keep track of when to publish messages
  publish_state = PublishState()
  # Access persistent parameters
  params = Params()

  # setup filter to track dropped frames
  # Setup a filter to monitor the number of dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_FREQ)
  # Initialize frame ID counters
  frame_id = 0
  last_vipc_frame_id = 0
  # Counter for the number of run iterations
  run_count = 0

  # Initialize transformation matrices for main and extra models to zeros
  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  # Flag to check if live calibration data has been seen
  live_calib_seen = False
  # Initialize navigation features and instructions to zeros
  nav_features = np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32)
  nav_instructions = np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32)
  # Initialize buffers for main and extra camera frames to None
  buf_main, buf_extra = None, None
  # Initialize metadata structures for main and extra frames
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  if demo:
    # Get car parameters for demo mode
    CP = get_demo_car_params()
  else:
    # Load car parameters from persistent storage if not in demo mode
    with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
      CP = msg
  # Log the retrieved car parameters
  cloudlog.info("modeld got CarParams: %s", CP.carName)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # Estimate steering actuator delay and add a buffer for other delays
  steer_delay = CP.steerActuatorDelay + .2

  # Initialize a helper for managing driver desires
  DH = DesireHelper()

  # Main processing loop
  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    # Keep receiving frames until we are at least 1 frame ahead of the previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      # Receive a frame from the main vision IPC client
      buf_main = vipc_client_main.recv()
      # Extract metadata from the received main frame
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        # If no frame is received, log an error and break the loop
        break

    if buf_main is None:
      # Log error if main camera client does not provide a frame
      cloudlog.error("vipc_client_main no frame")
      # Continue to the next iteration of the loop
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      # Keep receiving extra frames until the frame ID matches that of the main camera
      while True:
        # Receive a frame from the extra vision IPC client
        buf_extra = vipc_client_extra.recv()
        # Extract metadata from the received extra frame
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          # If no frame is received or if the main frame timestamp is ahead, break the loop
          break

      if buf_extra is None:
        # Log error if extra camera client does not provide a frame
        cloudlog.error("vipc_client_extra no frame")
        # Continue to the next iteration of the loop
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        # Log error if frames from main and extra cameras are out of sync
        cloudlog.error("frames out of sync! main: {} ({:.5f}), extra: {} ({:.5f})".format(
          meta_main.frame_id, meta_main.timestamp_sof / 1e9,
          meta_extra.frame_id, meta_extra.timestamp_sof / 1e9))

    else:
      # Use single camera
      # If not using an extra client, use the main camera frame for both
      buf_extra = buf_main
      meta_extra = meta_main

    # Update the subscriber with latest messages
    sm.update(0)
    # Retrieve the current desire from the DesireHelper
    desire = DH.desire
    # Determine if the driver is on the right-hand side of the vehicle
    is_rhd = sm["driverMonitoringState"].isRHD
    # Retrieve the frame ID from the road camera state
    frame_id = sm["roadCameraState"].frameId
    # Prepare lateral control parameters for the model
    lateral_control_params = np.array([sm["carState"].vEgo, steer_delay], dtype=np.float32)
    if sm.updated["liveCalibration"]:
      # If live calibration data is updated, compute warp matrices for model transformations
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_wide_camera, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, True, True).astype(np.float32)
      # Mark that live calibration data has been seen
      live_calib_seen = True

    # Initialize a zero vector for traffic convention
    traffic_convention = np.zeros(2)
    # Set the appropriate index to 1 based on the traffic convention (RHD or LHD)
    traffic_convention[int(is_rhd)] = 1

    # Initialize a zero vector for desires
    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      # Set the index corresponding to the current desire to 1
      vec_desire[desire] = 1

    # Enable/disable nav features
    # Determine if navigation features are enabled based on model and parameter settings
    timestamp_llk = sm["navModel"].locationMonoTime
    # Check if the navigation model data is valid
    nav_valid = sm.valid["navModel"] # and (nanos_since_boot() - timestamp_llk < 1e9)
    nav_enabled = nav_valid and (params.get_bool("ExperimentalMode") or params.get_bool("NavChill"))

    if not nav_enabled:
      # If navigation is not enabled, reset navigation features and instructions
      nav_features[:] = 0
      nav_instructions[:] = 0

    if nav_enabled and sm.updated["navModel"]:
      # If navigation is enabled and the navigation model is updated, load new features
      nav_features = np.array(sm["navModel"].features)

    if nav_enabled and sm.updated["navInstruction"]:
      # Reset navigation instructions to zeros
      nav_instructions[:] = 0
      # Iterate through all maneuvers in the navigation instructions
      for maneuver in sm["navInstruction"].allManeuvers:
        # Calculate the index based on the distance for the maneuver
        distance_idx = 25 + int(maneuver.distance / 20)
        # Initialize the direction index to 0 (straight)
        direction_idx = 0
        # Adjust the direction index based on the maneuver modifier
        if maneuver.modifier in ("left", "slight left", "sharp left"):
          direction_idx = 1
        if maneuver.modifier in ("right", "slight right", "sharp right"):
          direction_idx = 2
        # Only update if within the valid range
        if 0 <= distance_idx < 50:
          nav_instructions[distance_idx*3 + direction_idx] = 1

    # tracked dropped frames
    # Track dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      # Initially set dropped frames to 0 to stabilize
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    # Increment the run count after each loop iteration
    run_count = run_count + 1

    # Calculate the frame drop ratio for adaptive model input preparation
    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    # Skip model evaluation if frames were dropped to maintain timing
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      # Log an error if skipping model evaluation due to dropped frames
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    # Prepare inputs for the model including desires and traffic conventions
    inputs:Dict[str, np.ndarray] = {
      'desire': vec_desire,
      'traffic_convention': traffic_convention,
      'lateral_control_params': lateral_control_params,
      'nav_features': nav_features,
      'nav_instructions': nav_instructions}

    # Start timing model execution
    mt1 = time.perf_counter()
    # Run the model with the prepared inputs and buffers
    model_output = model.run(buf_main, buf_extra, model_transform_main, model_transform_extra, inputs, prepare_only)
    # Stop timing model execution
    mt2 = time.perf_counter()
    # Calculate the model execution time
    model_execution_time = mt2 - mt1

    if model_output is not None:
      # Prepare a new message for the model output
      modelv2_send = messaging.new_message('modelV2')
      # Prepare a new message for the pose output
      posenet_send = messaging.new_message('cameraOdometry')
      # Fill the model message with the model's output and state information
      fill_model_msg(modelv2_send, model_output, publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id, frame_drop_ratio,
                      meta_main.timestamp_eof, timestamp_llk, model_execution_time, nav_enabled, live_calib_seen)

      # Extract desire states from the model output for lane change decision making
      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      # Update the DesireHelper with the current state for lane changes
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, sm['frogpilotPlan'])
      # Update the model message with the lane change state and direction
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      # Also update the model message with the turn direction as determined by the DesireHelper
      modelv2_send.modelV2.meta.turnDirection = DH.turn_direction

      # Fill the pose message with the model's output, using various metadata and calibration flags
      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      # Send the filled modelV2 and cameraOdometry messages to their respective channels
      pm.send('modelV2', modelv2_send)
      pm.send('cameraOdometry', posenet_send)

    # Update the last VIPC frame ID for dropped frame tracking in the next iteration
    last_vipc_frame_id = meta_main.frame_id

# The entry point for running the script, handling command line arguments for demo mode
if __name__ == "__main__":
  try:
    # Parse command line arguments for demo mode
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    # Run the main function with the demo mode argument
    main(demo=args.demo)
  except KeyboardInterrupt:
    # Catch and log keyboard interrupt signals (e.g., CTRL+C)
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    # Capture and re-raise exceptions after logging them to Sentry
    sentry.capture_exception()
    raise
