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
  cloudlog.warning("modeld init")

  sentry.set_tag("daemon", PROCESS_NAME)
  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)
  config_realtime_process(7, 54)

  cloudlog.warning("setting up CL context")
  cl_context = CLContext()
  cloudlog.warning("CL context ready; loading model")
  model = ModelState(cl_context)
  cloudlog.warning("models loaded, modeld starting")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False, cl_context)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  # messaging
  pm = PubMaster(["modelV2", "cameraOdometry"])
  sm = SubMaster(["carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "navModel", "navInstruction", "carControl", "frogpilotPlan"])

  publish_state = PublishState()
  params = Params()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  nav_features = np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32)
  nav_instructions = np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32)
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  if demo:
    CP = get_demo_car_params()
  else:
    with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
      CP = msg
  cloudlog.info("modeld got CarParams: %s", CP.carName)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  steer_delay = CP.steerActuatorDelay + .2

  DH = DesireHelper()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.error("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.error("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error("frames out of sync! main: {} ({:.5f}), extra: {} ({:.5f})".format(
          meta_main.frame_id, meta_main.timestamp_sof / 1e9,
          meta_extra.frame_id, meta_extra.timestamp_sof / 1e9))

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    lateral_control_params = np.array([sm["carState"].vEgo, steer_delay], dtype=np.float32)
    if sm.updated["liveCalibration"]:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_wide_camera, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, True, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # Enable/disable nav features
    timestamp_llk = sm["navModel"].locationMonoTime
    nav_valid = sm.valid["navModel"] # and (nanos_since_boot() - timestamp_llk < 1e9)
    nav_enabled = nav_valid and (params.get_bool("ExperimentalMode") or params.get_bool("NavChill"))

    if not nav_enabled:
      nav_features[:] = 0
      nav_instructions[:] = 0

    if nav_enabled and sm.updated["navModel"]:
      nav_features = np.array(sm["navModel"].features)

    if nav_enabled and sm.updated["navInstruction"]:
      nav_instructions[:] = 0
      for maneuver in sm["navInstruction"].allManeuvers:
        distance_idx = 25 + int(maneuver.distance / 20)
        direction_idx = 0
        if maneuver.modifier in ("left", "slight left", "sharp left"):
          direction_idx = 1
        if maneuver.modifier in ("right", "slight right", "sharp right"):
          direction_idx = 2
        if 0 <= distance_idx < 50:
          nav_instructions[distance_idx*3 + direction_idx] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    inputs:Dict[str, np.ndarray] = {
      'desire': vec_desire,
      'traffic_convention': traffic_convention,
      'lateral_control_params': lateral_control_params,
      'nav_features': nav_features,
      'nav_instructions': nav_instructions}

    mt1 = time.perf_counter()
    model_output = model.run(buf_main, buf_extra, model_transform_main, model_transform_extra, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      posenet_send = messaging.new_message('cameraOdometry')
      fill_model_msg(modelv2_send, model_output, publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id, frame_drop_ratio,
                      meta_main.timestamp_eof, timestamp_llk, model_execution_time, nav_enabled, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, sm['frogpilotPlan'])
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      modelv2_send.modelV2.meta.turnDirection = DH.turn_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('cameraOdometry', posenet_send)

    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    sentry.capture_exception()
    raise
