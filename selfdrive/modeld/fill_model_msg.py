# Import necessary libraries and modules
import os
import capnp
import numpy as np
from typing import Dict
from cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan, Meta

# Check for environment variable to send raw predictions
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

# Assign ConfidenceClass for ease of reference
ConfidenceClass = log.ModelDataV2.ConfidenceClass

# Define PublishState class to manage publishing state information
class PublishState:
  # Initialize PublishState with zeros arrays for disengage buffer and previous brake probabilities
  def __init__(self):
    # Buffer for disengage probabilities
    self.disengage_buffer = np.zeros(ModelConstants.CONFIDENCE_BUFFER_LEN*ModelConstants.DISENGAGE_WIDTH, dtype=np.float32)
    # Previous probabilities for braking at 5 m/s^2
    self.prev_brake_5ms2_probs = np.zeros(ModelConstants.FCW_5MS2_PROBS_WIDTH, dtype=np.float32)
    # Previous probabilities for braking at 3 m/s^2
    self.prev_brake_3ms2_probs = np.zeros(ModelConstants.FCW_3MS2_PROBS_WIDTH, dtype=np.float32)

# Define a function to fill XYZT values in capnp builder
def fill_xyzt(builder, t, x, y, z, x_std=None, y_std=None, z_std=None):
  # Set builder fields for position and optionally standard deviations
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.z = z.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if z_std is not None:
    builder.zStd = z_std.tolist()

# Define a function to fill XYZVAT (velocity and acceleration) values in capnp builder
def fill_xyvat(builder, t, x, y, v, a, x_std=None, y_std=None, v_std=None, a_std=None):
  # Set builder fields for position, velocity, acceleration, and optionally their standard deviations
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.v = v.tolist()
  builder.a = a.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if v_std is not None:
    builder.vStd = v_std.tolist()
  if a_std is not None:
    builder.aStd = a_std.tolist()

# Define a function to fill the model message with output data and state
def fill_model_msg(msg: capnp._DynamicStructBuilder, net_output_data: Dict[str, np.ndarray], publish_state: PublishState,
                   vipc_frame_id: int, vipc_frame_id_extra: int, frame_id: int, frame_drop: float,
                   timestamp_eof: int, timestamp_llk: int, model_execution_time: float,
                   nav_enabled: bool, valid: bool) -> None:
  # Calculate frame age and set validity of the message
  frame_age = frame_id - vipc_frame_id if frame_id > vipc_frame_id else 0
  msg.valid = valid

  # Start filling in the modelV2 message fields with data
  modelV2 = msg.modelV2
  modelV2.frameId = vipc_frame_id
  modelV2.frameIdExtra = vipc_frame_id_extra
  modelV2.frameAge = frame_age
  modelV2.frameDropPerc = frame_drop * 100
  modelV2.timestampEof = timestamp_eof
  modelV2.locationMonoTime = timestamp_llk
  modelV2.modelExecutionTime = model_execution_time
  modelV2.navEnabled = nav_enabled

  # plan
  # Fill in positional data fields in the modelV2 message
  position = modelV2.position
  fill_xyzt(position, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.POSITION].T, *net_output_data['plan_stds'][0,:,Plan.POSITION].T)
  velocity = modelV2.velocity
  fill_xyzt(velocity, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.VELOCITY].T)
  acceleration = modelV2.acceleration
  fill_xyzt(acceleration, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ACCELERATION].T)
  orientation = modelV2.orientation
  fill_xyzt(orientation, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.T_FROM_CURRENT_EULER].T)
  orientation_rate = modelV2.orientationRate
  fill_xyzt(orientation_rate, ModelConstants.T_IDXS, *net_output_data['plan'][0,:,Plan.ORIENTATION_RATE].T)

  # lateral planning
  # Lateral planning: set the desired curvature for vehicle control
  action = modelV2.action
  action.desiredCurvature = float(net_output_data['desired_curvature'][0,0])

  # times at X_IDXS according to model plan
  # Compute times at specific X indices based on the model's plan for positional data
  PLAN_T_IDXS = [np.nan] * ModelConstants.IDX_N
  PLAN_T_IDXS[0] = 0.0
  plan_x = net_output_data['plan'][0,:,Plan.POSITION][:,0].tolist()
  for xidx in range(1, ModelConstants.IDX_N):
    tidx = 0
    # increment tidx until we find an element that's further away than the current xidx
    # Find the time index where the vehicle's position exceeds the current X index
    while tidx < ModelConstants.IDX_N - 1 and plan_x[tidx+1] < ModelConstants.X_IDXS[xidx]:
      tidx += 1
    if tidx == ModelConstants.IDX_N - 1:
      # if the Plan doesn't extend far enough, set plan_t to the max value (10s), then break
      # Use the maximum time if the plan doesn't extend far enough
      PLAN_T_IDXS[xidx] = ModelConstants.T_IDXS[ModelConstants.IDX_N - 1]
      break
    # interpolate to find `t` for the current xidx
    # Interpolate time for current X index based on plan positions
    current_x_val = plan_x[tidx]
    next_x_val = plan_x[tidx+1]
    p = (ModelConstants.X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val) if abs(next_x_val - current_x_val) > 1e-9 else float('nan')
    PLAN_T_IDXS[xidx] = p * ModelConstants.T_IDXS[tidx+1] + (1 - p) * ModelConstants.T_IDXS[tidx]

  # lane lines
  # Initialize lane lines data in the message and fill with model predictions
  modelV2.init('laneLines', 6)
  for i in range(6):
    # For the first four lane lines (typically the ego lane and adjacent lanes)
    if i < 4:
      lane_line = modelV2.laneLines[i]
      fill_xyzt(lane_line, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['lane_lines'][0,i,:,0], net_output_data['lane_lines'][0,i,:,1])
    # For additional lane lines, calculate the average positions
    else:
      lane_line = modelV2.laneLines[i]
      far_lane, near_lane, road_edge = (0, 1, 0) if i == 4 else (3, 2, 1)
      y_min = net_output_data['lane_lines'][0, near_lane,:,0]
      z_min = net_output_data['lane_lines'][0, near_lane,:,1]
      lane_diff = np.abs(net_output_data['lane_lines'][0,near_lane] - net_output_data['lane_lines'][0,far_lane])
      road_edge_diff = np.abs(net_output_data['lane_lines'][0,near_lane] - net_output_data['road_edges'][0,road_edge])
      y_min += np.where(lane_diff[:,0] < road_edge_diff[:,0], net_output_data['lane_lines'][0,far_lane,:,0], net_output_data['road_edges'][0,road_edge,:,0])
      z_min += np.where(lane_diff[:,1] < road_edge_diff[:,1], net_output_data['lane_lines'][0,far_lane,:,1], net_output_data['road_edges'][0,road_edge,:,1])
      y_min /= 2
      z_min /= 2
      fill_xyzt(lane_line, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), y_min, z_min)

  # Set standard deviations and probabilities for lane lines based on model predictions
  modelV2.laneLineStds = net_output_data['lane_lines_stds'][0,:,0,0].tolist()
  modelV2.laneLineProbs = net_output_data['lane_lines_prob'][0,1::2].tolist()

  # road edges
  # Initialize road edges data in the message and fill with model predictions
  modelV2.init('roadEdges', 2)
  for i in range(2):
    road_edge = modelV2.roadEdges[i]
    fill_xyzt(road_edge, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), net_output_data['road_edges'][0,i,:,0], net_output_data['road_edges'][0,i,:,1])
  modelV2.roadEdgeStds = net_output_data['road_edges_stds'][0,:,0,0].tolist()

  # leads
  # Initialize leads data in the message and fill with model predictions
  modelV2.init('leadsV3', 3)
  for i in range(3):
    lead = modelV2.leadsV3[i]
    fill_xyvat(lead, ModelConstants.LEAD_T_IDXS, *net_output_data['lead'][0,i].T, *net_output_data['lead_stds'][0,i].T)
    lead.prob = net_output_data['lead_prob'][0,i].tolist()
    lead.probTime = ModelConstants.LEAD_T_OFFSETS[i]

  # meta
  # Fill the meta information related to the model's output for engagement and prediction of disengagements
  meta = modelV2.meta
  meta.desireState = net_output_data['desire_state'][0].reshape(-1).tolist()
  meta.desirePrediction = net_output_data['desire_pred'][0].reshape(-1).tolist()
  meta.engagedProb = net_output_data['meta'][0,Meta.ENGAGED].item()
  meta.init('disengagePredictions')
  disengage_predictions = meta.disengagePredictions
  disengage_predictions.t = ModelConstants.META_T_IDXS
  disengage_predictions.brakeDisengageProbs = net_output_data['meta'][0,Meta.BRAKE_DISENGAGE].tolist()
  disengage_predictions.gasDisengageProbs = net_output_data['meta'][0,Meta.GAS_DISENGAGE].tolist()
  disengage_predictions.steerOverrideProbs = net_output_data['meta'][0,Meta.STEER_OVERRIDE].tolist()
  disengage_predictions.brake3MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_3].tolist()
  disengage_predictions.brake4MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_4].tolist()
  disengage_predictions.brake5MetersPerSecondSquaredProbs = net_output_data['meta'][0,Meta.HARD_BRAKE_5].tolist()

  # Update the publish state with the latest probabilities for hard braking
  publish_state.prev_brake_5ms2_probs[:-1] = publish_state.prev_brake_5ms2_probs[1:]
  publish_state.prev_brake_5ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_5][0]
  publish_state.prev_brake_3ms2_probs[:-1] = publish_state.prev_brake_3ms2_probs[1:]
  publish_state.prev_brake_3ms2_probs[-1] = net_output_data['meta'][0,Meta.HARD_BRAKE_3][0]
  # Evaluate if a hard brake event is predicted based on the set thresholds
  hard_brake_predicted = (publish_state.prev_brake_5ms2_probs > ModelConstants.FCW_THRESHOLDS_5MS2).all() and \
    (publish_state.prev_brake_3ms2_probs > ModelConstants.FCW_THRESHOLDS_3MS2).all()
  meta.hardBrakePredicted = hard_brake_predicted.item()

  # temporal pose
  # Fill in the temporal pose information from the simulation's output
  temporal_pose = modelV2.temporalPose
  temporal_pose.trans = net_output_data['sim_pose'][0,:3].tolist()
  temporal_pose.transStd = net_output_data['sim_pose_stds'][0,:3].tolist()
  temporal_pose.rot = net_output_data['sim_pose'][0,3:].tolist()
  temporal_pose.rotStd = net_output_data['sim_pose_stds'][0,3:].tolist()

  # confidence
  # Assess the confidence level based on disengage probabilities and update the modelV2 confidence
  if vipc_frame_id % (2*ModelConstants.MODEL_FREQ) == 0:
    # any disengage prob
    # Calculate any disengage probabilities
    brake_disengage_probs = net_output_data['meta'][0,Meta.BRAKE_DISENGAGE]
    gas_disengage_probs = net_output_data['meta'][0,Meta.GAS_DISENGAGE]
    steer_override_probs = net_output_data['meta'][0,Meta.STEER_OVERRIDE]
    any_disengage_probs = 1-((1-brake_disengage_probs)*(1-gas_disengage_probs)*(1-steer_override_probs))
    # independent disengage prob for each 2s slice
    # Determine independent disengage probabilities for each 2-second slice
    ind_disengage_probs = np.r_[any_disengage_probs[0], np.diff(any_disengage_probs) / (1 - any_disengage_probs[:-1])]
    # rolling buf for 2, 4, 6, 8, 10s
    # Update the rolling buffer for disengage probabilities
    publish_state.disengage_buffer[:-ModelConstants.DISENGAGE_WIDTH] = publish_state.disengage_buffer[ModelConstants.DISENGAGE_WIDTH:]
    publish_state.disengage_buffer[-ModelConstants.DISENGAGE_WIDTH:] = ind_disengage_probs

  # Calculate the overall confidence score and categorize it into green, yellow, or red
  score = 0.
  for i in range(ModelConstants.DISENGAGE_WIDTH):
    score += publish_state.disengage_buffer[i*ModelConstants.DISENGAGE_WIDTH+ModelConstants.DISENGAGE_WIDTH-1-i].item() / ModelConstants.DISENGAGE_WIDTH
  # Determine the model's confidence level based on the calculated score and assign a color indicator
  if score < ModelConstants.RYG_GREEN:
    # High confidence scenario
    modelV2.confidence = ConfidenceClass.green
  elif score < ModelConstants.RYG_YELLOW:
    # Medium confidence scenario
    modelV2.confidence = ConfidenceClass.yellow
  else:
    # Low confidence scenario indicating potential issues
    modelV2.confidence = ConfidenceClass.red

  # raw prediction if enabled
  # Include raw predictions in the message if the environment variable is set to enable this feature
  if SEND_RAW_PRED:
    # Convert raw predictions to bytes for transmission
    modelV2.rawPredictions = net_output_data['raw_pred'].tobytes()  

# Define a function to fill pose information in the message based on the model's output and calibration status
def fill_pose_msg(msg: capnp._DynamicStructBuilder, net_output_data: Dict[str, np.ndarray],
                  vipc_frame_id: int, vipc_dropped_frames: int, timestamp_eof: int, live_calib_seen: bool) -> None:
  # Checking for live calibration and no dropped frames to validate message
  msg.valid = live_calib_seen & (vipc_dropped_frames < 1)

  # Accessing the cameraOdometry part of the message for odometry data
  cameraOdometry = msg.cameraOdometry
  
  # Setting the frame ID related to this odometry data
  cameraOdometry.frameId = vipc_frame_id
  # Setting the end-of-frame timestamp for the odometry data
  cameraOdometry.timestampEof = timestamp_eof

  # Populating translation vector from model's output
  cameraOdometry.trans = net_output_data['pose'][0,:3].tolist()
  # Populating rotation vector from model's output
  cameraOdometry.rot = net_output_data['pose'][0,3:].tolist()

  # Setting device orientation in Euler angles from model's output
  cameraOdometry.wideFromDeviceEuler = net_output_data['wide_from_device_euler'][0,:].tolist()
  # Setting road transform translation vector from model's output
  cameraOdometry.roadTransformTrans = net_output_data['road_transform'][0,:3].tolist()

  # Including standard deviation of the translation vector to indicate measurement uncertainty
  cameraOdometry.transStd = net_output_data['pose_stds'][0,:3].tolist()
  # Including standard deviation of the rotation vector to indicate measurement uncertainty
  cameraOdometry.rotStd = net_output_data['pose_stds'][0,3:].tolist()
  # Including standard deviation of device orientation to indicate uncertainty
  cameraOdometry.wideFromDeviceEulerStd = net_output_data['wide_from_device_euler_stds'][0,:].tolist()
  # Including standard deviation of road transform translation vector to indicate uncertainty
  cameraOdometry.roadTransformTransStd = net_output_data['road_transform_stds'][0,:3].tolist()
