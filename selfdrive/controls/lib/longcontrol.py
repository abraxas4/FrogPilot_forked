from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state

# The LongControl class handles the longitudinal control, which involves maintaining
# or changing the speed of the vehicle through acceleration and braking.
class LongControl:
  # Initialize the LongControl class with the car's parameters.
  def __init__(self, CP):
    # Store car parameters for use in control logic.
    # 1. Immutability : immutable throughout the instance's lifecycle. (accidentally modified elsewhere in the code won't affect here)
    # 2. Readability : makes the code easier to read without having to track where and how CP is modified elsewhere.
    # 3. Encapsulation : helps to keep the code modular and maintainable.
    self.CP = CP
    # Set initial state of longitudinal control to off.
    self.long_control_state = LongCtrlState.off  # initialized to off
    # Initialize PID controller with car's longitudinal tuning parameters.
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    # Initialize target PID speed to zero.
    self.v_pid = 0.0
    # Initialize last output acceleration to zero.
    self.last_output_accel = 0.0

  # Reset the PID controller with a new setpoint.
  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    # Reset the internal state of the PID controller.
    self.pid.reset()
    # Update the target PID speed.
    self.v_pid = v_pid

  # Update longitudinal control logic each time step.
  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory    
    # Interpolate the target speed and acceleration over the trajectory. (have a smooth and continuous control signal, rather than a step-like signal)
    speeds = long_plan.speeds
    # Ensure we have a full trajectory to work with.
    if len(speeds) == CONTROL_N:
      # Determine the current target speed based on the elapsed time.
      v_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      # Determine the current target acceleration based on the elapsed time.
      a_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], long_plan.accels)

      # Calculate the lower bounds of the target speed and acceleration considering actuator delay.
      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      # Calculate the upper bounds of the target speed and acceleration considering actuator delay.
      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      # Select the most conservative targets to avoid overshoot.
      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      # Determine a one-second-ahead target speed to anticipate future conditions.
      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, ModelConstants.T_IDXS[:CONTROL_N], speeds)
    else:
      # If trajectory data is incomplete, default to stationary targets.
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    # Set the PID controller's acceleration limits.
    self.pid.neg_limit = accel_limits[0]
    # Define a maximum acceleration limit to avoid uncomfortable jolts.
    MAX_ACCEL_LIMIT = 0.4  # Maximum acceleration limit in m/s²
    # Ensure the PID controller's positive limit does not exceed the maximum or the car's limit.
    self.pid.pos_limit = min(accel_limits[1], MAX_ACCEL_LIMIT)  # MAX_ACCEL_LIMIT is your desired maximum acceleration

    # Store the last output for use in state logic.
    output_accel = self.last_output_accel
    # Update the state machine of the longitudinal controller based on various factors.
    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill)

    # Logic for when the longitudinal control state is 'off'.
    if self.long_control_state == LongCtrlState.off:
      # Reset the controller and set acceleration to zero.
      self.reset(CS.vEgo)
      output_accel = 0.

    # Logic for when the car is in a stopping state.
    elif self.long_control_state == LongCtrlState.stopping:
      # If the current acceleration is above the stopping acceleration, reduce it smoothly.
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      # Reset the controller for the next state.
      self.reset(CS.vEgo)

    # Logic for when the car is in a starting state.
    elif self.long_control_state == LongCtrlState.starting:
      # Set a fixed acceleration to initiate motion.
      output_accel = self.CP.startAccel
      # Reset the controller for consistent behavior.
      self.reset(CS.vEgo)

    # Logic for when the car is being controlled by the PID loop.
    elif self.long_control_state == LongCtrlState.pid:
      # Set the target speed for the PID loop.
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      
      # Additional logic for Toyota vehicles to handle overshoot scenarios.
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      # Calculate a deadzone to prevent controller wind-up.
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      # Decide whether to freeze the integrator.
      freeze_integrator = prevent_overshoot

      # Calculate the error between target and actual speed.
      error = self.v_pid - CS.vEgo
      # Apply deadzone to error to avoid controller activity at small errors.
      error_deadzone = apply_deadzone(error, deadzone)
      # Update the PID controller output based on the error.
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      # Make sure the output does not exceed the maximum acceleration limit.
      output_accel = min(output_accel, MAX_ACCEL_LIMIT)

    # Clip the acceleration output to stay within vehicle limits.
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    # Return the calculated acceleration command for the vehicle to follow.
    return self.last_output_accel
