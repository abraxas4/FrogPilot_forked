from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

class CarController:
  # The constructor for CarController initializes various components for controlling the car.
  def __init__(self, dbc_name, CP, VM):
    # Store the car's parameters for easy access throughout the class.
    self.CP = CP
    # Set up a CanBus object to handle sending and receiving CAN messages.
    self.CAN = CanBus(CP)
    # Create an object to hold parameters specific to the car's controller.
    self.params = CarControllerParams(CP)
    # Initialize a CAN message packer to construct CAN messages using a dbc file.
    self.packer = CANPacker(dbc_name)
    # Counter used to prevent steering at too high of an angle, which could cause faults.
    self.angle_limit_counter = 0
    # A frame counter that keeps track of the number of control iterations.
    self.frame = 0

    # Variables to store the last known values of acceleration and steering for smooth transitions.
    self.accel_last = 0
    self.apply_steer_last = 0
    # The car's unique fingerprint used to identify the specific make, model, and features.
    self.car_fingerprint = CP.carFingerprint
    # The last frame number when a button press was sent, to avoid spamming button presses.
    self.last_button_frame = 0

  # The update function sends CAN messages to control the car based on driver and Openpilot inputs.
  def update(self, CC, CS, now_nanos, frogpilot_variables):
    # Retrieve the desired actuator commands from CarControl.
    actuators = CC.actuators
    # HUD control information for display alerts and warnings on the car's instrument cluster.
    hud_control = CC.hudControl

    # Calculate the new desired steering torque as an integer for CAN message encoding.
    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    # Apply driver steering torque limits to prevent unexpected behavior.
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
    
    # >90 degree steering fault prevention
    # Implement fault prevention logic to avoid steering faults when the angle exceeds a threshold.
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    # Disable steering if lateral control is inactive.
    if not CC.latActive:
      apply_steer = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # Induce a temporary fault to maintain steering torque when necessary.
    torque_fault = CC.latActive and not apply_steer_req

    # Maintain a history of the last applied steering value.
    self.apply_steer_last = apply_steer

    # accel + longitudinal
    # Determine the appropriate acceleration command, adjusting for sport-plus mode if enabled.
    if frogpilot_variables.sport_plus:
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX_PLUS)
    else:
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    # Identify if the car is in a stopping state for appropriate acceleration handling.
    stopping = actuators.longControlState == LongCtrlState.stopping
    # Convert the set speed to the appropriate units based on the car's system (metric or imperial).
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    # Process HUD alerts to display warnings and state information to the driver.
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    # Initialize a list to accumulate the CAN messages that need to be sent.
    can_sends = []

    # This section of the code handles common Hyundai-specific tasks such as sending tester present messages
    # to various ECUs to ensure they stay in the desired state. Additionally, for CAN-FD cars, messages are created
    # for steering control, suppressing LFA (Lane Following Assist), setting cluster icons for LFA and HDA (Highway Driving Assist),
    # handling turn signals (blinkers), and controlling acceleration and cruise control.

    # Each message generated is appended to the can_sends list, which is then returned from the update function along with the new actuator states.
    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append([0x7b1, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", self.CAN.ECAN])

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_steer))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units, CS.personality_profile))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control.leadVisible, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca, CS.out.cruiseState.available, CS.personality_profile))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaican.create_acc_opt(self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel
    # Note: The create_button_messages function handles the creation of CAN messages for button presses,
    # such as canceling or resuming cruise control, which are added to the can_sends list.

    # Return the updated actuator states and the list of CAN messages to be sent out.
    self.frame += 1
    return new_actuators, can_sends
    
  # The create_button_messages function is responsible for creating CAN messages related to button presses on the steering wheel.
  def create_button_messages(self, CC: car.CarControl, CS: car.CarState, use_clu11: bool):
    # Initialize a list to hold the CAN messages for button press actions.
    can_sends = []
    # When CLU11 messaging is used (certain Hyundai models), handle button presses differently.
    if use_clu11:
      # If the cancel button is pressed, send the cancel command to the car.
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
      # If the resume button is pressed, send multiple resume messages to ensure the command is registered.
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        # Send resume at a max frequency of 10Hz to prevent spamming the CAN bus.
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          # Send 25 messages at a time to increase the likelihood of the resume command being accepted.
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
          # Update the last button frame to throttle the message rate.
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      # For non-CLU11 systems, handle button presses at a lower rate.
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        # If the cancel button is pressed, create and send the cancel message.
        if CC.cruiseControl.cancel:
          # If alternate button messaging is enabled, use the specified cancel command.
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            # Otherwise, send the standard cancel button message.
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        # If the resume button is pressed, create and send the resume message.
        elif CC.cruiseControl.resume:
          # If alternate button messaging is enabled, handle resume for alt button cars (specific logic to be determined).
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            # TODO: Define resume logic for alternate button systems.
            pass
          else:
            # Send the standard resume button message.
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    # Return the list of button press CAN messages to be sent.
    return can_sends
