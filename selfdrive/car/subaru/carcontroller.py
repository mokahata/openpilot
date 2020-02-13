from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import CAR, DBC
from opendbc.can.packer import CANPacker



class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 2047                # max_steer 2047
    self.STEER_STEP = 2                  # how often we update the steer cmd
    self.STEER_DELTA_UP = 60             # torque increase per refresh, 0.68s to max
    self.STEER_DELTA_DOWN = 70           # torque decrease per refresh
    if car_fingerprint == CAR.IMPREZA:
      self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 10  # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1       # from dbc
    if car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      self.STEER_DRIVER_ALLOWANCE = 600  # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 1   # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1       # from dbc


class CarController():
  def __init__(self, car_fingerprint):
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.fake_button_prev = 0
    self.steer_rate_limited = False

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.params = CarControllerParams(car_fingerprint)
    self.packer = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []

    ### STEER ###

    if (frame % P.STEER_STEP) == 0:

      apply_steer = int(round(actuators.steer * P.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.steer_torque_driver, P)
      self.steer_rate_limited = new_steer != apply_steer

      lkas_enabled = enabled and not CS.steer_not_allowed

      if not lkas_enabled:
        apply_steer = 0
      self.apply_steer_last = apply_steer

      if not enabled:
        apply_steer = 0.

      can_sends.append(subarucan.create_steering_control(self.packer, CS.CP.carFingerprint, apply_steer, frame, P.STEER_STEP))

    ### DISENGAGE ###

    if self.car_fingerprint == CAR.IMPREZA:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        self.es_distance_cnt = CS.es_distance_msg["Counter"]
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd))

    ### ALERTS ###

    if self.car_fingerprint == CAR.IMPREZA:
      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, visual_alert, left_line, right_line))

    # button control
    if (frame % 5) == 0 and self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      # 1 = main, 2 = set shallow/slow down 1, 3 = set deep/slow down 10, 4 = resume shallow/speed up 1, 5 = resume deep/speed up 10
      fake_button = CS.button
      if enabled and (CS.v_ego_raw * CV.MS_TO_KPH) > 1 and (frame % 30) == 0:
        # alter target speed
        if actuators.brake > 0:
          target_speed = min(max(CS.stock_set_speed - int(actuators.brake * 30), 30), 145)
        else:
          target_speed = CS.v_cruise_pcm
        # set ACC speed to target speed
        if CS.stock_set_speed != target_speed:
          # if openpilot is higher than stock by 10 or greater
          if (target_speed - CS.stock_set_speed) >= 10:
            fake_button = 5
          # if openpilot is set higher than stock between 1 and 10
          elif 1 <= (target_speed - CS.stock_set_speed) < 10:
            fake_button = 4
          # if openpilot is lower than stock by 10 or greater
          elif (CS.stock_set_speed - target_speed) >= 10:
            fake_button = 3
          # if openpilot is lower than stock by 10 or less
          elif (CS.stock_set_speed - target_speed) < 10:
            fake_button = 2
          else:
            fake_button = CS.button

      ### DISENGAGE ###
      if pcm_cancel_cmd:
        fake_button = 1

      # cancel and main share the same button. this prevents accidental disabling of stock acc 
      if not CS.main_on and CS.ready:
        fake_button = 1

      # unstick previous mocked button press
      if fake_button != 0 and fake_button == self.fake_button_prev:
        fake_button = 0

      self.fake_button_prev = fake_button

      can_sends.append(subarucan.create_es_throttle_control(self.packer, fake_button, CS.es_accel_msg))

    return can_sends
