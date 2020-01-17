import copy
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from opendbc.can.parser import CANParser
from selfdrive.car.subaru.values import CAR, DBC, STEER_THRESHOLD

def get_powertrain_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("Steer_Torque_Sensor", "Steering_Torque", 0),
    ("Steering_Angle", "Steering_Torque", 0),
    ("Steer_Torque_Output", "Steering_Torque", 0),
    ("Cruise_On", "CruiseControl", 0),
    ("Cruise_Activated", "CruiseControl", 0),
    ("Brake_Pedal", "Brake_Pedal", 0),
    ("Throttle_Pedal", "Throttle", 0),
    ("LEFT_BLINKER", "Dashlights", 0),
    ("RIGHT_BLINKER", "Dashlights", 0),
    ("SEATBELT_FL", "Dashlights", 0),

    ("FL", "Wheel_Speeds", 0),
    ("FR", "Wheel_Speeds", 0),
    ("RL", "Wheel_Speeds", 0),
    ("RR", "Wheel_Speeds", 0),
    ("DOOR_OPEN_FR", "BodyInfo", 1),
    ("DOOR_OPEN_FL", "BodyInfo", 1),
    ("DOOR_OPEN_RR", "BodyInfo", 1),
    ("DOOR_OPEN_RL", "BodyInfo", 1),
  ]

  checks = [
    # sig_address, frequency
    ("Dashlights", 10),
    ("Wheel_Speeds", 50),
    ("Steering_Torque", 50),
  ]

  if CP.carFingerprint == CAR.IMPREZA:
    signals += [
      ("Units", "Dash_State", 1),
    ]
    checks += [
      ("BodyInfo", 10),
      ("CruiseControl", 20),
    ]

  if CP.carFingerprint in [CAR.OUTBACK, CAR.LEGACY]:
    signals += [
      ("LKA_Lockout", "Steering_Torque", 0),
    ]
    checks += [
      ("CruiseControl", 50),
    ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

def get_camera_can_parser(CP):
  signals = [
    ("Cruise_Set_Speed", "ES_DashStatus", 0),
  ]

  checks = [
    ("ES_DashStatus", 10),
  ]

  if CP.carFingerprint == CAR.IMPREZA:
    signals += [
      ("Counter", "ES_Distance", 0),
      ("Signal1", "ES_Distance", 0),
      ("Signal2", "ES_Distance", 0),
      ("Main", "ES_Distance", 0),
      ("Signal3", "ES_Distance", 0),

      ("Checksum", "ES_LKAS_State", 0),
      ("Counter", "ES_LKAS_State", 0),
      ("Keep_Hands_On_Wheel", "ES_LKAS_State", 0),
      ("Empty_Box", "ES_LKAS_State", 0),
      ("Signal1", "ES_LKAS_State", 0),
      ("LKAS_ACTIVE", "ES_LKAS_State", 0),
      ("Signal2", "ES_LKAS_State", 0),
      ("Backward_Speed_Limit_Menu", "ES_LKAS_State", 0),
      ("LKAS_ENABLE_3", "ES_LKAS_State", 0),
      ("Signal3", "ES_LKAS_State", 0),
      ("LKAS_ENABLE_2", "ES_LKAS_State", 0),
      ("Signal4", "ES_LKAS_State", 0),
      ("LKAS_Left_Line_Visible", "ES_LKAS_State", 0),
      ("Signal6", "ES_LKAS_State", 0),
      ("LKAS_Right_Line_Visible", "ES_LKAS_State", 0),
      ("Signal7", "ES_LKAS_State", 0),
      ("FCW_Cont_Beep", "ES_LKAS_State", 0),
      ("FCW_Repeated_Beep", "ES_LKAS_State", 0),
      ("Throttle_Management_Activated", "ES_LKAS_State", 0),
      ("Traffic_light_Ahead", "ES_LKAS_State", 0),
      ("Right_Depart", "ES_LKAS_State", 0),
      ("Signal5", "ES_LKAS_State", 0),
    ]

  if CP.carFingerprint in [CAR.OUTBACK, CAR.LEGACY]:
    signals += [
      ("Brake_On", "ES_CruiseThrottle", 0),
      ("Button", "ES_CruiseThrottle", 0),
      ("CloseDistance", "ES_CruiseThrottle", 0),
      ("Counter", "ES_CruiseThrottle", 0),
      ("Cruise_Activatedish", "ES_CruiseThrottle", 0),
      ("DistanceSwap", "ES_CruiseThrottle", 0),
      ("ES_Error", "ES_CruiseThrottle", 0),
      ("NEW_SIGNAL_1", "ES_CruiseThrottle", 0),
      ("Unknown", "ES_CruiseThrottle", 0),
      ("SET_0_1", "ES_CruiseThrottle", 0),
      ("SET_0_2", "ES_CruiseThrottle", 0),
      ("SET_0_3", "ES_CruiseThrottle", 0),
      ("SET_0_4", "ES_CruiseThrottle", 0),
      ("SET_1", "ES_CruiseThrottle", 0),
      ("SET_2", "ES_CruiseThrottle", 0),
      ("NEW_SIGNAL_9", "ES_CruiseThrottle", 0),
      ("Standstill", "ES_CruiseThrottle", 0),
      ("Standstill_2", "ES_CruiseThrottle", 0),
      ("Throttle_Cruise", "ES_CruiseThrottle", 0),
      ("Not_Ready_Startup", "ES_DashStatus", 0),
    ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)


class CarState():
  def __init__(self, CP):
    # initialize can parser
    self.CP = CP

    self.car_fingerprint = CP.carFingerprint
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_not_allowed = False
    if self.car_fingerprint in [CAR.OUTBACK, CAR.LEGACY]:
      self.v_cruise_pcm = 0
      self.button_prev = 0
      self.button_count = 0
      self.acc_active_prev = 0

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=[[0.], [0.]],
                         A=[[1., dt], [0., 1.]],
                         C=[1., 0.],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.

  def update(self, cp, cp_cam):

    self.pedal_gas = cp.vl["Throttle"]['Throttle_Pedal']
    self.brake_pressure = cp.vl["Brake_Pedal"]['Brake_Pedal']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    self.v_wheel_fl = cp.vl["Wheel_Speeds"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["Wheel_Speeds"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["Wheel_Speeds"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["Wheel_Speeds"]['RR'] * CV.KPH_TO_MS

    v_wheel = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.
    # Kalman filter, even though Subaru raw wheel speed is heaviliy filtered by default
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)

    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = cp.vl["Dashlights"]['LEFT_BLINKER'] == 1
    self.right_blinker_on = cp.vl["Dashlights"]['RIGHT_BLINKER'] == 1
    self.steer_torque_driver = cp.vl["Steering_Torque"]['Steer_Torque_Sensor']
    self.steer_torque_motor = cp.vl["Steering_Torque"]['Steer_Torque_Output']
    self.acc_active = cp.vl["CruiseControl"]['Cruise_Activated']
    self.main_on = cp.vl["CruiseControl"]['Cruise_On']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.car_fingerprint]
    self.angle_steers = cp.vl["Steering_Torque"]['Steering_Angle']
    self.door_open = any([cp.vl["BodyInfo"]['DOOR_OPEN_RR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_RL'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FL']])

    if self.car_fingerprint == CAR.IMPREZA:
      self.seatbelt_unlatched = cp.vl["Dashlights"]['SEATBELT_FL'] == 1
      self.v_cruise_pcm = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.MPH_TO_KPH
      self.steer_not_allowed = 0
      self.es_distance_msg = copy.copy(cp_cam.vl["ES_Distance"])
      self.es_lkas_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
      # 1 = imperial, 6 = metric
      if cp.vl["Dash_State"]['Units'] == 1:
        self.v_cruise_pcm *= CV.MPH_TO_KPH

    if self.car_fingerprint in [CAR.OUTBACK, CAR.LEGACY]:
      self.seatbelt_unlatched = False
      self.steer_not_allowed = cp.vl["Steering_Torque"]["LKA_Lockout"]
      self.button = cp_cam.vl["ES_CruiseThrottle"]["Button"]
      self.brake_hold = cp_cam.vl["ES_CruiseThrottle"]["Standstill"]
      self.close_distance = cp_cam.vl["ES_CruiseThrottle"]["CloseDistance"]
      self.es_accel_msg = copy.copy(cp_cam.vl["ES_CruiseThrottle"])
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]

      # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
      self.stock_set_speed = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"]
      # keep set speed within stock subaru limits
      if self.acc_active and 145 < self.v_cruise_pcm < 30:
        self.v_cruise_pcm = self.stock_set_speed
      
      if self.acc_active and self.button in [2,3,4,5] and self.button_count == 0:
        if self.button in [2, 4] and not self.button_prev in [3,5]:
          if self.button == 2:
          # round down to the nearest 5 
            self.v_cruise_pcm = (int(self.v_cruise_pcm / 5) - 1) * 5
          else:    # button 4
          # round up to the nearest 5 
            self.v_cruise_pcm = (int(self.v_cruise_pcm / 5) + 1) * 5
        elif self.button == 3:
          # round down to the nearest 10
          self.v_cruise_pcm = (int(self.v_cruise_pcm / 10) - 1) * 10
        else:      # button 5
          # round up to the nearest 10
          self.v_cruise_pcm = (int(self.v_cruise_pcm / 10) + 1) * 10
      #set cruise speed to current speed
      if self.acc_active and not self.acc_active_prev:
        self.v_cruise_pcm = max(self.v_ego_raw * CV.MS_TO_KPH, 30)
      # round to the lowest multiple of 5 if set speed is not a multiple of 5
      if self.v_cruise_pcm % 5 != 0:
        self.v_cruise_pcm = (int(self.v_cruise_pcm / 5) - 1) * 5
      # set speed delay at 2.5hz instead of 100hz to avoid unintended button pressing 
      if self.button == self.button_prev and self.button_count <= 40:
        self.button_count =+ 1
      else:
        self.button_count = 0 
      self.button_prev = self.button
      self.acc_active_prev = self.acc_active
      # set v_cruise_pcm to a reasonable value when main is off
      if not self.main_on:
        self.v_cruise_pcm = self.stock_set_speed
