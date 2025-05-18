import copy
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.rivian.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV
from cereal import messaging
from openpilot.common.params import Params

GearShifter = structs.CarState.GearShifter

MAX_SET_SPEED = 37.99  # m/s
MIN_SET_SPEED = 8.94  # m/s


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.sm = messaging.SubMaster(['uiSetSpeed'])
    self.button_hold_frames = 0
    self.last_button_press = -1
    self.set_speed = 10
    self.sign_speed = 10

    self.acm_lka_hba_cmd = None
    self.sccm_wheel_touch = None
    self.vdm_adas_status = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_Status"]["ESP_Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = abs(ret.vEgoRaw) < 0.01

    # Gas pedal
    pedal_status = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = pedal_status > 0

    # Brake pedal
    ret.brake = cp.vl["ESPiB3"]["ESPiB3_pMC1"] / 250.0  # pressure in Bar
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp.vl["EPAS_AdasStatus"]["EPAS_EacErrorCode"] != 0

    # Cruise state
    ret.cruiseState.enabled = cp_cam.vl["ACM_Status"]["ACM_FeatureStatus"] == 1

    # Button Logic
    self.sm.update(0)
    button_press = int(self.sm["uiSetSpeed"].buttonSignal)
    if button_press == self.last_button_press:
      self.button_hold_frames += 1
    else:
      self.button_hold_frames = 0
    self.last_button_press = button_press

    if self.button_hold_frames % 15 == 1 and button_press:
      self.set_speed += button_press * CV.MPH_TO_MS

    # Traffic Sign Detection
    current_sign_speed = int(cp_adas.vl["ACM_tsrCmd"]["ACM_tsrSpdDisClsMain"])
    if current_sign_speed in [0, 254, 255]:  # 0=No Recognition, 254=Reserved, 255=Invalid
      # If the speed sign is invalid, use the last valid detected speed.
      current_sign_speed = self.sign_speed
    else:
      current_sign_speed = min(current_sign_speed * CV.MPH_TO_MS, MAX_SET_SPEED)  # 253=Unlimited_Speed

    # Use the detected traffic sign speed only if it's a new value and within a reasonable range
    # (approximately +/- ~20MPH) of the current vehicle speed to avoid false positives.
    if self.sign_speed != current_sign_speed and abs(self.set_speed - current_sign_speed) <= 9:
      offset = int(Params().get("TrafficSignOffset"))
      if offset != 0:
       self.set_speed = current_sign_speed * (1 + ((5 * offset) / 100))
    self.sign_speed = current_sign_speed

    # If the driver is pressing the gas pedal and the vehicle speed exceeds the current set speed,
    if ret.gasPressed and ret.vEgo > self.set_speed:
      self.set_speed = ret.vEgo

    if not ret.cruiseState.enabled:
      self.set_speed = ret.vEgo

    self.set_speed = max(MIN_SET_SPEED, min(self.set_speed, MAX_SET_SPEED))
    ret.cruiseState.speed = self.set_speed

    ret.cruiseState.available = True  # cp.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = cp.vl["VDM_AdasSts"]["VDM_AdasAccelRequestAcknowledged"] == 1

    # TODO: log ACM_Unkown2=3 as a fault. need to filter it at the start and end of routes though
    # ACM_FaultStatus hasn't been seen yet
    ret.accFaulted = (cp_cam.vl["ACM_Status"]["ACM_FaultStatus"] == 1 or
                      # VDM_AdasFaultStatus=Brk_Intv is the default for some reason
                      # VDM_AdasFaultStatus=Imps_Cmd was seen when sending it rapidly changing ACC enable commands
                      # VDM_AdasFaultStatus=Cntr_Fault isn't fully understood, but we've seen it in the wild
                      cp.vl["VDM_AdasSts"]["VDM_AdasFaultStatus"] in (3,))  # 3=Imps_Cmd

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"]), GearShifter.unknown)

    # Doors
    ret.doorOpen = (cp_adas.vl["IndicatorLights"]["RearDriverDoor"] != 2 or
                    cp_adas.vl["IndicatorLights"]["FrontPassengerDoor"] != 2 or
                    cp_adas.vl["IndicatorLights"]["DriverDoor"] != 2 or
                    cp_adas.vl["IndicatorLights"]["RearPassengerDoor"] != 2)

    # Blinkers
    ret.leftBlinker = cp_adas.vl["IndicatorLights"]["TurnLightLeft"] in (1, 2)
    ret.rightBlinker = cp_adas.vl["IndicatorLights"]["TurnLightRight"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # Blindspot
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # Messages needed by carcontroller
    self.acm_lka_hba_cmd = copy.copy(cp_cam.vl["ACM_lkaHbaCmd"])
    self.sccm_wheel_touch = copy.copy(cp.vl["SCCM_WheelTouch"])
    self.vdm_adas_status = copy.copy(cp.vl["VDM_AdasSts"])

    return ret

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      # sig_address, frequency
      ("ESP_Status", 50),
      ("VDM_PropStatus", 50),
      ("iBESP2", 50),
      ("ESPiB3", 50),
      ("EPAS_AdasStatus", 100),
      ("EPAS_SystemStatus", 100),
      ("RCM_Status", 8),
      ("VDM_AdasSts", 100),
      ("SCCM_WheelTouch", 20),
    ]

    cam_messages = [
      ("ACM_longitudinalRequest", 100),
      ("ACM_AebRequest", 100),
      ("ACM_Status", 100),
      ("ACM_lkaHbaCmd", 100)
    ]

    adas_messages = [
      ("IndicatorLights", 10),
      ("ACM_tsrCmd", 10),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
