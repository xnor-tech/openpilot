import copy, math
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.rivian.values import DBC, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
MAX_SET_SPEED = 37.99  # m/s
MIN_SET_SPEED = 8.94  # m/s

class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.set_speed = 10
    self.increase_btn_pressed_prev = False
    self.increase_cntr = 0
    self.decrease_btn_pressed_prev = False
    self.decrease_cntr = 0
    self.distance_button = 0

    self.acm_lka_hba_cmd = None
    self.sccm_wheel_touch = None
    self.vdm_adas_status = None

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]
    cp_park = can_parsers[Bus.alt]
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

    # Button logic
    increase_btn_pressed_now = cp_park.vl["WheelButtons"]["RightButton_RightClick"] == 2
    decrease_btn_pressed_now = cp_park.vl["WheelButtons"]["RightButton_LeftClick"] == 2

    self.increase_cntr = self.increase_cntr + 1 if increase_btn_pressed_now else 0
    self.decrease_cntr = self.decrease_cntr + 1 if decrease_btn_pressed_now else 0

    set_speed_mph = self.set_speed * CV.MS_TO_MPH
    # Check if increase button was pressed in the previous frame and is not pressed now (rising edge)
    if increase_btn_pressed_now and self.increase_cntr % 66 == 0:
      self.set_speed = (int(math.ceil((set_speed_mph + 1) / 5.0)) * 5) * CV.MPH_TO_MS
    elif not self.increase_btn_pressed_prev and increase_btn_pressed_now:
      self.set_speed += 1 * CV.MPH_TO_MS

    # Check if decrease button was pressed in the previous frame and is not pressed now (rising edge)
    if decrease_btn_pressed_now and self.decrease_cntr % 66 == 0:
      self.set_speed = (int(math.floor((set_speed_mph - 1) / 5.0)) * 5) * CV.MPH_TO_MS
    elif not self.decrease_btn_pressed_prev and decrease_btn_pressed_now:
      self.set_speed -= 1 * CV.MPH_TO_MS

    # Update previous button states for the next iteration
    self.increase_btn_pressed_prev = increase_btn_pressed_now
    self.decrease_btn_pressed_prev = decrease_btn_pressed_now

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
    ret.leftBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Left"] != 0
    ret.rightBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Right"] != 0

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # distance scroll wheel
    right_scroll = cp_park.vl["WheelButtons"]["RightButton_Scroll"]
    if right_scroll != 255:
      prev_distance_button = self.distance_button
      self.distance_button = right_scroll
      pressed = prev_distance_button != self.distance_button
      # ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=pressed, type=ButtonType.gapAdjustCruise)]

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

    alt_messages = [
      ("WheelButtons", 20),
      ("BSM_BlindSpotIndicator", 20),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.alt], alt_messages, 5),
    }
