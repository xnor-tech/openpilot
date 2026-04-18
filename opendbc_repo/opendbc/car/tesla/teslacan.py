from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  return ctrl_type


def _crc8_j1850(data: bytes) -> int:
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) if (crc & 0x80) else (crc << 1)
      crc &= 0xFF
  return crc ^ 0xFF


def create_fake_das_msg(pedal_enabled: bool, autopilot_disabled: bool, bus: int,
                        stalk_main: bool = False, stalk_cancel: bool = False):
  dat = bytearray(8)
  dat[5] = ((0x20 if pedal_enabled else 0) |
            (0x80 if autopilot_disabled else 0) |
            (0x02 if stalk_main else 0) |
            (0x01 if stalk_cancel else 0))
  return (0x659, bytes(dat), bus)


def create_fake_das_message(pedal_enabled: bool, autopilot_disabled: bool, *,
                            stalk_main: bool = False, stalk_cancel: bool = False,
                            bus: int = 0):
  return create_fake_das_msg(pedal_enabled, autopilot_disabled, bus,
                             stalk_main=stalk_main, stalk_cancel=stalk_cancel)


class TeslaCAN:
  def __init__(self, *args):
    if len(args) == 2:
      self.CP, self.packer = args
    elif len(args) == 1:
      self.CP, self.packer = None, args[0]
    else:
      raise TypeError("TeslaCAN expects (packer) or (CP, packer)")
    self.CCP = CarControllerParams
    self.jerk_upper = self.CCP.JERK_LIMIT_MAX
    self.jerk_lower = self.CCP.JERK_LIMIT_MIN

  def _create_fake_das(self, pedal_enabled: bool, autopilot_disabled: bool, bus: int,
                       stalk_main: bool = False, stalk_cancel: bool = False):
    return create_fake_das_msg(pedal_enabled, autopilot_disabled, bus,
                               stalk_main=stalk_main, stalk_cancel=stalk_cancel)

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(getattr(self.CP, "flags", 0), 1 if enabled else 0),
    }
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active,
                                  gas_pressed=False, set_speed_kph: float | None = None):
    from opendbc.car.interfaces import V_CRUISE_MAX

    if set_speed_kph is not None:
      set_speed = float(max(0.0, min(float(set_speed_kph), V_CRUISE_MAX)))
    else:
      set_speed = max(v_ego * CV.MS_TO_KPH, 0.0)
      if active:
        set_speed = 0.0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
    }
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_stalk_request(self, bus: int, msg_stw_actn_req: dict | None, *,
                           cruise_button: int | None = None,
                           turn_signal_stalk_state: int | None = None) -> tuple[int, bytes, int]:
    values = dict(msg_stw_actn_req or {})
    if cruise_button is not None:
      values["SpdCtrlLvr_Stat"] = int(cruise_button)
    if turn_signal_stalk_state is not None:
      values["TurnIndLvr_Stat"] = int(turn_signal_stalk_state)

    counter = (int(values.get("MC_STW_ACTN_RQ", 0)) + 1) % 16
    values["MC_STW_ACTN_RQ"] = counter
    values["CRC_STW_ACTN_RQ"] = 0

    msg = self.packer.make_can_msg("STW_ACTN_RQ", int(bus), values)
    dat = msg[1]
    values["CRC_STW_ACTN_RQ"] = _crc8_j1850(dat[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", int(bus), values)

  def create_action_request(self, bus: int, msg_stw_actn_req: dict, cruise_button: int):
    return self.create_stalk_request(int(bus), msg_stw_actn_req, cruise_button=int(cruise_button))

  def create_body_controls_message(self, turn: int, hazard: int, bus: int, counter: int = 1):
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": int(hazard),
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": int(turn),
      "DAS_highLowBeamDecision": 3,
      "DAS_highLowBeamOffReason": 5,
      "DAS_turnIndicatorRequestReason": 1 if int(turn) > 0 else 0,
      "DAS_bodyControlsCounter": int(counter),
      "DAS_bodyControlsChecksum": 0,
    }
    return self.packer.make_can_msg("DAS_bodyControls", int(bus), values)



  def create_lane_message(self, lane_width_m: float, left_lane_visible: bool, right_lane_visible: bool,
                          lane_range_m: float, c0: float, c1: float, c2: float, c3: float,
                          left_fork: int, right_fork: int, bus: int, counter: int):
    values = {
      "DAS_leftLaneExists": int(bool(left_lane_visible)),
      "DAS_rightLaneExists": int(bool(right_lane_visible)),
      "DAS_virtualLaneWidth": float(lane_width_m),
      "DAS_virtualLaneViewRange": float(lane_range_m),
      "DAS_virtualLaneC0": float(c0),
      "DAS_virtualLaneC1": float(c1),
      "DAS_virtualLaneC2": float(c2),
      "DAS_virtualLaneC3": float(c3),
      "DAS_leftLineUsage": 2 if bool(left_lane_visible) else 0,
      "DAS_rightLineUsage": 2 if bool(right_lane_visible) else 0,
      "DAS_leftFork": int(left_fork),
      "DAS_rightFork": int(right_fork),
      "DAS_lanesCounter": int(counter) % 16,
    }
    return self.packer.make_can_msg("DAS_lanes", int(bus), values)

  def create_telemetry_road_info(self, left_lane_visible: bool, right_lane_visible: bool,
                                 left_lane_color: int, right_lane_color: int,
                                 alca_state: int, bus: int):
    values = {
      "DAS_telemetryMultiplexer": 0,
      "DAS_telLeftLaneType": 3 if bool(left_lane_visible) else 7,   # dashed / unknown
      "DAS_telRightLaneType": 3 if bool(right_lane_visible) else 7,  # dashed / unknown
      "DAS_telLeftMarkerQuality": 3 if bool(left_lane_visible) else 0,
      "DAS_telRightMarkerQuality": 3 if bool(right_lane_visible) else 0,
      "DAS_telLeftMarkerColor": int(left_lane_color) if bool(left_lane_visible) else 0,
      "DAS_telRightMarkerColor": int(right_lane_color) if bool(right_lane_visible) else 0,
      "DAS_telLeftLaneCrossing": 1 if int(alca_state) == 1 else 0,
      "DAS_telRightLaneCrossing": 1 if int(alca_state) == 2 else 0,
    }
    return self.packer.make_can_msg("DAS_telemetry", int(bus), values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
