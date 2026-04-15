import copy
import math
import time
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.modules.CFG_module import load_bool_param, load_float_param
from openpilot.selfdrive.car.modules.BLNK_module import BLNKController
from openpilot.selfdrive.car.modules.ALC_module import ALCController
from openpilot.selfdrive.car.modules.HSO_module import HSOController
import cereal.messaging as messaging
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.teslacan import get_steer_ctrl_type
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, TeslaFlags, LEGACY_CARS, CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type


@dataclass
class _TinklaConfig:
  autopilot_disabled: bool = False
  hands_on_level: float = 2.0
  adjust_acc_with_speed_limit: bool = True
  speed_limit_offset: float = 0.0
  speed_limit_use_relative: bool = False
  enable_alc: bool = True
  alc_delay: float = 0.75
  enable_hso: bool = True
  hso_numb_period: float = 1.5
  enable_acc: bool = False


class CarState(CarStateBase):
  DOUBLE_PULL_WINDOW_MS = 750
  def __init__(self, CP):
    super().__init__(CP)
    self.msg_stw_actn_req = None
    self.stw_actn_bus = int(CANBUS.party)
    # Follow-distance (DTR_Dist_Rq) can be intermittently missing/SNA; keep last valid.
    self.cruise_distance = int(getattr(self, "cruise_distance", 255))
    self._last_follow_distance_s = int(getattr(self, "_last_follow_distance_s", 6))
    # Remember which STW_ACTN_RQ bus actually carries valid DTR_Dist_Rq, to avoid mirrored/pinned copies.
    self._follow_stw_bus = getattr(self, "_follow_stw_bus", None)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None
    # Unity parity fields
    self._tinkla = _TinklaConfig()
    self._param_frame = 0
    self.autopilot_disabled = False
    self.cruiseEnabled = False
    self._prev_cruise_buttons = 0
    self.cruise_buttons = 0
    # Unity parity: double-pull to enable adaptive speed matching
    self.enable_adaptive_cruise = False
    self._prev_enable_adaptive_cruise = False
    self.acc_speed_max_ms = 0.0
    self._last_cruise_stalk_pull_ms = 0
    self._prev_pull_button = 0
    self._xnor_last_virtual_btn = 0
    self._xnor_last_virtual_ms = 0
    self._xnor_last_virtual_turn = 0
    self._xnor_last_virtual_turn_ms = 0

    self.turnSignalStalkState = 0
    self.speed_units = "MPH"
    self.speed_limit_ms = 0.0
    self.speed_limit_ms_das = 0.0
    self.baseMapSpeedLimitMPS = 0.0
    self.lastValidMapSpeedLimitMPS = 0.0
    # Tesla UI speed-limit offset (if present on CAN)
    self.ui_speed_limit_offset_uom = 0.0
    self.ui_speed_limit_offset_units = ""  # "MPH" or "KPH"
    self.ui_speed_limit_offset_valid = False
    self.stock_cruise_enabled = False
    self.stock_cruise_state = ""
    self.stock_cruise_set_speed_ms = 0.0
    self.leftBlinkerLamp = False
    self.rightBlinkerLamp = False
    # ALC/BLNK/HSO/ACC (Unity parity)
    self.enableALC = bool(self._tinkla.enable_alc)
    self.autoStartAlcaDelay = float(self._tinkla.alc_delay)
    self.enableHSO = bool(self._tinkla.enable_hso)
    self.hsoNumbPeriod = float(self._tinkla.hso_numb_period)
    self.enableACC = bool(self._tinkla.enable_acc)

    self.tap_direction = 0
    self._alc_tap_latch_dir = 0
    self._alc_tap_latch_until = 0
    self.alca_direction = 0  # 0-none, 1-left, 2-right
    self.alca_pre_engage = False
    self.prev_alca_pre_engage = False
    self.alca_engaged = False
    self.alca_done = False
    self.alca_need_engagement = False

    self.HSOSteeringPressed = False
    self.human_control = False

    self.blinker_controller = BLNKController()
    self.alca_controller = ALCController()
    self.hso_controller = HSOController()
    self._alc_plan_services = [s for s in ("modelV2", "lateralPlan") if s in messaging.SERVICE_LIST]
    self._alc_sm = messaging.SubMaster(self._alc_plan_services, ignore_avg_freq=True) if self._alc_plan_services else None
    self._vego_nan_logged = False
    try:
      self._reload_tinkla_params()
      self.autopilot_disabled = bool(self._tinkla.autopilot_disabled)
    except Exception:
      pass


  def _reload_tinkla_params(self) -> None:
    self._tinkla.autopilot_disabled = load_bool_param("TinklaAutopilotDisabled", False)
    self._tinkla.hands_on_level = load_float_param("TinklaHandsOnLevel", 2.0)
    self._tinkla.adjust_acc_with_speed_limit = load_bool_param("TinklaAdjustAccWithSpeedLimit", False)
    self._tinkla.speed_limit_offset = load_float_param("TinklaSpeedLimitOffset", 0.0)
    self._tinkla.speed_limit_use_relative = load_bool_param("TinklaSpeedLimitUseRelative", False)
    self._tinkla.enable_alc = load_bool_param("TinklaEnableALC", True)
    self._tinkla.alc_delay = load_float_param("TinklaAlcDelay", 0.75)
    self._tinkla.enable_hso = load_bool_param("TinklaEnableHSO", True)
    self._tinkla.hso_numb_period = load_float_param("TinklaHsoNumbPeriod", 1.5)
    self._tinkla.enable_acc = load_bool_param("TinklaEnableACC", False)
    self.enableALC = bool(self._tinkla.enable_alc)
    self.autoStartAlcaDelay = float(self._tinkla.alc_delay)
    self.enableHSO = bool(self._tinkla.enable_hso)
    self.hsoNumbPeriod = float(self._tinkla.hso_numb_period)
    self.enableACC = bool(self._tinkla.enable_acc)

  def _update_alc_state_from_plan(self, enabled: bool) -> None:
    if self._alc_sm is None:
      self.alca_controller.update(False, self, self._param_frame, None)
      return

    try:
      self._alc_sm.update(0)
      plan_msg = None
      if "modelV2" in self._alc_plan_services:
        plan_msg = self._alc_sm["modelV2"]
      if (plan_msg is None) and ("lateralPlan" in self._alc_plan_services):
        plan_msg = self._alc_sm["lateralPlan"]
      self.alca_controller.update(bool(enabled), self, self._param_frame, plan_msg)
    except Exception:
      self.alca_controller.update(False, self, self._param_frame, None)

  def _apply_alc_blinkers(self, ret) -> None:
    self.blinker_controller.update_state(self, self._param_frame)
    self.tap_direction = int(self.blinker_controller.tap_direction)

    hold_dir = 0
    one_lamp = bool(self.leftBlinkerLamp) != bool(self.rightBlinkerLamp)
    alca_pre_engage = bool(getattr(self, "alca_pre_engage", False))
    alca_engaged = bool(getattr(self, "alca_engaged", False))
    alca_done = bool(getattr(self, "alca_done", False))

    if alca_engaged:
      # Once the lane change has started, the pre-engage tap latch is no longer needed.
      self._alc_tap_latch_dir = 0
      self._alc_tap_latch_until = 0

    if self.enableALC and int(self.turnSignalStalkState) == 0:
      # Only extend the comfort tap once the planner has actually entered preLaneChange.
      # This avoids turning ordinary non-ALC indicator taps into a virtual held blinker.
      if alca_pre_engage and one_lamp and int(getattr(self, "_alc_tap_latch_until", 0)) <= int(self._param_frame):
        self._alc_tap_latch_dir = 1 if self.leftBlinkerLamp else 2
        dur_s = max(2.5, float(self.autoStartAlcaDelay) + 0.5)
        self._alc_tap_latch_until = int(self._param_frame + dur_s * 100)

      # Unity-style lifecycle: pre-engage owns the remembered tap, active change owns the blinker,
      # then finishing must be allowed to clear so DesireHelper does not re-arm another lane change.
      if alca_engaged and (not alca_done) and int(getattr(self, "alca_direction", 0) or 0) in (1, 2):
        hold_dir = int(self.alca_direction)
      elif (not alca_engaged) and int(getattr(self, "_alc_tap_latch_until", 0)) > int(self._param_frame):
        hold_dir = int(getattr(self, "_alc_tap_latch_dir", 0) or 0)

    if hold_dir == 1:
      ret.leftBlinker = True
      ret.rightBlinker = False
    elif hold_dir == 2:
      ret.leftBlinker = False
      ret.rightBlinker = True
    else:
      ret.leftBlinker = bool(self.leftBlinkerLamp) and int(self.turnSignalStalkState) == 0 and int(self.tap_direction) == 1
      ret.rightBlinker = bool(self.rightBlinkerLamp) and int(self.turnSignalStalkState) == 0 and int(self.tap_direction) == 2

  def _now_ms(self) -> int:
    return int(time.monotonic() * 1000.0)

  def _filter_virtual_turn_stalk(self, raw_ts: int) -> int:
    """Ignore our own virtual STW turn-hold frames so they do not look like a real held stalk."""
    raw_ts = int(raw_ts or 0)
    if raw_ts == 3:
      raw_ts = 0

    vturn = int(getattr(self, "_xnor_last_virtual_turn", 0) or 0)
    vms = int(getattr(self, "_xnor_last_virtual_turn_ms", 0) or 0)
    now_ms = int(self._now_ms())

    if raw_ts in (1, 2) and raw_ts == vturn and (0 <= (now_ms - vms) <= 250):
      return 0
    return raw_ts

  def _calc_speed_limit_target_ms(self, speed_units: str) -> float:
    """Compute target speed for speed-limit matching (Unity parity).

    If Tesla UI offset is present on CAN (UI_userSpeedOffset), use it as the base offset value.
    Then apply the repo's relative/absolute setting:
      - relative: treat offset as a percent
      - absolute: treat offset as an absolute MPH/KPH delta

    If Tesla UI offset is not available, fall back to Tinkla params.
    """
    limit_ms = float(getattr(self, "speed_limit_ms", 0.0) or getattr(self, "speed_limit_ms_das", 0.0) or 0.0)
    if limit_ms <= 0.0:
      return 0.0

    use_relative = bool(getattr(self._tinkla, "speed_limit_use_relative", False))

    # Base offset comes from Tesla UI if available, else from param.
    if bool(getattr(self, "ui_speed_limit_offset_valid", False)):
      off_uom = float(getattr(self, "ui_speed_limit_offset_uom", 0.0) or 0.0)
      off_units = str(getattr(self, "ui_speed_limit_offset_units", speed_units) or speed_units)
    else:
      off_uom = float(getattr(self._tinkla, "speed_limit_offset", 0.0) or 0.0)
      off_units = str(speed_units or "MPH")

    if use_relative:
      return max(0.0, limit_ms * (1.0 + off_uom / 100.0))

    if off_units == "KPH":
      return max(0.0, limit_ms + off_uom * CV.KPH_TO_MS)
    return max(0.0, limit_ms + off_uom * CV.MPH_TO_MS)
  def _update_adaptive_cruise_mode(self, *, now_ms: int, v_ego_ms: float) -> None:
    """Unity outcome: double stalk pull toggles adaptive speed matching.

    - Two MAIN pulls within 750ms enables adaptive matching.
    - One MAIN pull while enabled disables it.
    - Auto-disables when stock cruise is not ENABLED/STANDBY or speed is below minimum.
    """
    btn = int(getattr(self, "cruise_buttons", 0) or 0)
    prev_btn = int(getattr(self, "_prev_pull_button", 0) or 0)

    pull_btns = {int(CruiseButtons.MAIN), int(CruiseButtons.DECEL_SET)}
    is_pull = int(btn) in pull_btns
    is_edge = is_pull and (prev_btn != int(btn))

    if is_edge:
      # Ignore our own virtual stalk pulses (sent by CarController).
      vbtn = int(getattr(self, "_xnor_last_virtual_btn", 0) or 0)
      vms = int(getattr(self, "_xnor_last_virtual_ms", 0) or 0)
      is_virtual = (int(btn) == vbtn) and (0 <= (int(now_ms) - vms) <= 250)

      if not is_virtual:
        last_ms = int(getattr(self, "_last_cruise_stalk_pull_ms", 0) or 0)
        double_pull = (int(now_ms) - last_ms) <= int(self.DOUBLE_PULL_WINDOW_MS)
        self._last_cruise_stalk_pull_ms = int(now_ms)

        stock_state = str(getattr(self, "stock_cruise_state", "") or "")
        ready = (stock_state in ("ENABLED", "STANDBY", "OVERRIDE", "STANDSTILL")) and (float(v_ego_ms) > (17.1 * CV.MPH_TO_MS))

        if ready and (not bool(getattr(self, "enable_adaptive_cruise", False))):
          if double_pull:
            self.enable_adaptive_cruise = True
        elif ready and bool(getattr(self, "enable_adaptive_cruise", False)):
          # single pull toggles off
          self.enable_adaptive_cruise = False

    # auto-disable when cruise not ready
    stock_state = str(getattr(self, "stock_cruise_state", "") or "")
    if (stock_state not in ("ENABLED", "STANDBY", "OVERRIDE", "STANDSTILL")) or (float(v_ego_ms) <= (17.1 * CV.MPH_TO_MS)):
      self.enable_adaptive_cruise = False

    self._prev_pull_button = int(btn)

  def _pick_stock_cruise_set_u(self, di_state: dict, v_ego_ms: float, cruise_enabled: bool, speed_units: str) -> tuple[float, str]:
    """Pick Tesla cruise setpoint in MPH/KPH without changing the DBC.

    - If stock cruise is disabled (XNOR lateral-only), DI_cruiseSet may look like ~0.5*vEgo.
      Prefer DI_digitalSpeed to keep the Comma UI sane.
    - If stock cruise is enabled, choose between DI_cruiseSet, DI_digitalSpeed, and DI_cruiseSet*2
      using plausibility + stability.
    """
    try:
      a = float(di_state.get("DI_cruiseSet", 0.0) or 0.0)
    except Exception:
      a = 0.0
    try:
      b = float(di_state.get("DI_digitalSpeed", 0.0) or 0.0)
    except Exception:
      b = 0.0

    uom = "KPH" if speed_units == "KPH" else "MPH"
    ms_to_u = CV.MS_TO_KPH if uom == "KPH" else CV.MS_TO_MPH
    v_u = float(v_ego_ms) * ms_to_u
    thr = max(2.5, 0.10 * max(v_u, 1.0))

    prev_enabled = bool(getattr(self, "_stock_cruise_enabled_prev", False))
    if bool(cruise_enabled) and (not prev_enabled) and a > 0.0:
      # On enable edge, setpoint typically equals current speed; detect half-scale once.
      if (b > 0.0) and (abs((2.0 * a) - b) <= thr) and (abs(a - b) > thr):
        self._cruise_set_scale = 2.0
      elif (abs((2.0 * a) - v_u) <= thr) and (abs(a - v_u) > thr):
        self._cruise_set_scale = 2.0
      elif (b > 0.0) and (abs(a - b) <= thr):
        self._cruise_set_scale = 1.0
      elif abs(a - v_u) <= thr:
        self._cruise_set_scale = 1.0

    scale = float(getattr(self, "_cruise_set_scale", 1.0) or 1.0)
    self._stock_cruise_enabled_prev = bool(cruise_enabled)

    if bool(cruise_enabled):
      if a > 0.0:
        val = a * scale
        src = "DI_cruiseSet" if scale < 1.5 else "DI_cruiseSet_x2"
        return float(val), src
      if b > 0.0:
        return float(b), "DI_digitalSpeed"
      return 0.0, "none"

    # Stock cruise disabled: keep UI stable by using digital speed (and correct half-scale if present).
    if b > 0.0:
      if (abs((2.0 * b) - v_u) <= thr) and (abs(b - v_u) > thr):
        return float(2.0 * b), "DI_digitalSpeed_x2"
      return float(b), "DI_digitalSpeed"

    if a > 0.0:
      # Fallback (rare): if DI_cruiseSet is the only non-zero, apply learned scale.
      val = a * scale
      src = "DI_cruiseSet" if scale < 1.5 else "DI_cruiseSet_x2"
      return float(val), src

    return 0.0, "none"


  def _update_speed_limit(self, can_parsers) -> None:
    """Unity-parity speed limit parsing (map/sign + DAS fallback) into m/s."""
    speed_limit_ms = 0.0
    speed_limit_ms_das = 0.0

    # Debug/raw inputs (logged when TinklaAdjustAccWithSpeedLimit is enabled)
    gps_units = None
    gps_mpp = None
    rd_sign = None
    rd_base_mps = None
    map_type = None
    das_mph = None
    def _msg(name: str):
      for bk in (Bus.party, Bus.ap_party, Bus.cam, Bus.chassis, Bus.pt, Bus.ap_pt):
        cp = can_parsers.get(bk)
        if cp is None:
          continue
        try:
          return cp.vl.get(name)
        except KeyError:
          continue
      return None

    try:
      gps = _msg("UI_gpsVehicleSpeed")
      if gps is not None:
        # Tesla UI offset setting (absolute offset in MPH/KPH)
        try:
          off_u = float(gps.get("UI_userSpeedOffset", 0.0) or 0.0)
          off_units = int(gps.get("UI_userSpeedOffsetUnits", 0) or 0)
          self.ui_speed_limit_offset_uom = float(off_u)
          self.ui_speed_limit_offset_units = "KPH" if off_units == 1 else "MPH"
          self.ui_speed_limit_offset_valid = True
        except Exception:
          self.ui_speed_limit_offset_valid = False
          self.ui_speed_limit_offset_uom = 0.0
          self.ui_speed_limit_offset_units = ""
        msu = int(gps.get("UI_mapSpeedLimitUnits", 0))
        gps_units = msu
        map_uom_to_ms = CV.KPH_TO_MS if msu == 1 else CV.MPH_TO_MS
        map_ms_to_uom = CV.MS_TO_KPH if msu == 1 else CV.MS_TO_MPH

        map_data = _msg("UI_driverAssistMapData") or {}
        speed_limit_type = int(map_data.get("UI_mapSpeedLimitType", map_data.get("UI_mapSpeedLimitType", map_data.get("UI_mapSpeedLimit", 0))) or 0)
        map_type = speed_limit_type

        rd = _msg("UI_driverAssistRoadSign") or {}
        try:
          rd_sign = int(rd.get("UI_roadSign", 0) or 0)
        except Exception:
          rd_sign = None
        try:
          rd_base_mps = float(rd.get("UI_baseMapSpeedLimitMPS", 0.0) or 0.0)
        except Exception:
          rd_base_mps = None
        if int(rd.get("UI_roadSign", 0)) == 3:
          base_map = float(rd.get("UI_baseMapSpeedLimitMPS", 0.0) or 0.0)
          self.baseMapSpeedLimitMPS = int(base_map * map_ms_to_uom + 0.99) / map_ms_to_uom

        base_map = float(getattr(self, "baseMapSpeedLimitMPS", 0.0) or 0.0)
        gps_mpp = float(gps.get("UI_mppSpeedLimit", 0.0) or 0.0)

        live_map_limit_ms = 0.0
        if base_map > 0.0 and (speed_limit_type != 0x1F or base_map >= 5.56):
          live_map_limit_ms = float(base_map)
        elif gps_mpp > 0.0:
          live_map_limit_ms = gps_mpp * map_uom_to_ms

        if live_map_limit_ms > 0.0:
          self.lastValidMapSpeedLimitMPS = float(live_map_limit_ms)
          speed_limit_ms = float(live_map_limit_ms)
        else:
          speed_limit_ms = float(getattr(self, "lastValidMapSpeedLimitMPS", 0.0) or 0.0)
    except Exception:
      pass

    try:
      ds2 = _msg("DAS_status2") or {}
      if isinstance(ds2, dict) and "DAS_accSpeedLimit" in ds2:
        das_mph = float(ds2.get("DAS_accSpeedLimit", 0.0) or 0.0)
        speed_limit_ms_das = das_mph * CV.MPH_TO_MS
    except Exception:
      pass

    self.speed_limit_ms_das = float(speed_limit_ms_das)
    # Empirically on HW2, DAS_accSpeedLimit can stick at a low default (e.g. 15mph) while map/sign shows the real limit.
    # Use DAS as fallback only, never as a cap.
    chosen = speed_limit_ms if speed_limit_ms > 0.0 else speed_limit_ms_das
    self.speed_limit_ms = float(chosen)

    if self._tinkla.adjust_acc_with_speed_limit and (self._param_frame % 100 == 0):
      try:
        uom = str(getattr(self, "speed_units", "MPH"))
        conv = 2.2369362920544 if uom == "MPH" else 3.6
        cloudlog.info(
          f"[XNOR_CS] uom={uom} src={getattr(self, '_cruise_set_src', 'none')} "
          f"cruiseSet={float(getattr(self, 'stock_cruise_set_speed_ms', 0.0))*conv:.1f} "
          f"stockCruise={bool(getattr(self, 'stock_cruise_enabled', False))} "
          f"speedLimit={float(getattr(self, 'speed_limit_ms', 0.0))*conv:.1f} das={float(getattr(self, 'speed_limit_ms_das', 0.0))*conv:.1f} "
          f"raw(gps_u={gps_units}, gps_mpp={gps_mpp}, rd_sign={rd_sign}, rd_base_mps={rd_base_mps}, map_type={map_type}, das_mph={das_mph})"
        )
      except Exception:
        pass

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled


  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    if not math.isfinite(ret.vEgoRaw):
      if not self._vego_nan_logged:
        cloudlog.error(f"[XNOR_VEGO] non-finite vEgoRaw={ret.vEgoRaw!r} from DI_speed.DI_vehicleSpeed")
        self._vego_nan_logged = True
      ret.vEgoRaw = 0.0
      kf = getattr(self, "v_ego_kf", None)
      if kf is not None and hasattr(kf, "x"):
        try:
          kf.x = [[0.0], [0.0]]
        except Exception:
          try:
            kf.x = [0.0, 0.0]
          except Exception:
            pass
    ret.vEgo, ret.aEgo = self.update_speed_kf(float(ret.vEgoRaw))
    if (not math.isfinite(ret.vEgo)) or (not math.isfinite(ret.aEgo)):
      if not self._vego_nan_logged:
        cloudlog.error(f"[XNOR_VEGO] non-finite vEgo/aEgo from speed_kf vEgoRaw={ret.vEgoRaw!r}")
        self._vego_nan_logged = True
      ret.vEgoRaw = 0.0
      ret.vEgo = 0.0
      ret.aEgo = 0.0
      kf = getattr(self, "v_ego_kf", None)
      if kf is not None and hasattr(kf, "x"):
        try:
          kf.x = [[0.0], [0.0]]
        except Exception:
          try:
            kf.x = [0.0, 0.0]
          except Exception:
            pass

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    self.human_control = bool(self.hands_on_level >= 1)
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    if self.enableHSO:
      ret.steeringDisengage = (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
    else:
      ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    self.stock_cruise_state = str(cruise_state or "")
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)
    # Cruise set speed (DI_state): pick correct decoded field without changing the DBC
    uom = speed_units if speed_units in ("KPH", "MPH") else "MPH"
    cruise_set_u, src = self._pick_stock_cruise_set_u(cp_party.vl["DI_state"], float(ret.vEgo), bool(cruise_enabled), uom)
    self.stock_cruise_enabled = bool(cruise_enabled)
    if cruise_set_u > 0.0:
      self.stock_cruise_set_speed_ms = float(cruise_set_u) * (CV.KPH_TO_MS if uom == "KPH" else CV.MPH_TO_MS)
      ret.cruiseState.speed = max(float(self.stock_cruise_set_speed_ms), 1e-3)
    else:
      self.stock_cruise_set_speed_ms = 0.0
      ret.cruiseState.speed = max(float(ret.vEgo), 1e-3)
    self._cruise_set_src = str(src)
    if self.autopilot_disabled:
      # Unity parity: allow engagement without Tesla cruise (low-speed lateral only)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self.cruiseEnabled)
    else:
      ret.cruiseState.enabled = cruise_enabled and not self.autopark
      ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Unity parity: store last STW_ACTN_RQ for virtual stalk + tap-to-ALC
    self.speed_units = speed_units if speed_units in ("KPH", "MPH") else "MPH"

    # Prefer the STW_ACTN_RQ instance that actually carries a changing follow-distance.
    # Some harnesses/bus layouts mirror STW_ACTN_RQ onto multiple buses; a mirrored copy may pin DTR_Dist_Rq.
    stw_candidates = []
    for bk in (Bus.party, Bus.chassis, Bus.pt, Bus.ap_party, Bus.ap_pt):
      _cp = can_parsers.get(bk)
      if _cp is None:
        continue
      try:
        _stw = _cp.vl["STW_ACTN_RQ"]
      except KeyError:
        continue
      _bus = int(getattr(_cp, "bus", CANBUS.party))
      _dtr = _stw.get("DTR_Dist_Rq", None)
      try:
        _dtr_i = int(_dtr) if _dtr is not None else None
      except Exception:
        _dtr_i = None
      stw_candidates.append((_stw, _bus, _dtr_i))

    stw = None
    stw_bus = None
    if stw_candidates:
      # Prefer a previously-working bus for follow-distance, if still available.
      follow_bus = getattr(self, "_follow_stw_bus", None)
      if follow_bus is not None:
        for _stw, _bus, _dtr_i in stw_candidates:
          if int(_bus) == int(follow_bus):
            stw = _stw
            stw_bus = _bus
            break

      # Otherwise pick the STW_ACTN_RQ instance that actually carries a changing follow-distance.
      if stw is None:
        # 1) If any candidate shows a *change* vs last frame and is not SNA, pick it.
        changed = [c for c in stw_candidates if (c[2] is not None and c[2] != 255 and c[2] != getattr(self, "cruise_distance", 255))]
        if changed:
          stw, stw_bus, _ = changed[0]
        else:
          # 2) Otherwise, pick the first non-SNA dtr candidate (if any).
          non_sna = [c for c in stw_candidates if (c[2] is not None and c[2] != 255)]
          if non_sna:
            stw, stw_bus, _ = non_sna[0]
          else:
            # 3) Fallback: first available STW_ACTN_RQ.
            stw, stw_bus, _ = stw_candidates[0]

    if stw is not None:
      # Seed virtual stalk from a known-good party-bus STW frame when available.
      stw_seed = stw
      for _stw, _bus, _dtr_i in stw_candidates:
        if int(_bus) == int(CANBUS.party):
          stw_seed = _stw
          break

      self.msg_stw_actn_req = copy.copy(stw_seed)
      # Never transmit on CANBUS.radar (bus 1). Keep TX on CANBUS.party to avoid CAN errors.
      self.stw_actn_bus = int(CANBUS.party)
      self.cruise_buttons = int(stw_seed.get("SpdCtrlLvr_Stat", 0))
      # Unity parity: publish followDistanceS from stalk distance setting (DTR_Dist_Rq).
      # Keep last valid value if the stalk message is missing/SNA this frame.
      ret.followDistanceS = self._last_follow_distance_s

      dtr_raw = stw.get("DTR_Dist_Rq", 255)
      try:
        dtr = int(dtr_raw) if dtr_raw is not None else 255
      except (ValueError, TypeError):
        dtr = 255

      if dtr != 255:
        # Remember the bus that carries valid follow-distance.
        self._follow_stw_bus = int(stw_bus) if stw_bus is not None else getattr(self, "_follow_stw_bus", None)
        # pos1=0, pos2=33, pos3=66, pos4=100, pos5=133, pos6=166, pos7=200, SNA=255
        self.cruise_distance = dtr
        follow_s = int(dtr / 33)
        if 0 <= follow_s <= 6:
          self._last_follow_distance_s = follow_s
          ret.followDistanceS = follow_s


      raw_ts = int(stw_seed.get("TurnIndLvr_Stat", 0))
      self.turnSignalStalkState = self._filter_virtual_turn_stalk(raw_ts)
    else:
      ret.followDistanceS = self._last_follow_distance_s
      self.cruise_buttons = 0
      self.turnSignalStalkState = 0
      self.tap_direction = 0
      self.blinker_controller.tap_direction = 0

    if self.autopilot_disabled:
      if self.cruise_buttons == 2:  # MAIN
        self.cruiseEnabled = True
      if self.cruise_buttons == 1:  # CANCEL
        self.cruiseEnabled = False


    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    # Modern Teslas report 1=blinking (stalk released), 2=stalk held.
    self.leftBlinkerLamp = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] != 0
    self.rightBlinkerLamp = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] != 0

    self._update_alc_state_from_plan(bool(self.cruiseEnabled))
    self._apply_alc_blinkers(ret)

    # HSO (Unity parity): use handsOnLevel for steeringPressed when enabled, but never during blinkers (preserve ALC)
    self.HSOSteeringPressed = bool(getattr(self, "hands_on_level", 0.0) >= float(self._tinkla.hands_on_level))
    if self.enableHSO and not (ret.leftBlinker or ret.rightBlinker):
      ret.steeringPressed = self.HSOSteeringPressed

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # Speed limit best-effort (needed for speed-limit matching)
    self._update_speed_limit(can_parsers)


    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    lkas_ctrl_type = get_steer_ctrl_type(self.CP.flags, 2)
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == lkas_ctrl_type

    # Stock Autosteer should be off (includes FSD)
    if not (self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0
    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    # Unity parity tail
    if (self._param_frame % 100) == 0:
      try:
        self._reload_tinkla_params()
        self.autopilot_disabled = bool(self._tinkla.autopilot_disabled)
        self.enableHSO = bool(getattr(self._tinkla, 'enable_hso', True))
        self.hsoNumbPeriod = float(getattr(self._tinkla, 'hso_numb_period', 1.5) or 1.5)
        self.handsOnLimit = float(getattr(self._tinkla, 'hands_on_level', 2.0) or 2.0)
      except Exception:
        pass
    self._param_frame += 1

    if self.autopilot_disabled:
      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self.cruiseEnabled) and (not ret.doorOpen) and (ret.gearShifter == structs.CarState.GearShifter.drive) and (not ret.seatbeltUnlatched)
      self.cruiseEnabled = bool(ret.cruiseState.enabled)

    ret.buttonEvents = []
    try:
      prev = int(self._prev_cruise_buttons)
      cur = int(getattr(self, "cruise_buttons", 0))
      def _be(t, pressed):
        e = structs.CarState.ButtonEvent()
        e.type = t
        e.pressed = pressed
        return e

      # XNOR/Unity parity:
      # - Virtual decel/cancel presses (issued by our LONG/ACC stalk logic) must NOT lower vCruise,
      #   otherwise the planner ceiling collapses and we never "resume" when the lead clears.
      now_ms = int(time.monotonic_ns() // 1_000_000)
      virt_btn = int(getattr(self, "_xnor_last_virtual_btn", 0) or 0)
      virt_ms = int(getattr(self, "_xnor_last_virtual_ms", 0) or 0)
      virt_prev = (prev == virt_btn) and (abs(now_ms - virt_ms) <= 1200)

      accel_vals = (4, 16)
      decel_vals = (8, 32)

      # Allow virtual accel/resume to raise vCruise toward the speed limit ceiling.
      if (prev in accel_vals) and (cur not in accel_vals):
        ret.buttonEvents.append(_be(ButtonType.accelCruise, False))

      # Suppress ONLY virtual decel/cancel so vCruise stays at the max ceiling (Unity behavior).
      if (not virt_prev) and (prev in decel_vals) and (cur not in decel_vals):
        ret.buttonEvents.append(_be(ButtonType.decelCruise, False))

      if (not virt_prev) and (prev == 1) and (cur != 1):
        ret.buttonEvents.append(_be(ButtonType.cancel, False))

      if (prev == 2) and (cur != 2):
        ret.buttonEvents.append(_be(ButtonType.resumeCruise, False))

      self._prev_cruise_buttons = cur
    except Exception:
      pass


    try:


      self._update_adaptive_cruise_mode(now_ms=int(time.monotonic_ns()//1_000_000), v_ego_ms=float(ret.vEgo))


    except Exception:


      pass


    # Unity parity: separate max cruise (planner/UI) from actual stock set speed when adaptive is enabled.
    try:
      prev_adapt = bool(getattr(self, "_prev_enable_adaptive_cruise", False))
      now_adapt = bool(getattr(self, "enable_adaptive_cruise", False))
      uom = str(getattr(self, "speed_units", "MPH") or "MPH")
      use_sl = bool(getattr(self._tinkla, "adjust_acc_with_speed_limit", False))
      sl_target_ms = float(self._calc_speed_limit_target_ms(uom)) if use_sl else 0.0

      # Unity parity: maintain a separate max cruise (planner/UI ceiling) so following a lead doesn't
      # permanently lower vCruise.
      if now_adapt:
        actual_set_ms = float(getattr(self, "stock_cruise_set_speed_ms", 0.0) or 0.0)
        if actual_set_ms <= 0.0:
          actual_set_ms = float(ret.cruiseState.speed or 0.0)

        if not prev_adapt:
          # On engage, seed max from speed-limit (if available) else current set speed.
          seed_ms = float(sl_target_ms) if sl_target_ms > 0.0 else float(actual_set_ms)
          self.acc_speed_max_ms = float(max(float(ret.vEgoRaw), seed_ms, float(getattr(self, "acc_speed_max_ms", 0.0) or 0.0)))
        else:
          if sl_target_ms > 0.0:
            # When speed-limit matching is active, keep max aligned to the current limit.
            self.acc_speed_max_ms = float(max(float(ret.vEgoRaw), float(sl_target_ms)))
          else:
            # No valid limit: hold prior max (but never below current speed).
            self.acc_speed_max_ms = float(max(float(ret.vEgoRaw), float(getattr(self, "acc_speed_max_ms", 0.0) or 0.0)))
      else:
        actual_set_ms = float(getattr(self, "stock_cruise_set_speed_ms", 0.0) or 0.0)
        if actual_set_ms <= 0.0:
          actual_set_ms = float(ret.cruiseState.speed or 0.0)
        self.acc_speed_max_ms = 0.0

      # Publish: cruiseState.speed = max cruise; cruiseState.speedCluster = actual Tesla set speed.
      if now_adapt and (float(self.acc_speed_max_ms) > 0.1):
        ret.cruiseState.speedCluster = max(float(actual_set_ms), 1e-3)
        ret.cruiseState.speed = max(float(self.acc_speed_max_ms), 1e-3)

        # XNOR longitudinal planner uses carState.vCruise (kph). Keep it at the ceiling while adaptive is enabled.

        try:

          ret.vCruise = float(self.acc_speed_max_ms) * CV.MS_TO_KPH

        except Exception:

          pass
      else:
        ret.cruiseState.speedCluster = max(float(ret.cruiseState.speed or 0.0), 1e-3)

      ret.adaptiveCruiseEnabled = bool(now_adapt)
      self._prev_enable_adaptive_cruise = bool(now_adapt)
    except Exception:
      ret.adaptiveCruiseEnabled = bool(getattr(self, "enable_adaptive_cruise", False))


    return ret


  def update_legacy(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    if not math.isfinite(ret.vEgoRaw):
      if not self._vego_nan_logged:
        cloudlog.error(f"[XNOR_VEGO] non-finite vEgoRaw={ret.vEgoRaw!r} from ESP_B.ESP_vehicleSpeed")
        self._vego_nan_logged = True
      ret.vEgoRaw = 0.0
      kf = getattr(self, "v_ego_kf", None)
      if kf is not None and hasattr(kf, "x"):
        try:
          kf.x = [[0.0], [0.0]]
        except Exception:
          try:
            kf.x = [0.0, 0.0]
          except Exception:
            pass
    ret.vEgo, ret.aEgo = self.update_speed_kf(float(ret.vEgoRaw))
    if (not math.isfinite(ret.vEgo)) or (not math.isfinite(ret.aEgo)):
      if not self._vego_nan_logged:
        cloudlog.error(f"[XNOR_VEGO] non-finite vEgo/aEgo from speed_kf vEgoRaw={ret.vEgoRaw!r}")
        self._vego_nan_logged = True
      ret.vEgoRaw = 0.0
      ret.vEgo = 0.0
      ret.aEgo = 0.0
      kf = getattr(self, "v_ego_kf", None)
      if kf is not None and hasattr(kf, "x"):
        try:
          kf.x = [[0.0], [0.0]]
        except Exception:
          try:
            kf.x = [0.0, 0.0]
          except Exception:
            pass

    # Gas pedal
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    self.human_control = bool(self.hands_on_level >= 1)
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    if self.enableHSO:
      ret.steeringDisengage = (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
    else:
      ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    self.stock_cruise_state = str(cruise_state or "")
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    # Cruise set speed (DI_state): pick correct decoded field without changing the DBC
    ret.cruiseState.enabled = cruise_enabled
    uom = speed_units if speed_units in ("KPH", "MPH") else "MPH"
    cruise_set_u, src = self._pick_stock_cruise_set_u(cp_chassis.vl["DI_state"], float(ret.vEgo), bool(cruise_enabled), uom)
    self.stock_cruise_enabled = bool(cruise_enabled)
    if cruise_set_u > 0.0:
      self.stock_cruise_set_speed_ms = float(cruise_set_u) * (CV.KPH_TO_MS if uom == "KPH" else CV.MPH_TO_MS)
      ret.cruiseState.speed = max(float(self.stock_cruise_set_speed_ms), 1e-3)
    else:
      self.stock_cruise_set_speed_ms = 0.0
      ret.cruiseState.speed = max(float(ret.vEgo), 1e-3)
    self._cruise_set_src = str(src)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Unity parity: store last STW_ACTN_RQ for virtual stalk + tap-to-ALC
    self.speed_units = speed_units if speed_units in ("KPH", "MPH") else "MPH"


    # Speed limit best-effort (needed for speed-limit matching)
    self._update_speed_limit(can_parsers)

    stw = None
    stw_bus = None
    for bk in (Bus.party, Bus.chassis, Bus.pt, Bus.ap_party, Bus.ap_pt):
      _cp = can_parsers.get(bk)
      if _cp is None:
        continue
      try:
        stw = _cp.vl["STW_ACTN_RQ"]
        stw_bus = int(getattr(_cp, "bus", CANBUS.party))
        break
      except KeyError:
        continue
    if stw is not None:
      self.msg_stw_actn_req = copy.copy(stw)
      # Never transmit on CANBUS.radar (bus 1). Keep TX on CANBUS.party to avoid CAN errors.
      self.stw_actn_bus = int(CANBUS.party)
      self.cruise_buttons = int(stw.get("SpdCtrlLvr_Stat", 0))
      # Unity parity: publish followDistanceS from stalk distance setting (DTR_Dist_Rq).
      ret.followDistanceS = 255
      try:
        dtr = stw.get("DTR_Dist_Rq", 255)
        self.cruise_distance = int(dtr) if dtr is not None else 255
        if self.cruise_distance != 255:
          # pos1=0, pos2=33, pos3=66, pos4=100, pos5=133, pos6=166, pos7=200, SNA=255
          ret.followDistanceS = int(self.cruise_distance / 33)
      except Exception:
        self.cruise_distance = 255

      raw_ts = int(stw.get("TurnIndLvr_Stat", 0))
      self.turnSignalStalkState = self._filter_virtual_turn_stalk(raw_ts)
    else:
      self.cruise_buttons = 0
      self.turnSignalStalkState = 0
      self.tap_direction = 0
      self.blinker_controller.tap_direction = 0


    if self.autopilot_disabled:
      if self.cruise_buttons == 2:  # MAIN
        self.cruiseEnabled = True
      if self.cruise_buttons == 1:  # CANCEL
        self.cruiseEnabled = False


    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    self.leftBlinkerLamp = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    self.rightBlinkerLamp = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    self._update_alc_state_from_plan(bool(self.cruiseEnabled))
    self._apply_alc_blinkers(ret)

    # HSO (Unity parity): use handsOnLevel for steeringPressed when enabled, but never during blinkers (preserve ALC)
    self.HSOSteeringPressed = bool(getattr(self, "hands_on_level", 0.0) >= float(self._tinkla.hands_on_level))
    if self.enableHSO and not (ret.leftBlinker or ret.rightBlinker):
      ret.steeringPressed = self.HSOSteeringPressed

    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    if (self._param_frame % 100) == 0:
      try:
        self._reload_tinkla_params()
        self.autopilot_disabled = bool(self._tinkla.autopilot_disabled)
        self.enableHSO = bool(getattr(self._tinkla, 'enable_hso', True))
        self.hsoNumbPeriod = float(getattr(self._tinkla, 'hso_numb_period', 1.5) or 1.5)
        self.handsOnLimit = float(getattr(self._tinkla, 'hands_on_level', 2.0) or 2.0)
      except Exception:
        pass
    self._param_frame += 1

    if self.autopilot_disabled:
      ret.cruiseState.available = True
      ret.cruiseState.enabled = bool(self.cruiseEnabled) and (not ret.doorOpen) and (ret.gearShifter == structs.CarState.GearShifter.drive) and (not ret.seatbeltUnlatched)
      self.cruiseEnabled = bool(ret.cruiseState.enabled)

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])


    try:


      self._update_adaptive_cruise_mode(now_ms=int(time.monotonic_ns()//1_000_000), v_ego_ms=float(ret.vEgo))


    except Exception:


      pass

    ret.adaptiveCruiseEnabled = bool(getattr(self, "enable_adaptive_cruise", False))


    return ret


  @staticmethod
  def get_can_parsers(CP):
    """Return CANParsers keyed by opendbc.car.Bus.

    XNOR note (HW2 legacy Tesla):
      - canValid is computed as all(cp.can_valid for cp in can_parsers.values()) in interfaces.py.
      - Therefore every parser included here must be pointed at a bus that actually carries the
        messages we ask it to validate.
      - On your HW2 wiring, we have measured:
          STW_ACTN_RQ (0x045): rx_src 0 and 130 (~10Hz each)
          DI_state    (0x368): rx_src 0, 4, 130 (~10Hz)
          UI_gpsVehicleSpeed (0x2f8): rx_src 0 and 130 (~1Hz)
          EPAS_sysStatus (0x370): rx_src 0 and 130 (~25Hz)
          DAS_steeringControl (0x488): rx_src 2 and 128 (~50Hz)

    This function is written to avoid CAN error false positives by:
      - validating party on bus0
      - validating the mirrored party on bus130 when multiple pandas are present
      - validating powertrain on bus4
      - validating steering-control bus on bus2
    """
    # Determine whether a mirrored party bus is present (2-panda HW2 setups).
    num_pandas = int(getattr(CP, "numPandas", 1) or 1)
    has_mirrored_party = num_pandas > 1

    # Message validation lists (msg_name_or_addr, expected_hz).
    party_checks = [
      ("STW_ACTN_RQ", 10),
      ("DI_state", 10),
      ("UI_gpsVehicleSpeed", math.nan),
      ("EPAS_sysStatus", 25),
    ]

    mirrored_party_checks = [
      ("STW_ACTN_RQ", 10),
    ]

    pt_checks = [
      ("DI_state", 10),
    ]

    steer_checks = [
      ("DAS_steeringControl", 50),
      ("DAS_status2", math.nan),
    ]

    # Buses are the *rx_src* values we measured.
    party_bus = CANBUS.party  # 0
    mirrored_party_bus = 130  # measured mirror of party on HW2
    pt_bus = CANBUS.powertrain  # 4
    steer_bus = CANBUS.autopilot_party  # 2

    # Base dict: always include the buses we truly rely on.
    can_parsers = {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_checks, party_bus),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_checks, pt_bus),
      # Use Bus.ap_pt as the steering-control bus in legacy (0x488 observed on rx_src=2/128).
      Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.party], steer_checks, steer_bus),
    }

    # Add mirrored party bus parser only when a 2nd panda is present.
    if has_mirrored_party:
      can_parsers[Bus.ap_party] = CANParser(DBC[CP.carFingerprint][Bus.party], mirrored_party_checks, mirrored_party_bus)
    else:
      # Single-panda fallback: keep API compatibility, but don't require a separate bus.
      can_parsers[Bus.ap_party] = CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)

    # Keep these keys present for callers that expect them, but do not gate canValid on them.
    # Empty check lists => cp.can_valid will converge to True once updated.
    can_parsers[Bus.chassis] = CANParser(DBC[CP.carFingerprint][Bus.chassis], [], CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party)
    can_parsers[Bus.cam] = CANParser(
      DBC[CP.carFingerprint][Bus.party],
      [
        ("UI_driverAssistRoadSign", math.nan),
        ("UI_driverAssistMapData", math.nan),
      ],
      CANBUS.powertrain,
    )

    return can_parsers
