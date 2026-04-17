# /data/openpilot/opendbc/car/tesla/carcontroller.py
"""Tesla CarController (xnor C3)

Stable steering + Unity-parity virtual stalk for cruise speed-limit matching.

What this file does (only two things):
  1) publishes internal 0x659 (fake DAS) on bus 0 and bus 4 for panda safety (existing xnor behavior)
  2) when enabled + Tesla cruise is engaged, nudges Tesla cruise SET speed toward map speed limit
     by emitting STW_ACTN_RQ (cruise stalk up/down), using TeslaCAN.create_action_request() (CRC+counter).

It does *not* change steering behavior or ALC behavior.
"""

from __future__ import annotations

import os
import numpy as np
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.modules.LONG_module import LongController

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy as TeslaCANLegacy
except ImportError:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven as TeslaCANLegacy

from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR

try:
  from opendbc.car.tesla.teslacan import create_fake_das_msg as create_fake_das
except ImportError:
  from opendbc.car.tesla.teslacan import create_fake_das_message as create_fake_das


# SpdCtrlLvr_Stat (STW_ACTN_RQ)
BTN_IDLE = 0
BTN_CANCEL = 1
BTN_MAIN = 2
BTN_UP2 = 4
BTN_DOWN2 = 8
BTN_UP1 = 16
BTN_DOWN1 = 32

ROADWORKS_CAP_FILE = "/data/xnor_roadworks_speed_cap_kph.txt"
ROADWORKS_PRESET_FILE = "/data/xnor_roadworks_speed_cap_preset_kph.txt"
ROADWORKS_DEFAULT_KPH = 50.0 * CV.MPH_TO_KPH


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM=None):
    try:
      super().__init__(dbc_names, CP, VM)
    except TypeError:
      super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0

    self.params = Params()
    self._long_module = LongController()
    self._cached_autopilot_disabled = False
    self._cached_pedal_enabled = False
    self._cached_adjust_acc_with_speed_limit = False
    self._cached_speed_limit_offset_uom = 0.0
    self._cached_speed_limit_use_relative = False
    self._params_last_read_frame = -100000

    self._op659_prev_btn = 0
    self.apply_angle_last = 0.0
    self._lat_active_prev = False
    self._steer_warmup_until_frame = -1

    self._speed_sync_last_frame = -100000
    # Unity-parity pacing for automated cruise stalk presses
    self._human_cruise_action_time_ms = 0
    self._automated_cruise_action_time_ms = 0
    self._prev_cruise_buttons = BTN_IDLE


    self._stw_seed = None
    self._stw_seed_bus = int(CANBUS.party)
    self._stw_last_send_frame = -100000
    self._stw_release_frame = -1
    self._stw_release_bus = int(CANBUS.party)
    self._stw_sequence = []  # list[(frame:int, btn:int)]
    self._op_enabled_prev = False
    self._xnor_diag_last_log_ms = 0
    self._body_controls_prev_turn = 0
    self._virtual_turn_prev = 0
    self._virtual_turn_last_send_frame = -100000
    self._hud_prev_enabled = False

    self._roadworks_main_pulls_ms: list[int] = []
    self._roadworks_toggle_latch_until_ms = 0

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {
        CANBUS.party: CANPacker(dbc_names[Bus.party]),
        CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)

      # STW_ACTN_RQ needs CRC/counter; legacy helper doesn't implement it.
      self._action_can_by_bus = {int(bus): TeslaCAN(pkr) for bus, pkr in self.packers.items()}
      self._body_controls_can = self._action_can_by_bus[int(CANBUS.party)]
    else:
      self.packer = CANPacker(dbc_names[Bus.party])
      self.tesla_can = TeslaCAN(CP, self.packer)
      self._action_can_by_bus = {int(CANBUS.party): self.tesla_can}
      self._body_controls_can = self.tesla_can

  def _refresh_cached_params(self) -> None:
    if (self.frame - self._params_last_read_frame) < 50:
      return
    self._params_last_read_frame = int(self.frame)

    self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
    self._cached_pedal_enabled = bool(
      self.params.get_bool("TinklaPedalEnabled") or
      self.params.get_bool("PedalEnabled")
    )
    self._cached_adjust_acc_with_speed_limit = bool(self.params.get_bool("TinklaAdjustAccWithSpeedLimit"))
    self._cached_speed_limit_use_relative = bool(self.params.get_bool("TinklaSpeedLimitUseRelative"))
    try:
      self._cached_speed_limit_offset_uom = float(self.params.get("TinklaSpeedLimitOffset", encoding="utf-8") or "0")
    except Exception:
      self._cached_speed_limit_offset_uom = 0.0


  @staticmethod
  def _now_ms() -> int:
    return int(time.monotonic_ns() // 1_000_000)


  def _read_roadworks_file_float(self, path: str) -> float | None:
    try:
      with open(path, "r", encoding="utf-8") as f:
        raw = str(f.read()).strip()
    except OSError:
      return None

    if not raw:
      return None

    try:
      value = float(raw)
    except Exception:
      return None

    return float(value) if np.isfinite(value) and float(value) > 0.1 else None

  def _write_roadworks_file_float(self, path: str, value: float) -> None:
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      f.write(f"{float(value):.3f}")
    os.replace(tmp_path, path)

  def _clear_roadworks_file(self, path: str) -> None:
    try:
      os.remove(path)
    except OSError:
      pass

  def _roadworks_cap_current_kph(self) -> float | None:
    return self._read_roadworks_file_float(ROADWORKS_CAP_FILE)

  def _roadworks_cap_preset_kph(self) -> float:
    preset = self._read_roadworks_file_float(ROADWORKS_PRESET_FILE)
    if preset is not None:
      return float(preset)
    return float(ROADWORKS_DEFAULT_KPH)

  def _toggle_roadworks_cap(self) -> None:
    current_kph = self._roadworks_cap_current_kph()
    if current_kph is None:
      preset_kph = self._roadworks_cap_preset_kph()
      self._write_roadworks_file_float(ROADWORKS_CAP_FILE, float(preset_kph))
      cloudlog.info(f"[XNOR_RW_CAP] enabled cap_kph={preset_kph:.3f}")
    else:
      self._clear_roadworks_file(ROADWORKS_CAP_FILE)
      cloudlog.info(f"[XNOR_RW_CAP] cleared previous_cap_kph={current_kph:.3f}")

  def _maybe_handle_roadworks_triple_pull(self, CS) -> None:
    now_ms = int(self._now_ms())
    if int(now_ms) < int(self._roadworks_toggle_latch_until_ms):
      return

    btn = int(getattr(CS, "cruise_buttons", BTN_IDLE) or BTN_IDLE)
    prev_btn = int(getattr(self, "_prev_cruise_buttons", BTN_IDLE) or BTN_IDLE)
    main_edge = (btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    if not main_edge:
      return

    recent = [int(ts) for ts in self._roadworks_main_pulls_ms if (int(now_ms) - int(ts)) <= 1800]
    recent.append(int(now_ms))
    self._roadworks_main_pulls_ms = recent[-3:]

    if len(self._roadworks_main_pulls_ms) >= 3:
      span_ms = int(self._roadworks_main_pulls_ms[-1]) - int(self._roadworks_main_pulls_ms[-3])
      if span_ms <= 1800:
        self._toggle_roadworks_cap()
        self._roadworks_main_pulls_ms = []
        self._roadworks_toggle_latch_until_ms = int(now_ms) + 1800

  def _track_human_cruise_actions(self, CS) -> None:
    btn = int(getattr(CS, 'cruise_buttons', BTN_IDLE) or BTN_IDLE)
    prev_btn = int(getattr(self, '_prev_cruise_buttons', BTN_IDLE) or BTN_IDLE)
    self._maybe_handle_roadworks_triple_pull(CS)
    # Unity: throttle automation on any button other than MAIN/IDLE
    if (btn not in (BTN_MAIN, BTN_IDLE)) and (btn != prev_btn):
      self._human_cruise_action_time_ms = self._now_ms()
    self._prev_cruise_buttons = btn

  def _emit_internal_0x659(self, CS, can_sends) -> None:
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    cancel_edge = (stalk_btn == BTN_CANCEL) and (prev_btn != BTN_CANCEL)

    self._op659_prev_btn = stalk_btn

    if (self.frame % 10 == 0) or main_edge or cancel_edge:
      buses = {int(CANBUS.party)}
      if self.CP.carFingerprint in LEGACY_CARS:
        buses.add(int(CANBUS.powertrain))
      for bus in sorted(buses):
        can_sends.append(create_fake_das(
          self._cached_pedal_enabled,
          self._cached_autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
        ))

  def _speed_limit_target_ms(self, CS) -> float:
    # Prefer CarState's helper (uses DAS fused if present + supports relative offset)
    try:
      return float(CS._calc_speed_limit_target_ms(str(getattr(CS, "speed_units", "MPH"))))
    except Exception:
      pass

    limit_ms = float(getattr(CS, "speed_limit_ms_das", 0.0) or getattr(CS, "speed_limit_ms", 0.0) or 0.0)
    if limit_ms <= 0.0:
      return 0.0

    off = float(self._cached_speed_limit_offset_uom)
    if self._cached_speed_limit_use_relative:
      return max(0.0, limit_ms * (1.0 + off / 100.0))

    uom = str(getattr(CS, "speed_units", "MPH"))
    return max(0.0, limit_ms + (off * (CV.KPH_TO_MS if uom == "KPH" else CV.MPH_TO_MS)))

  def _stw_bus(self, CS) -> int:
    try:
      b = int(getattr(CS, "stw_actn_bus", CANBUS.party))
      # Never transmit on CANBUS.radar (bus 1); some harnesses have no ACK on that bus.
      return int(CANBUS.party) if b == int(CANBUS.radar) else int(b)
    except Exception:
      return int(CANBUS.party)

  def _action_can_for_bus(self, bus: int):
    return (
      self._action_can_by_bus.get(int(bus)) or
      self._action_can_by_bus.get(int(CANBUS.party)) or
      next(iter(self._action_can_by_bus.values()))
    )
  def _send_stw(self, CS, can_sends, btn: int, *, bus: int | None = None, turn_signal_stalk_state: int | None = None) -> bool:
    msg = getattr(CS, "msg_stw_actn_req", None)
    if msg is None:
      return False

    b = int(bus if bus is not None else self._stw_bus(CS))
    seed = dict(msg)  # Unity parity: seed from latest observed frame every send

    can_sends.append(
      self._action_can_for_bus(b).create_stalk_request(
        int(b),
        seed,
        cruise_button=int(btn),
        turn_signal_stalk_state=(None if turn_signal_stalk_state is None else int(turn_signal_stalk_state)),
      )
    )
    # Mark last virtual stalk press so CarState can ignore it for adaptive double-pull detection.
    try:
      now_ms = int(self._now_ms())
      if int(btn) != int(BTN_IDLE):
        CS._xnor_last_virtual_btn = int(btn)
        CS._xnor_last_virtual_ms = now_ms
      if turn_signal_stalk_state is not None:
        CS._xnor_last_virtual_turn = int(turn_signal_stalk_state)
        CS._xnor_last_virtual_turn_ms = now_ms
    except Exception:
      pass
    self._stw_seed_bus = int(b)
    self._stw_last_send_frame = int(self.frame)
    return True

  def _queue_stalk_pulse(self, CS, can_sends, btn: int) -> bool:
    # Unity-like pulse: press now, release next frame.
    if int(self._stw_release_frame) > int(self.frame):
      return False

    if not self._send_stw(CS, can_sends, btn):
      return False

    self._stw_release_frame = int(self.frame) + 1
    self._stw_release_bus = int(self._stw_seed_bus)
    return True

  def _process_stalk_actions(self, CS, can_sends) -> None:
    hold_turn = 0
    if self.CP.carFingerprint in LEGACY_CARS:
      # patch141 fixed non-ALC taps by tightening the owned blinker lifecycle in CarState.
      # For the physical HW2 hold, use the planner-owned ALC direction directly rather than
      # the derived engaged/done booleans, which can clear too early on this base.
      if int(getattr(CS, "turnSignalStalkState", 0) or 0) == 0:
        turn = int(getattr(CS, "alca_direction", 0) or 0)
        if turn in (1, 2):
          hold_turn = turn

    # Release pending cruise pulse, preserving any active virtual turn hold.
    if int(self._stw_release_frame) == int(self.frame):
      self._send_stw(
        CS,
        can_sends,
        BTN_IDLE,
        bus=int(self._stw_release_bus),
        turn_signal_stalk_state=(hold_turn if hold_turn in (1, 2) else None),
      )
      self._stw_release_frame = -1

    # Legacy HW2 physical blinker hold follows STW_ACTN_RQ TurnIndLvr_Stat at a stock-like comfort cadence.
    # Mirror the existing internal Unity-style blinker ownership already exposed via CS.out,
    # and send one explicit release when that ownership ends.
    if self.CP.carFingerprint in LEGACY_CARS:
      prev_hold_turn = int(getattr(self, "_virtual_turn_prev", 0) or 0)
      last_send_frame = int(getattr(self, "_virtual_turn_last_send_frame", -100000) or -100000)

      send_turn = None
      if hold_turn in (1, 2):
        # Align the virtual held-stalk cadence to the moment ownership starts, so the
        # physical lamp continues with a stock-like comfort-blink rhythm from the
        # initial tap instead of being refreshed at a rapid hold cadence.
        if hold_turn != prev_hold_turn or (int(self.frame) - last_send_frame) >= 50:
          send_turn = int(hold_turn)
      elif prev_hold_turn in (1, 2):
        # Release immediately when the owned lane change ends.
        send_turn = 0

      if send_turn is not None:
        self._send_stw(
          CS,
          can_sends,
          BTN_IDLE,
          bus=int(self._stw_bus(CS)),
          turn_signal_stalk_state=int(send_turn),
        )
        self._virtual_turn_last_send_frame = int(self.frame)

      self._virtual_turn_prev = int(hold_turn)



  def _hud_alca_state(self, CS) -> int:
    turn = int(getattr(CS, "alca_direction", 0) or 0)
    if bool(getattr(CS, "alca_pre_engage", False) or getattr(CS, "alca_engaged", False)) and turn in (1, 2):
      return 8 + turn
    return 1

  def _hud_speed_limit_uom(self, CS) -> float:
    limit_ms = float(getattr(CS, "speed_limit_ms", 0.0) or 0.0)
    units = str(getattr(CS, "speed_units", "MPH"))
    return max(0.0, limit_ms * (CV.MS_TO_KPH if units == "KPH" else CV.MS_TO_MPH))

  def _process_hud_status(self, CC, CS, can_sends, human_control: bool) -> None:
    # Do not emit partial Tesla HUD status from userspace.
    # This tree already forwards stock 0x399/0x389 from AP-side, and dual ownership causes
    # IC oscillation (blue/white D), flashing speed-limit icons, and startup availability alerts.
    self._hud_prev_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    return

  def _lane_positioned_target_angle(self, desired_angle_deg: float, current_angle_deg: float, v_ego: float) -> float:
    desired_angle_deg = float(desired_angle_deg)
    current_angle_deg = float(current_angle_deg)
    v_ego = float(v_ego)

    desired_mag = abs(desired_angle_deg)
    if (desired_mag < 1.5) or (v_ego < 4.0):
      return desired_angle_deg

    assist_gain = float(np.interp(
      desired_mag,
      CarControllerParams.CURVE_ASSIST_ANGLE_BP,
      CarControllerParams.CURVE_ASSIST_GAIN_V,
    ))
    assist_extra = float(np.interp(
      desired_mag,
      CarControllerParams.CURVE_ASSIST_ANGLE_BP,
      CarControllerParams.CURVE_ASSIST_EXTRA_DEG_V,
    ))
    assist_speed_gain = float(np.interp(
      v_ego,
      CarControllerParams.CURVE_ASSIST_SPEED_BP,
      CarControllerParams.CURVE_ASSIST_SPEED_GAIN_V,
    ))
    max_delta = float(np.interp(
      v_ego,
      CarControllerParams.CURVE_ASSIST_MAX_DELTA_BP,
      CarControllerParams.CURVE_ASSIST_MAX_DELTA_V,
    ))

    assisted_angle = (desired_angle_deg * assist_gain) + (np.sign(desired_angle_deg) * assist_extra * assist_speed_gain)

    # Keep the lane-positioning assist as a modest bias around the planner request.
    return float(np.clip(
      assisted_angle,
      desired_angle_deg - max_delta,
      desired_angle_deg + max_delta,
    ))

  def _body_controls_turn(self, CS) -> int:
    if not bool(getattr(CS, "enableALC", False)):
      return 0

    cs_out = getattr(CS, "out", None)
    if cs_out is not None:
      left = bool(getattr(cs_out, "leftBlinker", False))
      right = bool(getattr(cs_out, "rightBlinker", False))
      if left != right:
        return 1 if left else 2

    turn = int(getattr(CS, "alca_direction", 0) or 0)
    return turn if turn in (1, 2) else 0

  def _process_body_controls(self, CS, can_sends) -> None:
    if self.CP.carFingerprint in LEGACY_CARS:
      self._body_controls_prev_turn = 0
      return

    turn = int(self._body_controls_turn(CS))
    prev_turn = int(getattr(self, "_body_controls_prev_turn", 0) or 0)

    # Unity HUD parity: body-controls indicator requests are refreshed continuously at 20 Hz
    # while a lane-change direction is owned, then sent once more with turn=0 to clear.
    if (self.frame % 5 == 0) and (turn in (1, 2) or prev_turn in (1, 2)):
      can_sends.append(
        self._body_controls_can.create_body_controls_message(
          turn,
          1 if bool(getattr(CS, "needs_hazard", False)) else 0,
          int(CANBUS.party),
          1,
        )
      )

    self._body_controls_prev_turn = turn

  def _diag_log(self, msg: str) -> None:
    now_ms = int(self._now_ms())
    if (now_ms - int(self._xnor_diag_last_log_ms)) < 1000:
      return
    self._xnor_diag_last_log_ms = int(now_ms)
    cloudlog.info(msg)

  def _speed_limit_sync(self, CC, CS, can_sends) -> None:
    enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    if (not enabled) or (not self._cached_autopilot_disabled):
      self._diag_log(
        f"[XNOR_CC_DIAG] gate=pre enabled={int(enabled)} "
        f"latActive={int(bool(getattr(CC, 'latActive', False)))} "
        f"cc_enabled={int(bool(getattr(CC, 'enabled', False)))} "
        f"ap_disabled={int(bool(self._cached_autopilot_disabled))}"
      )
      return

    # Don't overlap with explicit sequences, a pending pulse release, or legacy virtual turn hold.
    if self.CP.carFingerprint in LEGACY_CARS:
      hold_turn = int(getattr(self, "_virtual_turn_prev", 0) or 0)
      if hold_turn in (1, 2):
        self._diag_log(f"[XNOR_CC_DIAG] gate=turn_hold turn={hold_turn}")
        return

    if (int(self._stw_release_frame) >= 0):
      self._diag_log(f"[XNOR_CC_DIAG] gate=pending_release release_frame={int(self._stw_release_frame)} frame={int(self.frame)}")
      return

    decision = self._long_module.update(CS, enabled=enabled, frame=int(self.frame), now_ms=int(self._now_ms()))
    if decision.button is None:
      self._diag_log(f"[XNOR_CC_DIAG] gate=no_decision detail={decision.log or 'none'}")
      return

    # One Unity-style pulse; release is handled next frame by _queue_stalk_pulse().
    if self._queue_stalk_pulse(CS, can_sends, int(decision.button)):
      self._automated_cruise_action_time_ms = int(self._now_ms())

  def update(self, CC, CS, now_nanos):


    actuators = CC.actuators
    can_sends = []

    self._track_human_cruise_actions(CS)


    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._cached_autopilot_disabled)

    # Always define before use
    cs_out = getattr(CS, "out", None)
    out_steer_pressed = bool(getattr(cs_out, "steeringPressed", False)) if cs_out is not None else False
    human_control = bool(getattr(CS, "human_control", False) or out_steer_pressed)
    steer_inhibit = bool(
      (bool(getattr(cs_out, "steerFaultTemporary", False)) if cs_out is not None else False) or
      (bool(getattr(cs_out, "steerFaultPermanent", False)) if cs_out is not None else False) or
      (bool(getattr(cs_out, "steeringDisengage", False)) if cs_out is not None else False)
    )

    op_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    if op_enabled and (not bool(self._op_enabled_prev)):
      # Avoid a first-command step when engaging with wheel turned (EPS inhibit prevention).
      try:
        self.apply_angle_last = float(getattr(cs_out, "steeringAngleDeg", 0.0) if cs_out is not None else 0.0)
      except Exception:
        pass
    self._op_enabled_prev = bool(op_enabled)

    self._process_stalk_actions(CS, can_sends)
    self._process_body_controls(CS, can_sends)
    self._process_hud_status(CC, CS, can_sends, human_control)

    self._speed_limit_sync(CC, CS, can_sends)

    lat_active = (
      bool(CC.latActive) and
      autopilot_disabled and
      (not CS.out.cruiseState.standstill) and
      (not human_control) and
      (not steer_inhibit)
    )

    # Steering warm-up: for a short window after lateral becomes active, command current wheel angle.
    # This prevents an initial command step (EPS inhibit) when engaging with the wheel turned.
    if lat_active and (not bool(self._lat_active_prev)):
      self._steer_warmup_until_frame = int(self.frame) + 2  # shorter warmup so turn-in starts sooner
    self._lat_active_prev = bool(lat_active)

    # Steering (50Hz)
    if self.frame % 2 == 0:
      if (not lat_active) or human_control or steer_inhibit or (int(self.frame) < int(self._steer_warmup_until_frame)):
        apply_angle = float(CS.out.steeringAngleDeg)
      else:
        desired_angle = self._lane_positioned_target_angle(
          float(actuators.steeringAngleDeg),
          float(CS.out.steeringAngleDeg),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
        )
        apply_angle = float(apply_std_steer_angle_limits(
          float(desired_angle),
          float(self.apply_angle_last),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          float(CS.out.steeringAngleDeg),
          lat_active,
          CarControllerParams.ANGLE_LIMITS,
        ))
        steer_guard_deg = float(np.interp(
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          [0.0, 10.0, 20.0, 30.0],
          [34.0, 42.0, 52.0, 62.0],
        ))
        # Keep a measured-angle guard, but widen it with speed so the car can
        # build angle earlier into sharper corners instead of washing wide.
        apply_angle = float(np.clip(
          apply_angle,
          float(CS.out.steeringAngleDeg) - steer_guard_deg,
          float(CS.out.steeringAngleDeg) + steer_guard_deg,
        ))

      self.apply_angle_last = float(apply_angle)

      if self.CP.carFingerprint in LEGACY_CARS:
        counter = (self.frame // 2) % 16
        can_sends.append(
          self.tesla_can.create_steering_control(counter, self.apply_angle_last, lat_active)
        )
      else:
        can_sends.append(
          self.tesla_can.create_steering_control(self.apply_angle_last, lat_active)
        )

    # EPS allow (legacy)
    if (self.CP.carFingerprint in LEGACY_CARS) and (self.frame % 10 == 0):
      counter = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(counter))

    # Longitudinal (optional)
    if self.CP.openpilotLongitudinalControl and (self.frame % 4 == 0):
      state = 13 if CC.cruiseControl.cancel else 4
      accel = float(np.clip(
        float(actuators.accel),
        CarControllerParams.ACCEL_MIN,
        CarControllerParams.ACCEL_MAX
      ))
      counter = (self.frame // 4) % 8
      long_active = bool(CC.longActive) and (not autopilot_disabled)

      can_sends.append(
        self.tesla_can.create_longitudinal_command(
          state,
          accel,
          counter,
          float(CS.out.vEgo),
          long_active
        )
      )

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
