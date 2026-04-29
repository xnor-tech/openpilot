# /data/openpilot/selfdrive/car/modules/ACC_module.py
"""
Human-tuned Tesla cruise stalk button selection for XNOR.

This keeps the Unity-style owner split:
- LONG provides the desired speed target.
- ACC owns the adaptive cruise ceiling internally.
- Speed-limit changes may raise that ceiling, but automated slowdowns do not lower it.
- A manual stalk-down latches a temporary lower hold until the driver manually raises again.

The acceleration side is tuned to feel more natural:
- clear-road jumps may still begin with 5-step RES pulses
- active lead-following sticks to 1-step RES pulses for smoother spacing
- cadence stays calmer while a lead is still present to reduce hunting
- v60 adds a bounded SET fallback when RES autoengage is ignored from STANDBY
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _cc_units_kph(speed_units: str) -> tuple[float, float]:
  if speed_units == "MPH":
    return 1.0 * CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH
  return 1.0, 5.0


@dataclass
class LeadInfo:
  status: bool = False
  d_rel: float = 0.0
  v_rel: float = 0.0


@dataclass
class AccDecision:
  button: Optional[int]
  reason: str
  target_kph: float = 0.0
  current_kph: float = 0.0
  est_kph: float = 0.0


class ACCController:
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  _HUMAN_COOLDOWN_MS = 3000
  _AUTO_COOLDOWN_MS = 300
  _AUTO_COOLDOWN_ACCEL_BASE_MS = 440
  _AUTO_COOLDOWN_ACCEL_FAST_MS = 340
  _AUTO_COOLDOWN_ACCEL_SLOW_MS = 520
  _AUTO_COOLDOWN_ACCEL_FULL_MS = 440
  _ACCEL_AFTER_LEAD_CLEAR_SETTLE_MS = 180
  _FAST_DECEL_RESUME_HOLDOFF_MS = 2000
  _LEAD_FRESH_MS = 450
  _AUTOENGAGE_SPEED_WINDOW_MS = 0.8
  _AUTOENGAGE_SET_FALLBACK_MS = 1400
  _AUTOENGAGE_SET_FALLBACK_REPEAT_MS = 2200
  _AUTO_ECHO_IGNORE_MS = 550
  _MIN_CRUISE_HOLD_MARGIN_MS = 5.0 * CV.MPH_TO_MS
  _MIN_CRUISE_CANCEL_VEGO_MARGIN_MS = 1.0 * CV.MPH_TO_MS
  _MANUAL_LOWER_STALE_CLEAR_MS = 6000
  _MANUAL_CONFIRM_WINDOW_MS = 1500
  _MANUAL_PENDING_TIMEOUT_MS = 1800
  _MANUAL_LATCH_SUPPRESS_AFTER_AUTO_DECEL_MS = 2600
  _LEAD_FOLLOW_SOFT_DEADBAND_MPH = 1.0
  _LEAD_FOLLOW_PERSIST_BAND_MPH = 2.0
  _LEAD_FOLLOW_PERSIST_MS = 900


  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)
    self._last_human_button = int(CruiseButtons.IDLE)
    self._last_human_button_time_ms = 0

    self._last_auto_button = int(CruiseButtons.IDLE)
    self._last_auto_button_time_ms = 0
    self._accel_burst_steps_remaining = 0

    self.fast_decel_time_ms = 0
    self.lead_last_seen_time_ms = 0
    self._last_lead_status = False
    self._lead_cleared_time_ms = 0
    self.acc_speed_kph = 0.0
    self.speed_limit_kph = 0.0
    self.prev_speed_limit_kph = 0.0
    self._prev_enabled = False
    self._manual_lower_hold_active = False
    self._manual_hold_restore_ceiling_kph = 0.0
    self._manual_hold_restore_requested = False
    self._manual_lower_pending = False
    self._manual_lower_pending_time_ms = 0
    self._manual_raise_pending = False
    self._manual_raise_pending_time_ms = 0
    self._clear_road_ceiling_kph = 0.0
    self._last_current_set_speed_kph = 0.0
    self._lead_follow_offset_sign = 0
    self._lead_follow_offset_since_ms = 0
    self._standby_autoengage_started_ms = 0
    self._standby_autoengage_last_set_ms = 0

    self._radar_sm = messaging.SubMaster(["radarState"])

  @staticmethod
  def _button_direction(button: int) -> int:
    if CruiseButtons.is_accel(button):
      return 1
    if CruiseButtons.is_decel(button) or int(button) == int(CruiseButtons.CANCEL):
      return -1
    return 0

  def _update_max_acc_speed_from_button(self, *, button: int, speed_units: str) -> None:
    half_press_kph, full_press_kph = _cc_units_kph(speed_units)
    speed_change_map = {
      int(CruiseButtons.RES_ACCEL): float(half_press_kph),
      int(CruiseButtons.RES_ACCEL_2ND): float(full_press_kph),
      int(CruiseButtons.DECEL_SET): -1.0 * float(half_press_kph),
      int(CruiseButtons.DECEL_2ND): -1.0 * float(full_press_kph),
    }
    self.acc_speed_kph += float(speed_change_map.get(int(button), 0.0))
    self.acc_speed_kph = min(float(self.acc_speed_kph), 170.0)
    self.acc_speed_kph = max(float(self.acc_speed_kph), 0.0)


  def _recent_auto_button_direction(self, *, now_ms: int) -> int:
    if (int(now_ms) - int(self._last_auto_button_time_ms)) > int(self._AUTO_ECHO_IGNORE_MS):
      return 0
    return self._button_direction(int(self._last_auto_button))

  def _observe_manual_set_speed_change(
    self,
    *,
    now_ms: int,
    current_kph: float,
    desired_kph: float,
    speed_limit_kph: float,
    speed_units: str,
    brake_pressed: bool = False,
    stock_cruise_state: str = "",
  ) -> None:
    half_kph, _ = _cc_units_kph(speed_units)
    threshold_kph = max(0.55 * float(half_kph), 0.5)
    previous_kph = float(self._last_current_set_speed_kph)

    if float(current_kph) <= 0.0:
      self._last_current_set_speed_kph = float(current_kph)
      self._clear_manual_pending()
      return

    if float(previous_kph) <= 0.0:
      self._last_current_set_speed_kph = float(current_kph)
      return

    if bool(brake_pressed) or str(stock_cruise_state or "").upper() == "STANDBY":
      self._last_current_set_speed_kph = float(current_kph)
      self._clear_manual_pending()
      return

    delta_kph = float(current_kph) - float(previous_kph)
    recent_auto_dir = self._recent_auto_button_direction(now_ms=now_ms)
    recent_auto_age_ms = int(now_ms) - int(self._last_auto_button_time_ms)
    recent_auto_decel = (
      CruiseButtons.is_decel(int(self._last_auto_button))
      and recent_auto_age_ms <= int(self._MANUAL_LATCH_SUPPRESS_AFTER_AUTO_DECEL_MS)
    )
    recent_auto_accel = (
      CruiseButtons.is_accel(int(self._last_auto_button))
      and recent_auto_age_ms <= int(self._MANUAL_LATCH_SUPPRESS_AFTER_AUTO_DECEL_MS)
    )

    if not self._pending_manual_lower(now_ms=now_ms):
      self._manual_lower_pending = False
      self._manual_lower_pending_time_ms = 0
    if not self._pending_manual_raise(now_ms=now_ms):
      self._manual_raise_pending = False
      self._manual_raise_pending_time_ms = 0

    system_target_kph = max(float(desired_kph), float(speed_limit_kph))
    driver_is_lowering_below_system_target = float(current_kph) < (
      float(system_target_kph) - max(0.35 * float(half_kph), 0.4)
    )

    if (
      delta_kph <= (-1.0 * float(threshold_kph))
      and int(recent_auto_dir) >= 0
      and (not recent_auto_decel)
      and self._pending_manual_lower(now_ms=now_ms)
      and driver_is_lowering_below_system_target
    ):
      if not self._manual_lower_hold_active:
        self._manual_hold_restore_ceiling_kph = max(
          float(self._manual_hold_restore_ceiling_kph),
          float(self.acc_speed_kph),
          float(self.speed_limit_kph),
          float(previous_kph),
          float(desired_kph),
          float(speed_limit_kph),
        )
      self._manual_lower_hold_active = True
      self._manual_hold_restore_requested = False
      self.acc_speed_kph = max(float(current_kph), 0.0)
      self._manual_lower_pending = False
      self._manual_lower_pending_time_ms = 0
      self._manual_raise_pending = False
      self._manual_raise_pending_time_ms = 0

    elif (
      delta_kph >= float(threshold_kph)
      and int(recent_auto_dir) <= 0
      and (not recent_auto_accel)
      and self._pending_manual_raise(now_ms=now_ms)
      and (self._manual_lower_hold_active or float(self._manual_hold_restore_ceiling_kph) > 0.0)
      and float(current_kph) >= (float(previous_kph) + (0.40 * float(half_kph)))
    ):
      self._manual_hold_restore_requested = True
      self._manual_lower_hold_active = False
      self._manual_raise_pending = False
      self._manual_raise_pending_time_ms = 0
      self._manual_lower_pending = False
      self._manual_lower_pending_time_ms = 0

    self._last_current_set_speed_kph = float(current_kph)


  def note_human_buttons(self, cruise_buttons: int, *, speed_units: str, now_ms: Optional[int] = None) -> None:
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)

    if btn == int(self.prev_cruise_buttons):
      return

    # Ignore recent echoes of our own virtual pulse so ACC does not throttle itself
    # as "human input" on the readback edge.
    recent_auto_echo = (
      btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN))
      and btn == int(self._last_auto_button)
      and (now - int(self._last_auto_button_time_ms)) <= int(self._AUTO_ECHO_IGNORE_MS)
    )

    if btn not in (int(CruiseButtons.IDLE), int(CruiseButtons.MAIN)) and not recent_auto_echo:
      self.human_action_time_ms = now
      self._last_human_button = int(btn)
      self._last_human_button_time_ms = int(now)
      if CruiseButtons.is_decel(btn):
        if not self._manual_lower_hold_active:
          self._manual_hold_restore_ceiling_kph = max(
            float(self._manual_hold_restore_ceiling_kph),
            float(self.acc_speed_kph),
            float(self.speed_limit_kph),
          )
        self._manual_hold_restore_requested = False
        self._manual_lower_pending = True
        self._manual_lower_pending_time_ms = int(now)
        self._manual_raise_pending = False
        self._manual_raise_pending_time_ms = 0
      elif CruiseButtons.is_accel(btn):
        self._manual_raise_pending = True
        self._manual_raise_pending_time_ms = int(now)
        self._manual_lower_pending = False
        self._manual_lower_pending_time_ms = 0
        if self._manual_lower_hold_active or float(self._manual_hold_restore_ceiling_kph) > 0.0:
          self._manual_hold_restore_requested = True
        self._manual_lower_hold_active = False
        self._update_max_acc_speed_from_button(button=btn, speed_units=speed_units)

    self.prev_cruise_buttons = btn


  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) >= int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return int(now_ms) >= int(self.automated_action_time_ms) + int(milliseconds)

  def _poll_lead(self, *, now_ms: int) -> LeadInfo:
    try:
      self._radar_sm.update(0)
    except Exception:
      return LeadInfo()

    try:
      if not bool(self._radar_sm.valid.get("radarState", False)):
        return LeadInfo()

      rs = self._radar_sm["radarState"]
      lead = getattr(rs, "leadOne", None)
      if lead is None or not bool(getattr(lead, "status", False)):
        return LeadInfo()

      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)
      if d_rel <= 0.0:
        return LeadInfo()

      self.lead_last_seen_time_ms = int(now_ms)
      return LeadInfo(True, d_rel, v_rel)
    except Exception:
      return LeadInfo()

  @staticmethod
  def _seconds_to_collision(*, lead: LeadInfo) -> float:
    if not lead.status or lead.d_rel <= 0.0 or lead.v_rel >= 0.0:
      return 1e6
    return float(lead.d_rel) / max(1e-3, -float(lead.v_rel))


  def _note_lead_transition(self, *, lead: LeadInfo, now_ms: int) -> None:
    lead_present = bool(lead.status)

    if bool(self._last_lead_status) and not lead_present:
      self._lead_cleared_time_ms = int(now_ms)

    self._last_lead_status = lead_present

  def _lead_blocks_fast_accel(self, *, lead: LeadInfo) -> bool:
    if not lead.status or lead.d_rel <= 0.0:
      return False

    close_lead = float(lead.d_rel) < 30.0
    medium_lead = float(lead.d_rel) < 55.0
    not_opening = float(lead.v_rel) < 0.8
    return bool(close_lead or (medium_lead and not_opening))

  def _accel_cooldown_ms(
    self,
    *,
    now_ms: int,
    speed_offset_kph: float,
    available_speed_kph: float,
    lead: LeadInfo,
  ) -> int:
    if bool(lead.status):
      return int(self._AUTO_COOLDOWN_ACCEL_BASE_MS)

    if self._lead_blocks_fast_accel(lead=lead):
      return int(self._AUTO_COOLDOWN_ACCEL_SLOW_MS)

    if (int(now_ms) - int(self._lead_cleared_time_ms)) <= int(self._ACCEL_AFTER_LEAD_CLEAR_SETTLE_MS):
      return int(self._AUTO_COOLDOWN_ACCEL_SLOW_MS)

    if float(speed_offset_kph) >= 10.0 and float(available_speed_kph) >= 7.0:
      return int(self._AUTO_COOLDOWN_ACCEL_FAST_MS)

    if float(speed_offset_kph) >= 6.0 and float(available_speed_kph) >= 4.0:
      return int(self._AUTO_COOLDOWN_ACCEL_BASE_MS)

    return int(self._AUTO_COOLDOWN_ACCEL_SLOW_MS)


  def _reset_accel_burst(self) -> None:
    self._accel_burst_steps_remaining = 0

  def _prime_accel_burst(self, *, speed_offset_kph: float, available_speed_kph: float, full_kph: float) -> None:
    if float(available_speed_kph) < max(0.60 * float(full_kph), 2.0):
      self._accel_burst_steps_remaining = 0
      return

    if float(speed_offset_kph) >= (1.90 * float(full_kph)):
      self._accel_burst_steps_remaining = 2
    elif float(speed_offset_kph) >= (0.90 * float(full_kph)):
      self._accel_burst_steps_remaining = 1
    else:
      self._accel_burst_steps_remaining = 0

  def _choose_accel_button(
    self,
    *,
    now_ms: int,
    speed_units: str,
    speed_offset_kph: float,
    available_speed_kph: float,
    lead: LeadInfo,
  ) -> Optional[int]:
    half_kph, full_kph = _cc_units_kph(speed_units)
    single_threshold_kph = max(0.60 * float(half_kph), 0.5)
    single_available_threshold_kph = max(0.35 * float(half_kph), 0.3)
    full_available_threshold_kph = max(0.60 * float(full_kph), 2.0)
    recent_lead_clear = (int(now_ms) - int(self._lead_cleared_time_ms)) <= int(self._ACCEL_AFTER_LEAD_CLEAR_SETTLE_MS)
    active_lead_follow = bool(lead.status)
    burst_blocked = active_lead_follow or self._lead_blocks_fast_accel(lead=lead)

    if burst_blocked or recent_lead_clear:
      self._reset_accel_burst()
    elif int(self._accel_burst_steps_remaining) <= 0:
      self._prime_accel_burst(
        speed_offset_kph=float(speed_offset_kph),
        available_speed_kph=float(available_speed_kph),
        full_kph=float(full_kph),
      )

    if int(self._accel_burst_steps_remaining) > 0:
      full_ready = self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_ACCEL_FULL_MS)
      full_gap_threshold_kph = max(0.80 * float(full_kph), single_threshold_kph)
      if (
        full_ready
        and float(speed_offset_kph) >= float(full_gap_threshold_kph)
        and float(available_speed_kph) >= float(full_available_threshold_kph)
      ):
        self._accel_burst_steps_remaining = max(0, int(self._accel_burst_steps_remaining) - 1)
        return int(CruiseButtons.RES_ACCEL_2ND)

      if float(speed_offset_kph) < float(full_gap_threshold_kph):
        self._reset_accel_burst()

    accel_cooldown_ms = self._accel_cooldown_ms(
      now_ms=now_ms,
      speed_offset_kph=float(speed_offset_kph),
      available_speed_kph=float(available_speed_kph),
      lead=lead,
    )
    accel_ready = self._no_automated_action_for(now_ms=now_ms, milliseconds=accel_cooldown_ms)
    lead_single_threshold_kph = max(float(single_threshold_kph), 0.7)
    if (
      accel_ready
      and float(speed_offset_kph) >= (float(lead_single_threshold_kph) if active_lead_follow else float(single_threshold_kph))
      and float(available_speed_kph) >= float(single_available_threshold_kph)
    ):
      return int(CruiseButtons.RES_ACCEL)

    return None


  def _reset_lead_follow_deadband(self) -> None:
    self._lead_follow_offset_sign = 0
    self._lead_follow_offset_since_ms = 0
    self._standby_autoengage_started_ms = 0
    self._standby_autoengage_last_set_ms = 0

  def _lead_follow_effective_speed_offset_kph(
    self,
    *,
    now_ms: int,
    speed_units: str,
    speed_offset_kph: float,
    lead: LeadInfo,
  ) -> float:
    if not bool(lead.status):
      self._reset_lead_follow_deadband()
      return float(speed_offset_kph)

    half_kph, _ = _cc_units_kph(speed_units)
    soft_deadband_kph = max(float(self._LEAD_FOLLOW_SOFT_DEADBAND_MPH) * CV.MPH_TO_KPH, 0.95 * float(half_kph))
    persist_band_kph = max(float(self._LEAD_FOLLOW_PERSIST_BAND_MPH) * CV.MPH_TO_KPH, 1.8 * float(half_kph))

    offset_kph = float(speed_offset_kph)
    abs_offset_kph = abs(offset_kph)
    sign = 1 if offset_kph > 0.0 else (-1 if offset_kph < 0.0 else 0)

    if sign == 0 or abs_offset_kph < soft_deadband_kph:
      self._reset_lead_follow_deadband()
      return 0.0

    if sign != int(self._lead_follow_offset_sign):
      self._lead_follow_offset_sign = int(sign)
      self._lead_follow_offset_since_ms = int(now_ms)

    if abs_offset_kph < persist_band_kph:
      age_ms = int(now_ms) - int(self._lead_follow_offset_since_ms)
      if age_ms < int(self._LEAD_FOLLOW_PERSIST_MS):
        return 0.0

    return offset_kph

  def _fast_decel_required(self, *, v_ego_ms: float, lead: LeadInfo) -> bool:
    if not lead.status or lead.d_rel <= 0.0:
      return False

    collision_imminent = self._seconds_to_collision(lead=lead) < 4.0
    lead_absolute_speed_ms = float(v_ego_ms) + float(lead.v_rel)
    lead_too_slow = lead_absolute_speed_ms < float(self.MIN_CRUISE_SPEED_MS)
    return bool(collision_imminent or lead_too_slow)

  def _should_autoengage_cc(
    self,
    *,
    now_ms: int,
    v_ego_ms: float,
    brake_pressed: bool,
    lead: LeadInfo,
  ) -> bool:
    if float(v_ego_ms) < float(self.MIN_CRUISE_SPEED_MS):
      return False
    if bool(brake_pressed):
      return False
    if int(now_ms) <= int(self.fast_decel_time_ms) + int(self._FAST_DECEL_RESUME_HOLDOFF_MS):
      return False

    recent_lead = lead.status or ((int(now_ms) - int(self.lead_last_seen_time_ms)) < int(self._LEAD_FRESH_MS))
    if recent_lead and lead.status:
      if self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead):
        return False
      materially_closing = (lead.d_rel > 0.0) and (lead.v_rel < -1.2)
      if materially_closing:
        return False

    return True

  def _record_button(self, *, now_ms: int, button: int, speed_units: str) -> None:
    self.automated_action_time_ms = int(now_ms)
    self._last_auto_button = int(button)
    self._last_auto_button_time_ms = int(now_ms)
    if int(button) == int(CruiseButtons.CANCEL):
      self.fast_decel_time_ms = int(now_ms)
      return

    # Automated slowdowns should not lower the retained clear-road ceiling.
    if CruiseButtons.is_accel(int(button)):
      self._update_max_acc_speed_from_button(button=int(button), speed_units=speed_units)

  def _consume_recent_manual_raise_clear(
    self,
    *,
    now_ms: int,
    speed_limit_kph: float,
    current_kph: float,
    speed_units: str,
  ) -> bool:
    btn = int(self._last_human_button)
    if btn not in (int(CruiseButtons.RES_ACCEL), int(CruiseButtons.RES_ACCEL_2ND)):
      return False
    if (int(now_ms) - int(self._last_human_button_time_ms)) > int(self._AUTO_ECHO_IGNORE_MS):
      return False

    half_kph, _ = _cc_units_kph(speed_units)
    limit_kph = float(speed_limit_kph)
    if limit_kph <= 0.0:
      return False
    if limit_kph <= (float(self.acc_speed_kph) + max(0.5 * float(half_kph), 0.2)):
      return False
    if limit_kph <= (float(current_kph) + max(0.5 * float(half_kph), 0.2)):
      return False

    self.acc_speed_kph = max(float(limit_kph), float(self._manual_hold_restore_ceiling_kph))
    self._manual_lower_hold_active = False
    self._manual_hold_restore_ceiling_kph = 0.0
    self._manual_hold_restore_requested = False
    self._last_human_button = int(CruiseButtons.IDLE)
    self._last_human_button_time_ms = 0
    return True


  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    stock_cruise_enabled: bool,
    stock_cruise_state: str,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
    brake_pressed: bool = False,
    speed_limit_target_ms: Optional[float] = None,
    set_speed_limit_active: bool = False,
  ) -> AccDecision:
    self.note_human_buttons(cruise_buttons, speed_units=speed_units, now_ms=now_ms)
    lead = self._poll_lead(now_ms=now_ms)
    self._note_lead_transition(lead=lead, now_ms=now_ms)

    self.prev_speed_limit_kph = float(self.speed_limit_kph)
    if bool(set_speed_limit_active) and speed_limit_target_ms is not None and float(speed_limit_target_ms) > 0.0:
      self.speed_limit_kph = float(speed_limit_target_ms) * CV.MS_TO_KPH
      # A real posted-limit change may reseed the ACC ceiling, but an active
      # manual-lower hold must remain latched until the driver explicitly raises
      # it with the stalk.
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph) and not self._manual_lower_hold_active:
        self.acc_speed_kph = float(self.speed_limit_kph)
        self._manual_hold_restore_ceiling_kph = 0.0
        self._manual_hold_restore_requested = False
    else:
      self.speed_limit_kph = 0.0

    current_kph = float(current_set_speed_ms) * CV.MS_TO_KPH
    target_kph_seed = max(float(current_kph), float(desired_speed_ms) * CV.MS_TO_KPH)
    stock_state = str(stock_cruise_state or "").upper()
    self._observe_manual_set_speed_change(
      now_ms=now_ms,
      current_kph=float(current_kph),
      desired_kph=float(desired_speed_ms) * CV.MS_TO_KPH,
      speed_limit_kph=float(self.speed_limit_kph),
      speed_units=speed_units,
      brake_pressed=bool(brake_pressed),
      stock_cruise_state=stock_state,
    )
    manual_raise_cleared_to_limit = self._consume_recent_manual_raise_clear(
      now_ms=now_ms,
      speed_limit_kph=float(self.speed_limit_kph),
      current_kph=float(current_kph),
      speed_units=speed_units,
    )
    enabled_edge = bool(enabled) and not bool(self._prev_enabled)
    disabled_edge = (not bool(enabled)) and bool(self._prev_enabled)
    self._prev_enabled = bool(enabled)

    if disabled_edge:
      # Preserve a manual-lower hold across disengage; only an explicit manual
      # raise should clear it.
      if not self._manual_lower_hold_active:
        self._clear_road_ceiling_kph = max(
          float(self._clear_road_ceiling_kph),
          float(self.acc_speed_kph),
          float(current_kph),
          float(self.speed_limit_kph),
          float(target_kph_seed),
        )
      self._reset_accel_burst()
      self._clear_manual_pending()

    if not enabled:
      self._reset_accel_burst()
      self._clear_manual_pending()
      self._standby_autoengage_started_ms = 0
      self._standby_autoengage_last_set_ms = 0
      return AccDecision(None, "gated: not enabled")

    if (
      stock_state in ("ENABLED", "OVERRIDE", "STANDSTILL")
      and not bool(brake_pressed)
      and not bool(self._manual_lower_hold_active)
    ):
      self._clear_road_ceiling_kph = max(
        float(self._clear_road_ceiling_kph),
        float(self.acc_speed_kph),
        float(current_kph),
        float(self._last_current_set_speed_kph),
        float(v_ego_ms) * CV.MS_TO_KPH,
        float(self.speed_limit_kph),
        float(target_kph_seed),
      )

    if enabled_edge or self.acc_speed_kph <= 0.0:
      if self._manual_lower_hold_active:
        # Re-engage should keep the manually lowered ceiling latched.
        self.acc_speed_kph = max(float(self.acc_speed_kph), float(current_kph), 0.0)
      else:
        # Re-engage should restore a sane ceiling from the live set speed / ego speed.
        self.acc_speed_kph = max(
          float(current_kph),
          float(v_ego_ms) * CV.MS_TO_KPH,
          float(self.speed_limit_kph),
          float(target_kph_seed),
          float(self._manual_hold_restore_ceiling_kph),
          float(self._clear_road_ceiling_kph),
        )
        self._manual_hold_restore_ceiling_kph = 0.0
        self._manual_hold_restore_requested = False
    elif self._manual_hold_restore_requested:
      restore_kph = max(
        float(current_kph),
        float(v_ego_ms) * CV.MS_TO_KPH,
        float(self.speed_limit_kph),
        float(target_kph_seed),
        float(self._manual_hold_restore_ceiling_kph),
        float(self._clear_road_ceiling_kph),
      )
      self.acc_speed_kph = max(float(self.acc_speed_kph), float(restore_kph))
      self._manual_lower_hold_active = False
      self._manual_hold_restore_ceiling_kph = 0.0
      self._manual_hold_restore_requested = False
    elif self._manual_lower_hold_active:
      # Preserve a manually lowered ceiling until the driver explicitly raises it
      # again or a new posted speed-limit context supersedes it.
      self.acc_speed_kph = max(float(self.acc_speed_kph), 0.0)
    else:
      # Normal behavior: keep the internal ACC ceiling aligned with the active
      # clear-road ceiling so engage and post-curve recovery do not get stuck at
      # the initial set speed. Only raise the ceiling here; slowdowns still come
      # from the desired target path, not by lowering acc_speed_kph.
      self.acc_speed_kph = max(
        float(self.acc_speed_kph),
        float(current_kph),
        float(self.speed_limit_kph),
        float(target_kph_seed),
        float(self._clear_road_ceiling_kph),
      )
      self._manual_hold_restore_ceiling_kph = 0.0

    half_kph, full_kph = _cc_units_kph(speed_units)

    # An explicit manual raise is the only thing that should clear a manual-lower hold.

    if stock_state == "STANDBY":
      can_autoengage = (
        float(desired_speed_ms) >= float(v_ego_ms) - float(self._AUTOENGAGE_SPEED_WINDOW_MS)
        and self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS)
        and self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS)
        and self._should_autoengage_cc(now_ms=now_ms, v_ego_ms=v_ego_ms, brake_pressed=brake_pressed, lead=lead)
      )
      if can_autoengage:
        if int(self._standby_autoengage_started_ms) <= 0:
          self._standby_autoengage_started_ms = int(now_ms)

        elapsed_ms = int(now_ms) - int(self._standby_autoengage_started_ms)
        use_set_fallback = (
          elapsed_ms >= int(self._AUTOENGAGE_SET_FALLBACK_MS)
          and (int(now_ms) - int(self._standby_autoengage_last_set_ms)) >= int(self._AUTOENGAGE_SET_FALLBACK_REPEAT_MS)
        )

        if use_set_fallback:
          button = int(CruiseButtons.DECEL_SET)
          self._standby_autoengage_last_set_ms = int(now_ms)
          self._record_button(now_ms=now_ms, button=button, speed_units=speed_units)
          return AccDecision(button, "autoengage_set_fallback: SET", float(desired_speed_ms) * CV.MS_TO_KPH, current_kph, current_kph)

        self._record_button(now_ms=now_ms, button=int(CruiseButtons.RES_ACCEL), speed_units=speed_units)
        return AccDecision(int(CruiseButtons.RES_ACCEL), "autoengage: RES", float(desired_speed_ms) * CV.MS_TO_KPH, current_kph, current_kph + float(half_kph))

      self._standby_autoengage_started_ms = 0
      self._standby_autoengage_last_set_ms = 0
      return AccDecision(None, "standby: no autoengage", current_kph, current_kph, current_kph)

    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL"):
      self._standby_autoengage_started_ms = 0
      self._standby_autoengage_last_set_ms = 0
      return AccDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}", current_kph, current_kph, current_kph)

    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled", current_kph, current_kph, current_kph)

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=self._HUMAN_COOLDOWN_MS):
      return AccDecision(None, "gated: recent human action", current_kph, current_kph, current_kph)

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=self._AUTO_COOLDOWN_MS):
      return AccDecision(None, "gated: cooldown", current_kph, current_kph, current_kph)

    if float(desired_speed_ms) <= 0.1 or float(current_set_speed_ms) <= 0.1:
      return AccDecision(None, "gated: missing target/current", current_kph, current_kph, current_kph)

    # Keep emergency/below-min handling narrow. On no-lead curve/map targets that dip a
    # little below Tesla's minimum set-speed, prefer holding MIN instead of dropping ACC.
    fast_decel_required = self._fast_decel_required(v_ego_ms=v_ego_ms, lead=lead)
    min_cruise_kph = float(self.MIN_CRUISE_SPEED_MS) * CV.MS_TO_KPH

    min_hold_active = (
      (not fast_decel_required)
      and float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS)
      and float(desired_speed_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._MIN_CRUISE_HOLD_MARGIN_MS))
      and float(v_ego_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._MIN_CRUISE_CANCEL_VEGO_MARGIN_MS))
      and float(current_set_speed_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - (0.5 * CV.MPH_TO_MS))
    )

    target_speed_ms = max(float(desired_speed_ms), float(self.MIN_CRUISE_SPEED_MS)) if min_hold_active else float(desired_speed_ms)
    target_kph = float(target_speed_ms) * CV.MS_TO_KPH

    # A manual stalk-down hold is meant to be a real ceiling override until the driver
    # explicitly clears it with a manual raise. Clamp the effective target used for
    # automated accel decisions so LONG/mapd cannot silently climb back to the posted limit.
    effective_target_kph = float(target_kph)
    if self._manual_lower_hold_active and not self._manual_hold_restore_requested:
      effective_target_kph = min(float(effective_target_kph), float(self.acc_speed_kph))

    raw_speed_offset_kph = float(effective_target_kph) - float(current_kph)
    speed_offset_kph = self._lead_follow_effective_speed_offset_kph(
      now_ms=now_ms,
      speed_units=speed_units,
      speed_offset_kph=float(raw_speed_offset_kph),
      lead=lead,
    )
    desired_headroom_kph = max(float(self.acc_speed_kph), float(effective_target_kph)) - float(current_kph)
    available_speed_kph = max(0.0, float(desired_headroom_kph))

    button: Optional[int] = None

    if float(desired_speed_ms) < float(self.MIN_CRUISE_SPEED_MS) and (not min_hold_active):
      button = int(CruiseButtons.CANCEL)
    elif fast_decel_required and self._seconds_to_collision(lead=lead) < 2.5 and float(current_kph) > 0.0:
      button = int(CruiseButtons.CANCEL)
    elif speed_offset_kph < (-2.0 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.6 * float(full_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_2ND)
    elif speed_offset_kph < (-0.9 * float(half_kph)) and current_kph > 0.0:
      button = int(CruiseButtons.DECEL_SET)
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      button = self._choose_accel_button(
        now_ms=now_ms,
        speed_units=speed_units,
        speed_offset_kph=float(speed_offset_kph),
        available_speed_kph=float(available_speed_kph),
        lead=lead,
      )

    if button is None:
      reason = "no-op[min_hold]" if min_hold_active else "no-op"
      return AccDecision(None, reason, effective_target_kph, current_kph, current_kph)

    if button is not None and not CruiseButtons.is_accel(button):
      self._reset_accel_burst()

    if CruiseButtons.is_decel(button):
      if int(button) == int(CruiseButtons.DECEL_2ND) and (float(current_kph) - float(full_kph)) < float(min_cruise_kph):
        if (float(current_kph) - float(half_kph)) >= float(min_cruise_kph):
          button = int(CruiseButtons.DECEL_SET)
        else:
          button = None
      elif int(button) == int(CruiseButtons.DECEL_SET) and (float(current_kph) - float(half_kph)) < float(min_cruise_kph):
        button = None

    if button is None:
      reason = "no-op[min_cruise_guard]" if min_hold_active else "no-op[min_cruise_guard]"
      return AccDecision(None, reason, effective_target_kph, current_kph, current_kph)

    self._record_button(now_ms=now_ms, button=int(button), speed_units=speed_units)

    est_kph = float(current_kph)
    if int(button) == int(CruiseButtons.RES_ACCEL_2ND):
      est_kph += float(full_kph)
    elif int(button) == int(CruiseButtons.RES_ACCEL):
      est_kph += float(half_kph)
    elif int(button) == int(CruiseButtons.DECEL_2ND):
      est_kph = max(0.0, est_kph - float(full_kph))
    elif int(button) == int(CruiseButtons.DECEL_SET):
      est_kph = max(0.0, est_kph - float(half_kph))
    else:
      est_kph = 0.0

    reason = "cancel" if int(button) == int(CruiseButtons.CANCEL) else "press"
    return AccDecision(int(button), reason, effective_target_kph, current_kph, est_kph)

  def _clear_manual_pending(self) -> None:
    self._manual_lower_pending = False
    self._manual_lower_pending_time_ms = 0
    self._manual_raise_pending = False
    self._manual_raise_pending_time_ms = 0

  def _pending_manual_lower(self, *, now_ms: int) -> bool:
    return bool(self._manual_lower_pending) and (int(now_ms) - int(self._manual_lower_pending_time_ms)) <= int(self._MANUAL_PENDING_TIMEOUT_MS)

  def _pending_manual_raise(self, *, now_ms: int) -> bool:
    return bool(self._manual_raise_pending) and (int(now_ms) - int(self._manual_raise_pending_time_ms)) <= int(self._MANUAL_PENDING_TIMEOUT_MS)

