# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Human-tuned stock cruise syncing for XNOR.

This keeps the stable owner split and the no-lead curve behavior, while
making lead-follow recovery smoother and less sticky:

- lead-following still stays on the planner tail while a lead is constraining
- opening / non-constraining leads stop capping the target too aggressively
- small follow-speed oscillations no longer latch extra far-queue protection
- no-lead curve control remains planner-first, with mapd only helping release

XNOR architecture adaptations retained:
- 5 Hz cruise stalk pacing
- stock cruise state gating
- startup freshness guard for the planner stream
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from openpilot.common.swaglog import cloudlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _mono_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS
  _ROADWORKS_CAP_FILE = "/data/xnor_roadworks_speed_cap_kph.txt"
  _LEAD_NIBBLE_HOLD_MAX_DROP_MS = 2.4 * CV.MPH_TO_MS
  _LEAD_NIBBLE_HOLD_MIN_DREL_M = 32.0
  _LEAD_NIBBLE_HOLD_MIN_VREL_MS = -1.35
  _LEAD_NIBBLE_HOLD_MAX_ATARGET_MS2 = -0.30
  _LP_FRESH_NS = 1_500_000_000
  _PLANNER_DRAG_MARGIN_MS = 0.25
  _PLANNER_BELOW_EGO_MARGIN_MS = 0.05
  _STRONG_DECEL_ATARGET_MS2 = -0.7

  _CURVE_ENTRY_PERSIST_MS = 320
  _CURVE_EXIT_PERSIST_MS = 120
  _CURVE_EXIT_RECOVERY_MS_PER_S = 7.0
  _CURVE_HOLD_ENTRY_FREEZE_MS = 650
  _CURVE_HOLD_DROP_PERSIST_MS = 260
  _CURVE_HOLD_DROP_RATE_MS_PER_S = 1.8
  _CURVE_HOLD_HARD_DROP_EXTRA_MS = 5.0 * CV.MPH_TO_MS
  _CURVE_ENTRY_PREVIEW_DROP_MS = 2.0 * CV.MPH_TO_MS
  _CURVE_MIN_CRUISE_HOLD_MARGIN_MS = 5.0 * CV.MPH_TO_MS
  _CURVE_MAPD_MIN_HOLD_MARGIN_MS = 0.5 * CV.MPH_TO_MS
  _CURVE_HARD_ENTRY_EXTRA_MS = 3.0 * CV.MPH_TO_MS
  _CURVE_RELEASE_NEAR_TARGET_MARGIN_MS = 0.4 * CV.MPH_TO_MS
  _CURVE_HOLD_DROP_DEADBAND_MS = 2.0 * CV.MPH_TO_MS
  _MAPD_FRESH_NS = 1_500_000_000
  _CURVE_MAPD_RELEASE_PERSIST_MS = 220
  _CURVE_PLANNER_RELEASE_PERSIST_MS = 180
  _CURVE_PLANNER_RELEASE_MAPD_TOLERANCE_MS = 5.5 * CV.MPH_TO_MS
  _CURVE_PLANNER_RELEASE_OVERRIDE_MS = 260
  _LEAD_HOLD_PERSIST_MS = 560
  _LEAD_HOLD_RELEASE_MARGIN_MS = 0.20 * CV.MPH_TO_MS
  _LEAD_OPENING_VREL_MS = 0.12
  _LEAD_OPENING_GAP_MIN_M = 18.0
  _LEAD_OPENING_TIME_GAP_S = 1.15
  _LEAD_CONSTRAIN_CLOSING_VREL_MS = -0.15
  _LEAD_CONSTRAIN_GAP_MIN_M = 22.0
  _LEAD_CONSTRAIN_TIME_GAP_S = 1.7
  _LEAD_OFFLANE_YREL_M = 2.2
  _LEAD_OPENING_RELAX_ATARGET_MS2 = -0.15
  _NO_LEAD_MAPD_CURRENT_GATE_MS = 0.5 * CV.MPH_TO_MS
  _MAPD_ONLY_ENTRY_PERSIST_MS = 180
  _MAPD_ONLY_HIGHWAY_ENTRY_PERSIST_MS = 460
  _MAPD_ONLY_ENTRY_MIN_DROP_MS = 2.0 * CV.MPH_TO_MS
  _MAPD_DUAL_SOURCE_AGREE_MS = 4.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_SPEED_MS = 55.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_DROP_MS = 24.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_PLANNER_DELTA_MS = 16.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_PLANNER_SUPPORT_DROP_MS = 4.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_EXTRA_DROP_WITH_PLANNER_MS = 12.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_NEAR_STEER_DEG = 2.5
  _LEAD_CLEAR_MAPD_GRACE_MS = 900
  _LEAD_CLEAR_OPENING_GRACE_MS = 220
  _CURVE_REENTRY_BLOCK_MS = 1800
  _CURVE_REENTRY_ALLOW_DROP_MS = 6.0 * CV.MPH_TO_MS
  _CURVE_REENTRY_ALLOW_STEER_DEG = 2.0
  _PLANNER_ONLY_REENTRY_BLOCK_DROP_MS = 3.0 * CV.MPH_TO_MS
  _PLANNER_ONLY_REENTRY_ALLOW_STEER_DEG = 3.0
  _PLANNER_CURVE_ENTRY_MARGIN_MS = 1.0 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_ENTRY_STEER_DEG = 5.0
  _CURVE_LIMIT_GUARD_FULL_STEER_DEG = 9.0
  _CURVE_LIMIT_GUARD_MIN_SPEED_MS = 25.0 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_ENTRY_PERSIST_MS = 120
  _CURVE_LIMIT_GUARD_RELEASE_PERSIST_MS = 650
  _CURVE_LIMIT_GUARD_ACTIVATION_DROP_MS = 1.0 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_VEGO_MARGIN_MS = 0.15 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_MIN_DROP_MS = 1.5 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_MAX_DROP_MS = 5.5 * CV.MPH_TO_MS
  _CONTROLS_STATE_FRESH_NS = 500_000_000
  _CURVE_LIMIT_GUARD_FALLBACK_STEER_DEG = 7.5
  _MAPD_LOW_SPEED_ENTRY_SPEED_MS = 45.0 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_SHARP_DROP_MS = 3.0 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_PLANNER_HINT_DROP_MS = 0.4 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_STEER_HINT_DEG = 0.35
  _SHARP_CURVE_FAST_ENTRY_DROP_MS = 6.0 * CV.MPH_TO_MS
  _LP_QUEUE_FALLBACK_MAX_SPEED_MS = 45.0 * CV.MPH_TO_MS
  _LP_QUEUE_FALLBACK_DROP_MS = 1.0 * CV.MPH_TO_MS
  _LP_QUEUE_FALLBACK_EGO_MARGIN_MS = 0.5 * CV.MPH_TO_MS
  _LP_QUEUE_FALLBACK_ATARGET_MS2 = -0.25
  _MAPD_STRAIGHT_ONLY_ENTRY_DROP_MS = 4.0 * CV.MPH_TO_MS
  _MAPD_STRAIGHT_ONLY_STEER_DEG = 1.0
  _PLANNER_SET_TRACK_MARGIN_MS = 0.8 * CV.MPH_TO_MS
  _WEAK_LEAD_OWNER_VREL_MS = -0.35
  _WEAK_LEAD_OWNER_ATARGET_MS2 = -0.35
  _WEAK_LEAD_OWNER_TIME_GAP_S = 2.2

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan", "radarState", "mapdOut", "controlsState"])

    self._lp_target_last_ms: Optional[float] = None
    self._lp_target_near_ms: Optional[float] = None
    self._lp_last_ns: int = 0
    self._lp_has_lead: bool = False
    self._lp_a_target: float = 0.0
    self._lp_source: str = ""

    self._lead_present: bool = False
    self._lead_drel: float = 0.0
    self._lead_vrel: float = 0.0
    self._lead_yrel: float = 0.0
    self._mapd_suggested_ms: Optional[float] = None
    self._mapd_map_curve_ms: Optional[float] = None
    self._mapd_vision_curve_ms: Optional[float] = None
    self._mapd_last_ns: int = 0

    self._last_info_log_ms: int = 0
    self._enabled_since_ms: int = 0
    self._last_active: bool = False
    self._last_lp_seen_ns: int = 0
    self._stable_plan_samples: int = 0

    self._curve_entry_candidate_since_ms: int = 0
    self._curve_exit_candidate_since_ms: int = 0
    self._curve_hold_active: bool = False
    self._curve_hold_target_ms: float = 0.0
    self._curve_hold_last_update_ms: int = 0
    self._curve_mapd_release_candidate_since_ms: int = 0
    self._curve_planner_release_candidate_since_ms: int = 0
    self._mapd_entry_candidate_since_ms: int = 0
    self._lead_hold_until_ms: int = 0
    self._lead_recently_cleared_until_ms: int = 0
    self._lead_present_prev: bool = False
    self._lead_last_drel: float = 0.0
    self._lead_last_vrel: float = 0.0
    self._lead_last_yrel: float = 0.0
    self._curve_recent_clear_until_ms: int = 0
    self._curve_limit_guard_candidate_since_ms: int = 0
    self._curve_limit_guard_release_candidate_since_ms: int = 0
    self._curve_limit_guard_active: bool = False
    self._controls_state_last_ns: int = 0
    self._lat_limit_saturated: bool = False
    self._lat_limit_severity: float = 0.0
    self._lat_limit_source: str = ""

  def _rate_log(self, msg: str) -> None:
    now = _mono_ms()
    if now - int(self._last_info_log_ms) < 1000:
      return
    self._last_info_log_ms = int(now)
    cloudlog.info(msg)


  @staticmethod
  def _safe_finite_float(value: object, default: float = 0.0) -> float:
    try:
      out = float(value if value is not None else default)
    except Exception:
      return float(default)
    if not math.isfinite(out):
      return float(default)
    return float(out)

  def _controls_state_is_fresh(self, *, now_ns: int) -> bool:
    try:
      controls_mono_ns = int(self._sm.logMonoTime.get("controlsState", 0) or 0)
    except Exception:
      controls_mono_ns = 0
    if controls_mono_ns <= 0:
      return False
    self._controls_state_last_ns = int(controls_mono_ns)
    try:
      controls_valid = bool(self._sm.valid.get("controlsState", False))
    except Exception:
      controls_valid = False
    if not controls_valid:
      return False
    age_ns = int(now_ns) - int(controls_mono_ns)
    return 0 <= age_ns < int(self._CONTROLS_STATE_FRESH_NS)

  def _refresh_lateral_limit_state(self, *, now_ns: int) -> None:
    self._lat_limit_saturated = False
    self._lat_limit_severity = 0.0
    self._lat_limit_source = ""
    if not self._controls_state_is_fresh(now_ns=int(now_ns)):
      return
    try:
      controls_state = self._sm["controlsState"]
    except Exception:
      return
    try:
      lat_saturated, lat_severity, lat_source = self._extract_lateral_limit_signal(controls_state)
    except Exception:
      return
    self._lat_limit_saturated = bool(lat_saturated)
    self._lat_limit_severity = max(0.0, self._safe_finite_float(lat_severity, 0.0))
    self._lat_limit_source = str(lat_source or "")

  @staticmethod
  def _extract_plan_speed_last(lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None
    try:
      v_last = float(speeds[-1])
    except Exception:
      return None
    if not (math.isfinite(v_last) and v_last >= 0.0):
      return None
    return v_last

  @staticmethod
  def _extract_plan_speed_near_min(lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None
    valid_speeds: list[float] = []
    try:
      count = max(4, int(math.ceil(len(speeds) * 0.40)))
      for v in speeds[:count]:
        vf = float(v)
        if math.isfinite(vf) and vf >= 0.0:
          valid_speeds.append(vf)
    except Exception:
      return None
    if not valid_speeds:
      return None
    ordered = sorted(valid_speeds)
    quantile_idx = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * 0.25))))
    return float(ordered[quantile_idx])

  @staticmethod
  def _curve_entry_threshold_ms(reference_ms: float) -> float:
    if float(reference_ms) >= (55.0 * CV.MPH_TO_MS):
      return 8.0 * CV.MPH_TO_MS
    return 2.0 * CV.MPH_TO_MS

  @staticmethod
  def _curve_exit_margin_ms(reference_ms: float) -> float:
    if float(reference_ms) >= (60.0 * CV.MPH_TO_MS):
      return 2.0 * CV.MPH_TO_MS
    if float(reference_ms) >= (40.0 * CV.MPH_TO_MS):
      return 1.5 * CV.MPH_TO_MS
    return 1.0 * CV.MPH_TO_MS

  @staticmethod
  def _reference_speed_ms(*, base_target_ms: Optional[float], current_set_ms: float, v_ego_ms: float) -> float:
    if base_target_ms is not None and float(base_target_ms) > 0.0:
      return float(base_target_ms)
    if float(current_set_ms) > 0.0:
      return float(current_set_ms)
    return float(max(0.0, v_ego_ms))

  def _resume_ceiling_ms(self, *, current_set_ms: float, v_ego_ms: float) -> float:
    retained_ceiling_ms = max(0.0, float(self.acc.acc_speed_kph) * CV.KPH_TO_MS)
    return max(float(current_set_ms), float(v_ego_ms), float(retained_ceiling_ms))

  def _mapd_curve_target_ms(self, *, now_ns: int) -> Optional[float]:
    if self._mapd_suggested_ms is None:
      return None
    if int(self._mapd_last_ns) <= 0:
      return None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None
    suggested_ms = float(self._mapd_suggested_ms)
    if not (math.isfinite(suggested_ms) and suggested_ms > 0.1):
      return None
    return suggested_ms


  def _curve_specific_mapd_target_ms(self, *, now_ns: int) -> Optional[float]:
    if int(self._mapd_last_ns) <= 0:
      return None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None

    candidates: list[float] = []
    for raw in (self._mapd_map_curve_ms, self._mapd_vision_curve_ms):
      if raw is None:
        continue
      val = float(raw)
      if math.isfinite(val) and val > 0.1:
        candidates.append(val)

    if not candidates:
      return None
    return float(min(candidates))

  def _curve_specific_mapd_sources(self, *, now_ns: int) -> tuple[Optional[float], Optional[float]]:
    if int(self._mapd_last_ns) <= 0:
      return None, None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None, None

    def _clean(raw: Optional[float]) -> Optional[float]:
      if raw is None:
        return None
      val = float(raw)
      if math.isfinite(val) and val > 0.1:
        return val
      return None

    return _clean(self._mapd_map_curve_ms), _clean(self._mapd_vision_curve_ms)


  def _mapd_curve_active_target_ms(self, *, now_ns: int) -> Optional[float]:
    # Only curve-specific mapd sources are allowed to own active no-lead curve
    # slowdown. Generic suggestedSpeed repeatedly re-opened curve_hold[mapd] on
    # straight 30/40 roads and held the set at ~26/30 mph in the watcher logs.
    curve_specific_ms = self._curve_specific_mapd_target_ms(now_ns=now_ns)
    if curve_specific_ms is not None:
      return float(curve_specific_ms)
    return None


  def _mapd_entry_target_ms(
    self,
    *,
    now_ms: int,
    now_ns: int,
    reference_ms: float,
    planner_near_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
    planner_curve_active: bool,
  ) -> Optional[float]:
    curve_specific_ms = self._curve_specific_mapd_target_ms(now_ns=now_ns)
    if curve_specific_ms is None:
      self._mapd_entry_candidate_since_ms = 0
      return None

    reference_ms = float(reference_ms)
    planner_near_ms = float(planner_near_ms)
    v_ego_ms = float(v_ego_ms)
    current_angle_deg = abs(float(current_angle_deg))

    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)
    entry_threshold_ms = float(self._curve_entry_threshold_ms(reference_ms))
    mapd_only_min_drop_ms = max(float(self._MAPD_ONLY_ENTRY_MIN_DROP_MS), float(entry_threshold_ms))
    strong_drop_ms = max(float(self._SHARP_CURVE_FAST_ENTRY_DROP_MS), float(entry_threshold_ms))

    # After a real curve clear, block weak mapd re-entry on straight road unless there
    # is already meaningful steering input or a clearly lower planner/mapd target.
    if (
      int(now_ms) <= int(self._curve_recent_clear_until_ms)
      and current_angle_deg <= float(self._CURVE_REENTRY_ALLOW_STEER_DEG)
      and planner_near_ms >= (reference_ms - float(self._CURVE_REENTRY_ALLOW_DROP_MS))
      and float(curve_specific_ms) >= (reference_ms - float(self._CURVE_REENTRY_ALLOW_DROP_MS))
    ):
      self._mapd_entry_candidate_since_ms = 0
      return None

    if float(curve_specific_ms) >= (reference_ms - max(float(self._NO_LEAD_MAPD_CURRENT_GATE_MS), release_margin_ms)):
      self._mapd_entry_candidate_since_ms = 0
      return None

    if (not bool(planner_curve_active)) and float(curve_specific_ms) >= (reference_ms - mapd_only_min_drop_ms):
      self._mapd_entry_candidate_since_ms = 0
      return None

    if (
      (not bool(planner_curve_active))
      and current_angle_deg <= float(self._MAPD_STRAIGHT_ONLY_STEER_DEG)
      and planner_near_ms >= (reference_ms - float(self._PLANNER_CURVE_ENTRY_MARGIN_MS))
      and float(curve_specific_ms) >= (reference_ms - float(self._MAPD_STRAIGHT_ONLY_ENTRY_DROP_MS))
    ):
      self._mapd_entry_candidate_since_ms = 0
      return None

    if float(curve_specific_ms) >= (v_ego_ms - (0.10 * CV.MPH_TO_MS)):
      self._mapd_entry_candidate_since_ms = 0
      return None

    highway_speed = v_ego_ms >= float(self._MAPD_ONLY_HIGHWAY_SPEED_MS)
    planner_supports_meaningful_drop = planner_near_ms <= (reference_ms - float(self._MAPD_ONLY_HIGHWAY_PLANNER_SUPPORT_DROP_MS))
    planner_far_from_mapd = planner_near_ms >= (float(curve_specific_ms) + float(self._MAPD_ONLY_HIGHWAY_MAX_PLANNER_DELTA_MS))
    mapd_much_lower_than_planner = planner_near_ms >= (float(curve_specific_ms) + float(self._MAPD_ONLY_HIGHWAY_MAX_EXTRA_DROP_WITH_PLANNER_MS))
    near_straight = current_angle_deg <= float(self._MAPD_ONLY_HIGHWAY_NEAR_STEER_DEG)
    recent_lead_clear = int(now_ms) <= int(self._lead_recently_cleared_until_ms)
    map_curve_ms, vision_curve_ms = self._curve_specific_mapd_sources(now_ns=now_ns)
    dual_source_agreement = (
      map_curve_ms is not None
      and vision_curve_ms is not None
      and abs(float(map_curve_ms) - float(vision_curve_ms)) <= float(self._MAPD_DUAL_SOURCE_AGREE_MS)
    )
    sharp_curve_hint = (
      float(curve_specific_ms) <= (reference_ms - strong_drop_ms)
      and (
        current_angle_deg >= float(self._CURVE_REENTRY_ALLOW_STEER_DEG)
        or planner_near_ms <= (reference_ms - (0.45 * strong_drop_ms))
        or dual_source_agreement
      )
    )

    low_speed = v_ego_ms <= float(self._MAPD_LOW_SPEED_ENTRY_SPEED_MS)
    low_speed_sharp_entry = (
      low_speed
      and float(curve_specific_ms) <= (reference_ms - float(self._MAPD_LOW_SPEED_SHARP_DROP_MS))
      and (
        dual_source_agreement
        or planner_near_ms <= (reference_ms - float(self._MAPD_LOW_SPEED_PLANNER_HINT_DROP_MS))
        or current_angle_deg >= float(self._MAPD_LOW_SPEED_STEER_HINT_DEG)
        or float(curve_specific_ms) <= (reference_ms - float(self._SHARP_CURVE_FAST_ENTRY_DROP_MS))
      )
    )

    if low_speed_sharp_entry:
      self._mapd_entry_candidate_since_ms = 0
      return float(curve_specific_ms)

    if bool(planner_curve_active):
      if highway_speed:
        suspicious_large_drop = float(curve_specific_ms) <= (reference_ms - float(self._MAPD_ONLY_HIGHWAY_MAX_DROP_MS))
        mismatch_without_support = mapd_much_lower_than_planner and near_straight and (not planner_supports_meaningful_drop)
        if suspicious_large_drop and (not dual_source_agreement) and (mismatch_without_support or recent_lead_clear):
          self._mapd_entry_candidate_since_ms = 0
          return None
      self._mapd_entry_candidate_since_ms = 0
      return float(curve_specific_ms)

    persist_ms = int(self._MAPD_ONLY_ENTRY_PERSIST_MS)
    if highway_speed:
      suspicious_large_drop = float(curve_specific_ms) <= (reference_ms - float(self._MAPD_ONLY_HIGHWAY_MAX_DROP_MS))
      mismatch_without_support = mapd_much_lower_than_planner and near_straight and (not dual_source_agreement)
      if suspicious_large_drop and (planner_far_from_mapd or mismatch_without_support):
        if not dual_source_agreement and not sharp_curve_hint:
          self._mapd_entry_candidate_since_ms = 0
          return None
      persist_ms = int(self._MAPD_ONLY_HIGHWAY_ENTRY_PERSIST_MS)

    if dual_source_agreement:
      persist_ms = min(int(persist_ms), 220)

    if sharp_curve_hint:
      persist_ms = min(int(persist_ms), 120)

    if recent_lead_clear:
      persist_ms = max(int(persist_ms), int(self._LEAD_CLEAR_MAPD_GRACE_MS))

    if int(self._mapd_entry_candidate_since_ms) == 0:
      self._mapd_entry_candidate_since_ms = int(now_ms)
      return None

    if (int(now_ms) - int(self._mapd_entry_candidate_since_ms)) < int(persist_ms):
      return None

    return float(curve_specific_ms)

  def _mapd_curve_floor_ms(self, *, now_ns: int) -> Optional[float]:
    if int(self._mapd_last_ns) <= 0:
      return None
    if (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      return None

    candidates: list[float] = []
    for raw in (self._mapd_map_curve_ms, self._mapd_vision_curve_ms):
      if raw is None:
        continue
      val = float(raw)
      if math.isfinite(val) and val > 0.1:
        candidates.append(val)
    if not candidates:
      return None
    return float(max(candidates))

  def _should_hold_min_cruise_for_curve(self, *, now_ns: int, desired_ms: float, no_lead: bool) -> bool:
    if (not no_lead) or float(desired_ms) >= float(self.MIN_CRUISE_SPEED_MS):
      return False

    if float(desired_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MIN_CRUISE_HOLD_MARGIN_MS)):
      return True

    curve_floor_ms = self._mapd_curve_floor_ms(now_ns=now_ns)
    if curve_floor_ms is None:
      return False

    return float(curve_floor_ms) >= (float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MAPD_MIN_HOLD_MARGIN_MS))

  def _maybe_release_curve_hold_from_mapd(self, *, now_ms: int, now_ns: int, reference_ms: float) -> bool:
    if not self._curve_hold_active:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    reference_ms = float(reference_ms)
    if reference_ms <= 0.1:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    curve_specific_ms = self._curve_specific_mapd_target_ms(now_ns=now_ns)
    if curve_specific_ms is None:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)
    if float(curve_specific_ms) < (float(reference_ms) - float(release_margin_ms)):
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    if int(self._curve_mapd_release_candidate_since_ms) == 0:
      self._curve_mapd_release_candidate_since_ms = int(now_ms)
      return False

    # Keep mapd-only release as a soft clear signal; planner still confirms the final release path.
    return (int(now_ms) - int(self._curve_mapd_release_candidate_since_ms)) >= int(self._CURVE_MAPD_RELEASE_PERSIST_MS)

    mapd_target_ms = self._mapd_curve_active_target_ms(now_ns=now_ns)
    if mapd_target_ms is None:
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)
    if float(mapd_target_ms) < (float(reference_ms) - float(release_margin_ms)):
      self._curve_mapd_release_candidate_since_ms = 0
      return False

    if int(self._curve_mapd_release_candidate_since_ms) == 0:
      self._curve_mapd_release_candidate_since_ms = int(now_ms)
      return False

    if (int(now_ms) - int(self._curve_mapd_release_candidate_since_ms)) >= int(self._CURVE_MAPD_RELEASE_PERSIST_MS):
      self._reset_curve_hold()
      return True
    return False

  def _should_force_curve_release(
    self,
    *,
    now_ms: int,
    now_ns: int,
    reference_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
  ) -> bool:
    if not self._curve_hold_active:
      self._curve_planner_release_candidate_since_ms = 0
      return False

    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)
    planner_last_clear = float(planner_last_ms) >= (float(reference_ms) - release_margin_ms)
    planner_near_clear = float(planner_near_ms) >= (float(reference_ms) - (2.2 * release_margin_ms))

    if not (planner_last_clear and planner_near_clear):
      self._curve_planner_release_candidate_since_ms = 0
      return False

    if int(self._curve_planner_release_candidate_since_ms) == 0:
      self._curve_planner_release_candidate_since_ms = int(now_ms)
      return False

    elapsed_ms = max(0, int(now_ms) - int(self._curve_planner_release_candidate_since_ms))

    curve_specific_ms = self._curve_specific_mapd_target_ms(now_ns=now_ns)
    if curve_specific_ms is not None:
      mapd_release_margin_ms = max(
        float(release_margin_ms),
        float(self._CURVE_PLANNER_RELEASE_MAPD_TOLERANCE_MS),
      )
      mapd_clear = float(curve_specific_ms) >= (float(reference_ms) - mapd_release_margin_ms)
      if (not mapd_clear) and (elapsed_ms < int(self._CURVE_PLANNER_RELEASE_OVERRIDE_MS)):
        return False

    return elapsed_ms >= int(self._CURVE_PLANNER_RELEASE_PERSIST_MS)

  def _reset_curve_hold(self) -> None:
    self._curve_entry_candidate_since_ms = 0
    self._curve_exit_candidate_since_ms = 0
    self._curve_hold_active = False
    self._curve_hold_target_ms = 0.0
    self._curve_hold_last_update_ms = 0
    self._curve_mapd_release_candidate_since_ms = 0
    self._curve_planner_release_candidate_since_ms = 0
    self._mapd_entry_candidate_since_ms = 0

  def _reset_lead_hold(self) -> None:
    self._lead_hold_until_ms = 0

  def _lead_is_opening_clear(self, *, base_target_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False

    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return True

    opening_gap_m = max(float(self._LEAD_OPENING_GAP_MIN_M), float(v_ego_ms) * float(self._LEAD_OPENING_TIME_GAP_S))
    lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
    planner_not_decel = float(self._lp_a_target) >= float(self._LEAD_OPENING_RELAX_ATARGET_MS2)
    opening_gap_ok = float(self._lead_drel) >= float(opening_gap_m)
    opening_speed_ok = float(self._lead_vrel) >= float(self._LEAD_OPENING_VREL_MS)

    return bool(
      opening_speed_ok
      and opening_gap_ok
      and planner_not_decel
      and (
        float(lead_speed_ms) >= (float(v_ego_ms) - float(self._LEAD_HOLD_RELEASE_MARGIN_MS))
        or str(self._lp_source or "") in ("cruise", "e2e")
      )
    )



  def _lead_last_sample_was_opening_clear(self) -> bool:
    if float(self._lead_last_drel) <= 0.0:
      return False
    if abs(float(self._lead_last_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return True
    planner_not_decel = float(self._lp_a_target) >= float(self._LEAD_OPENING_RELAX_ATARGET_MS2)
    opening_gap_ok = float(self._lead_last_drel) >= max(float(self._LEAD_OPENING_GAP_MIN_M), 18.0)
    opening_speed_ok = float(self._lead_last_vrel) >= float(self._LEAD_OPENING_VREL_MS)
    return bool(
      opening_speed_ok
      and opening_gap_ok
      and planner_not_decel
      and str(self._lp_source or "") in ("cruise", "e2e", "")
    )


  def _lead_context_active_now(self, *, now_ms: int) -> bool:
    if bool(self._lead_present) and float(self._lead_drel) > 0.0:
      return True
    return int(now_ms) <= int(self._lead_recently_cleared_until_ms)

  def _lead_hold_active_now(self, *, now_ms: int, base_target_ms: float, planner_ms: float, v_ego_ms: float) -> bool:
    if int(self._lead_hold_until_ms) <= 0 or int(now_ms) > int(self._lead_hold_until_ms):
      self._lead_hold_until_ms = 0
      return False
    if float(planner_ms) <= 0.1:
      return False
    if self._lead_is_opening_clear(base_target_ms=float(base_target_ms), v_ego_ms=float(v_ego_ms)):
      self._lead_hold_until_ms = 0
      return False

    fallback_active = self._lp_queue_fallback_active(
      now_ms=int(now_ms),
      base_target_ms=float(base_target_ms),
      planner_ms=float(planner_ms),
      v_ego_ms=float(v_ego_ms),
    )
    if (not self._lead_present) and (not fallback_active):
      self._lead_hold_until_ms = 0
      return False

    return float(planner_ms) < (float(base_target_ms) - float(self._LEAD_HOLD_RELEASE_MARGIN_MS))

  def _stabilize_no_lead_curve_target(self, *, now_ms: int, raw_target_ms: float, reference_ms: float) -> tuple[float, str]:
    raw_target_ms = float(raw_target_ms)
    reference_ms = float(reference_ms)
    if raw_target_ms <= 0.1 or reference_ms <= 0.1:
      self._reset_curve_hold()
      return raw_target_ms, "curve_invalid"

    delta_ms = max(0.0, reference_ms - raw_target_ms)
    entry_threshold_ms = float(self._curve_entry_threshold_ms(reference_ms))
    exit_margin_ms = float(self._curve_exit_margin_ms(reference_ms))
    hard_entry_threshold_ms = float(entry_threshold_ms + self._CURVE_HARD_ENTRY_EXTRA_MS)
    release_margin_ms = float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS)

    if raw_target_ms >= (reference_ms - release_margin_ms):
      self._reset_curve_hold()
      return reference_ms, "curve_clear"

    if not self._curve_hold_active:
      if delta_ms >= hard_entry_threshold_ms:
        self._curve_hold_active = True
        self._curve_hold_target_ms = max(
          float(raw_target_ms),
          float(reference_ms) - float(self._CURVE_ENTRY_PREVIEW_DROP_MS),
        )
        self._curve_hold_last_update_ms = int(now_ms)
        self._curve_entry_candidate_since_ms = 0
        self._curve_exit_candidate_since_ms = 0
        return float(self._curve_hold_target_ms), "curve_hold(hard_entry)"

      if delta_ms >= entry_threshold_ms:
        if int(self._curve_entry_candidate_since_ms) == 0:
          self._curve_entry_candidate_since_ms = int(now_ms)
        elapsed_ms = max(0, int(now_ms) - int(self._curve_entry_candidate_since_ms))
        preview_ratio = min(1.0, float(elapsed_ms) / max(1.0, float(self._CURVE_ENTRY_PERSIST_MS)))
        preview_drop_ms = min(float(self._CURVE_ENTRY_PREVIEW_DROP_MS), float(delta_ms)) * preview_ratio
        preview_target_ms = max(float(raw_target_ms), float(reference_ms) - preview_drop_ms)
        if elapsed_ms >= int(self._CURVE_ENTRY_PERSIST_MS):
          self._curve_hold_active = True
          self._curve_hold_target_ms = float(preview_target_ms)
          self._curve_hold_last_update_ms = int(now_ms)
          self._curve_entry_candidate_since_ms = 0
          self._curve_exit_candidate_since_ms = 0
          return float(self._curve_hold_target_ms), "curve_hold(entry)"
        return float(preview_target_ms), "curve_gate(wait)"
      self._curve_entry_candidate_since_ms = 0
      return reference_ms, "curve_gate(clear)"

    time_since_hold_update_ms = max(0, int(now_ms) - int(self._curve_hold_last_update_ms))
    dt_s = max(0.0, float(time_since_hold_update_ms) / 1000.0)
    freeze_active = time_since_hold_update_ms < int(self._CURVE_HOLD_ENTRY_FREEZE_MS)
    self._curve_hold_last_update_ms = int(now_ms)

    drop_deadband_ms = float(self._CURVE_HOLD_DROP_DEADBAND_MS)
    hard_drop_ms = float(drop_deadband_ms + self._CURVE_HOLD_HARD_DROP_EXTRA_MS)

    if raw_target_ms < (float(self._curve_hold_target_ms) - drop_deadband_ms):
      immediate_hard_drop = raw_target_ms < (float(self._curve_hold_target_ms) - hard_drop_ms)
      if freeze_active and (not immediate_hard_drop):
        self._curve_exit_candidate_since_ms = 0
      else:
        if int(self._curve_entry_candidate_since_ms) == 0:
          self._curve_entry_candidate_since_ms = int(now_ms)
        drop_persist_satisfied = immediate_hard_drop or (
          (int(now_ms) - int(self._curve_entry_candidate_since_ms)) >= int(self._CURVE_HOLD_DROP_PERSIST_MS)
        )
        if drop_persist_satisfied:
          max_drop_ms = float(self._CURVE_HOLD_DROP_RATE_MS_PER_S) * max(dt_s, 0.20)
          self._curve_hold_target_ms = max(float(raw_target_ms), float(self._curve_hold_target_ms) - max_drop_ms)
          self._curve_entry_candidate_since_ms = 0
          self._curve_exit_candidate_since_ms = 0
    elif raw_target_ms > (float(self._curve_hold_target_ms) + exit_margin_ms):
      self._curve_entry_candidate_since_ms = 0
      if int(self._curve_exit_candidate_since_ms) == 0:
        self._curve_exit_candidate_since_ms = int(now_ms)
      if (int(now_ms) - int(self._curve_exit_candidate_since_ms)) >= int(self._CURVE_EXIT_PERSIST_MS):
        recovery_ms = float(self._CURVE_EXIT_RECOVERY_MS_PER_S) * dt_s
        self._curve_hold_target_ms = min(float(raw_target_ms), float(self._curve_hold_target_ms) + recovery_ms)
    else:
      self._curve_entry_candidate_since_ms = 0
      self._curve_exit_candidate_since_ms = 0

    desired_ms = float(self._curve_hold_target_ms)
    if (float(raw_target_ms) >= (float(desired_ms) - 0.05)) and (delta_ms < (0.5 * entry_threshold_ms)):
      self._reset_curve_hold()
      return float(raw_target_ms), "curve_release"

    return float(desired_ms), "curve_hold"


  @staticmethod
  def _interp_clipped(x: float, xp: list[float], fp: list[float]) -> float:
    if not xp or not fp or len(xp) != len(fp):
      return float(x)
    if x <= xp[0]:
      return float(fp[0])
    if x >= xp[-1]:
      return float(fp[-1])
    for i in range(1, len(xp)):
      if x <= xp[i]:
        x0 = float(xp[i - 1])
        x1 = float(xp[i])
        y0 = float(fp[i - 1])
        y1 = float(fp[i])
        if x1 <= x0:
          return float(y1)
        ratio = (float(x) - x0) / (x1 - x0)
        return float(y0 + ratio * (y1 - y0))
    return float(fp[-1])

  def _extract_lateral_limit_signal(self, controls_state) -> tuple[bool, float, str]:
    try:
      lateral = getattr(controls_state, "lateralControlState", None)
    except Exception:
      lateral = None
    if lateral is None:
      return False, 0.0, ""

    try:
      active_name = str(lateral.which() or "")
    except Exception:
      active_name = ""

    def _maybe_state(name: str):
      try:
        state = getattr(lateral, name)
      except Exception:
        return None
      if state is None:
        return None
      try:
        active = bool(getattr(state, "active", False))
      except Exception:
        active = False
      try:
        saturated = bool(getattr(state, "saturated", False))
      except Exception:
        saturated = False
      active_match = bool(active_name and (active_name == name))
      if not (active_match or active or saturated):
        return None
      return state

    state = _maybe_state("torqueState")
    if state is not None:
      desired_lat_accel = self._safe_finite_float(getattr(state, "desiredLateralAccel", 0.0), 0.0)
      actual_lat_accel = self._safe_finite_float(getattr(state, "actualLateralAccel", 0.0), 0.0)
      severity = max(0.0, abs(desired_lat_accel) - abs(actual_lat_accel))
      try:
        saturated = bool(getattr(state, "saturated", False))
      except Exception:
        saturated = False
      return saturated, float(severity), "torque"

    for name in ("angleState", "pidState"):
      state = _maybe_state(name)
      if state is None:
        continue
      desired_angle = self._safe_finite_float(getattr(state, "steeringAngleDesiredDeg", 0.0), 0.0)
      actual_angle = self._safe_finite_float(getattr(state, "steeringAngleDeg", 0.0), 0.0)
      severity = abs(desired_angle - actual_angle)
      try:
        saturated = bool(getattr(state, "saturated", False))
      except Exception:
        saturated = False
      return saturated, float(severity), name.replace("State", "")

    state = _maybe_state("debugState")
    if state is not None:
      try:
        saturated = bool(getattr(state, "saturated", False))
      except Exception:
        saturated = False
      return saturated, 1.0, "debug"

    return False, 0.0, ""

  def _apply_curve_limit_guard(
    self,
    *,
    now_ms: int,
    current_angle_deg: float,
    curve_target_ms: float,
    reference_ms: float,
    v_ego_ms: float,
    no_lead: bool,
  ) -> tuple[float, bool, str]:
    curve_target_ms = self._safe_finite_float(curve_target_ms, 0.0)
    reference_ms = self._safe_finite_float(reference_ms, curve_target_ms)
    v_ego_ms = self._safe_finite_float(v_ego_ms, 0.0)
    current_angle_deg = abs(self._safe_finite_float(current_angle_deg, 0.0))
    try:
      controls_fresh = self._controls_state_is_fresh(now_ns=int(now_ms) * 1_000_000)
    except Exception:
      controls_fresh = False
    real_limit_trigger = bool(controls_fresh and bool(self._lat_limit_saturated))
    fallback_trigger = bool((not controls_fresh) and current_angle_deg >= float(self._CURVE_LIMIT_GUARD_FALLBACK_STEER_DEG))
    eligible = (
      bool(no_lead)
      and v_ego_ms >= float(self._CURVE_LIMIT_GUARD_MIN_SPEED_MS)
      and curve_target_ms < (reference_ms - float(self._CURVE_LIMIT_GUARD_ACTIVATION_DROP_MS))
      and v_ego_ms > (curve_target_ms + float(self._CURVE_LIMIT_GUARD_VEGO_MARGIN_MS))
      and (real_limit_trigger or fallback_trigger)
    )

    if eligible:
      self._curve_limit_guard_release_candidate_since_ms = 0
      if not self._curve_limit_guard_active:
        if int(self._curve_limit_guard_candidate_since_ms) <= 0:
          self._curve_limit_guard_candidate_since_ms = int(now_ms)
        elif (int(now_ms) - int(self._curve_limit_guard_candidate_since_ms)) >= int(self._CURVE_LIMIT_GUARD_ENTRY_PERSIST_MS):
          self._curve_limit_guard_active = True
      else:
        self._curve_limit_guard_candidate_since_ms = int(now_ms)
    else:
      self._curve_limit_guard_candidate_since_ms = 0
      if self._curve_limit_guard_active:
        if int(self._curve_limit_guard_release_candidate_since_ms) <= 0:
          self._curve_limit_guard_release_candidate_since_ms = int(now_ms)
        elif (int(now_ms) - int(self._curve_limit_guard_release_candidate_since_ms)) >= int(self._CURVE_LIMIT_GUARD_RELEASE_PERSIST_MS):
          self._curve_limit_guard_active = False
          self._curve_limit_guard_release_candidate_since_ms = 0
      else:
        self._curve_limit_guard_release_candidate_since_ms = 0

    if not self._curve_limit_guard_active:
      return float(curve_target_ms), False, ""

    guard_reason = "steer_proxy"
    if real_limit_trigger:
      guard_reason = f"lat_sat[{self._lat_limit_source or 'unknown'}]"
      severity = float(self._lat_limit_severity)
      if str(self._lat_limit_source or "") == "torque":
        extra_drop_ms = self._interp_clipped(
          float(severity),
          [0.10, 0.35, 0.70, 1.20],
          [
            float(self._CURVE_LIMIT_GUARD_MIN_DROP_MS),
            3.0 * CV.MPH_TO_MS,
            4.5 * CV.MPH_TO_MS,
            float(self._CURVE_LIMIT_GUARD_MAX_DROP_MS),
          ],
        )
      elif str(self._lat_limit_source or "") in ("angle", "pid"):
        extra_drop_ms = self._interp_clipped(
          float(severity),
          [0.8, 2.0, 4.0, 6.0],
          [
            float(self._CURVE_LIMIT_GUARD_MIN_DROP_MS),
            3.0 * CV.MPH_TO_MS,
            4.5 * CV.MPH_TO_MS,
            float(self._CURVE_LIMIT_GUARD_MAX_DROP_MS),
          ],
        )
      else:
        extra_drop_ms = max(float(self._CURVE_LIMIT_GUARD_MIN_DROP_MS), 3.0 * CV.MPH_TO_MS)
    else:
      extra_drop_ms = self._interp_clipped(
        float(current_angle_deg),
        [
          float(self._CURVE_LIMIT_GUARD_FALLBACK_STEER_DEG),
          9.0,
          12.0,
        ],
        [
          float(self._CURVE_LIMIT_GUARD_MIN_DROP_MS),
          4.0 * CV.MPH_TO_MS,
          float(self._CURVE_LIMIT_GUARD_MAX_DROP_MS),
        ],
      )

    guarded_target_ms = max(float(self.MIN_CRUISE_SPEED_MS), float(curve_target_ms) - float(extra_drop_ms))
    return float(guarded_target_ms), True, str(guard_reason)

  def _poll_plan_and_lead(self, *, now_ns: int) -> None:
    prev_lead_present = bool(self._lead_present_prev)
    try:
      self._sm.update(0)
    except Exception:
      return

    try:
      lp = self._sm["longitudinalPlan"]
      v_last = self._extract_plan_speed_last(lp)
      v_near = self._extract_plan_speed_near_min(lp)
      lp_mono_ns = int(self._sm.logMonoTime.get("longitudinalPlan", 0) or 0)
      if (v_last is not None) and (lp_mono_ns > 0):
        self._lp_target_last_ms = float(v_last)
        self._lp_target_near_ms = float(v_near if v_near is not None else v_last)
        self._lp_last_ns = int(lp_mono_ns)
        self._lp_has_lead = bool(getattr(lp, "hasLead", False))
        self._lp_a_target = float(getattr(lp, "aTarget", 0.0) or 0.0)
        self._lp_source = str(getattr(lp, "longitudinalPlanSource", "") or "").lower()
    except Exception:
      pass

    try:
      self._lead_present = False
      self._lead_drel = 0.0
      self._lead_vrel = 0.0
      self._lead_yrel = 0.0
      if bool(self._sm.valid.get("radarState", False)):
        rs = self._sm["radarState"]
        lead_one = getattr(rs, "leadOne", None)
        if lead_one is not None and bool(getattr(lead_one, "status", False)):
          d_rel = float(getattr(lead_one, "dRel", 0.0) or 0.0)
          v_rel = float(getattr(lead_one, "vRel", 0.0) or 0.0)
          y_rel = float(getattr(lead_one, "yRel", 0.0) or 0.0)
          if d_rel > 0.0:
            self._lead_present = True
            self._lead_drel = d_rel
            self._lead_vrel = v_rel
            self._lead_yrel = y_rel
            self._lead_last_drel = d_rel
            self._lead_last_vrel = v_rel
            self._lead_last_yrel = y_rel
    except Exception:
      pass

    try:
      self._refresh_lateral_limit_state(now_ns=int(now_ns))
    except Exception:
      self._lat_limit_saturated = False
      self._lat_limit_severity = 0.0
      self._lat_limit_source = ""

    if prev_lead_present and (not bool(self._lead_present)):
      clear_grace_ms = int(self._LEAD_CLEAR_MAPD_GRACE_MS)
      if self._lead_last_sample_was_opening_clear():
        clear_grace_ms = min(clear_grace_ms, int(self._LEAD_CLEAR_OPENING_GRACE_MS))
      self._lead_recently_cleared_until_ms = int(now_ns // 1_000_000) + int(clear_grace_ms)
    self._lead_present_prev = bool(self._lead_present)

    try:
      if bool(self._sm.valid.get("mapdOut", False)):
        mo = self._sm["mapdOut"]
        suggested_ms = float(getattr(mo, "suggestedSpeed", 0.0) or 0.0)
        map_curve_ms = float(getattr(mo, "mapCurveSpeed", 0.0) or 0.0)
        vision_curve_ms = float(getattr(mo, "visionCurveSpeed", 0.0) or 0.0)
        mono_ns = int(self._sm.logMonoTime.get("mapdOut", 0) or 0)
        if mono_ns > 0:
          # Fresh mapd packets must overwrite the cached values even when the
          # current fields are zero. Otherwise a stale earlier curve speed can
          # survive into clear-road motorway segments and keep reopening
          # curve_hold[mapd] after mapd has already stopped publishing a curve.
          self._mapd_suggested_ms = suggested_ms if (math.isfinite(suggested_ms) and suggested_ms > 0.1) else None
          self._mapd_map_curve_ms = map_curve_ms if (math.isfinite(map_curve_ms) and map_curve_ms > 0.1) else None
          self._mapd_vision_curve_ms = vision_curve_ms if (math.isfinite(vision_curve_ms) and vision_curve_ms > 0.1) else None
          self._mapd_last_ns = mono_ns
    except Exception:
      pass

  def _resolve_speed_limit_target_ms(self, CS, *, speed_units: str) -> tuple[Optional[float], bool, str]:
    """
    Unity-style speed-limit ownership for ACC.

    LONG/ACC own whether a positive CarState target is available. If the optional
    _tinkla wrapper exists, a false config disables it; otherwise a positive
    CarState target is sufficient.
    """
    use_speed_limit = True
    tinkla = getattr(CS, "_tinkla", None)
    if tinkla is not None:
      try:
        use_speed_limit = bool(getattr(tinkla, "adjust_acc_with_speed_limit"))
      except Exception:
        use_speed_limit = True
    if not use_speed_limit:
      return None, False, "config_disabled"

    speed_limit_target_ms = 0.0
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0

    if speed_limit_target_ms > 0.0:
      return float(speed_limit_target_ms), True, "carstate_speed_limit_target"
    return None, False, "none"

  def _roadworks_cap_ms(self) -> Optional[float]:
    try:
      with open(self._ROADWORKS_CAP_FILE, "r", encoding="utf-8") as f:
        raw = str(f.read()).strip()
    except OSError:
      return None

    if not raw:
      return None

    try:
      kph = float(raw)
    except Exception:
      return None

    if (not math.isfinite(kph)) or kph <= 0.1:
      return None

    return float(kph) * CV.KPH_TO_MS


  def _lead_is_constraining(self, *, base_target_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return False
    if self._lead_is_opening_clear(base_target_ms=float(base_target_ms), v_ego_ms=float(v_ego_ms)):
      return False

    lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
    lead_slower_than_base = lead_speed_ms < (float(base_target_ms) - 0.25)
    closing = float(self._lead_vrel) < float(self._LEAD_CONSTRAIN_CLOSING_VREL_MS)
    near_gap_limit_m = min(80.0, max(float(self._LEAD_CONSTRAIN_GAP_MIN_M), float(v_ego_ms) * float(self._LEAD_CONSTRAIN_TIME_GAP_S)))
    near_lead = float(self._lead_drel) < float(near_gap_limit_m)
    non_opening_near = near_lead and float(self._lead_vrel) <= float(self._LEAD_OPENING_VREL_MS)
    return bool(lead_slower_than_base and (closing or non_opening_near))


  def _lead_follow_hold_needed(self, *, base_target_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return False
    if self._lead_is_opening_clear(base_target_ms=float(base_target_ms), v_ego_ms=float(v_ego_ms)):
      return False
    if self._lead_is_constraining(base_target_ms=float(base_target_ms), v_ego_ms=float(v_ego_ms)):
      return True

    close_gap_limit_m = min(
      34.0,
      max(16.0, float(v_ego_ms) * 1.15),
    )
    return bool(
      float(self._lead_drel) < float(close_gap_limit_m)
      and float(self._lead_vrel) <= 0.10
    )


  def _apply_lead_nibble_hold(
    self,
    *,
    desired_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    base_target_ms: float,
  ) -> tuple[float, bool]:
    desired_ms = float(desired_ms)
    current_set_ms = float(current_set_ms)
    v_ego_ms = float(v_ego_ms)
    base_target_ms = float(base_target_ms)

    if not self._lead_follow_hold_needed(base_target_ms=base_target_ms, v_ego_ms=v_ego_ms):
      return desired_ms, False
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return desired_ms, False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return desired_ms, False
    if self._lead_is_opening_clear(base_target_ms=base_target_ms, v_ego_ms=v_ego_ms):
      return desired_ms, False
    if float(self._lead_vrel) <= float(self._LEAD_NIBBLE_HOLD_MIN_VREL_MS):
      return desired_ms, False
    if float(self._lead_drel) < float(self._LEAD_NIBBLE_HOLD_MIN_DREL_M):
      return desired_ms, False
    if float(self._lp_a_target) <= float(self._LEAD_NIBBLE_HOLD_MAX_ATARGET_MS2):
      return desired_ms, False

    hold_ref_ms = min(base_target_ms, max(current_set_ms, v_ego_ms))
    drop_ms = float(hold_ref_ms) - float(desired_ms)
    if drop_ms <= 0.0:
      return desired_ms, False
    if drop_ms > float(self._LEAD_NIBBLE_HOLD_MAX_DROP_MS):
      return desired_ms, False

    return float(hold_ref_ms), True



  def _lp_queue_fallback_active(self, *, now_ms: int, base_target_ms: float, planner_ms: float, v_ego_ms: float) -> bool:
    if bool(self._lead_present):
      return False
    if not bool(self._lp_has_lead):
      return False
    if int(now_ms) > int(self._lead_recently_cleared_until_ms):
      return False
    if float(v_ego_ms) > float(self._LP_QUEUE_FALLBACK_MAX_SPEED_MS):
      return False
    if float(planner_ms) <= 0.1:
      return False

    materially_below_base = float(planner_ms) < (float(base_target_ms) - float(self._LP_QUEUE_FALLBACK_DROP_MS))
    materially_below_ego = float(planner_ms) < (float(v_ego_ms) - float(self._LP_QUEUE_FALLBACK_EGO_MARGIN_MS))
    planner_decel = float(self._lp_a_target) <= float(self._LP_QUEUE_FALLBACK_ATARGET_MS2)
    opening_clear_recent = bool(
      self._lead_last_sample_was_opening_clear()
      and int(now_ms) <= int(self._lead_recently_cleared_until_ms)
    )
    if opening_clear_recent and str(self._lp_source or "") in ("cruise", "e2e"):
      return False
    return bool(materially_below_base and materially_below_ego and planner_decel)

  def _planner_drag_reasons(self, *, now_ms: int, base_target_ms: float, planner_ms: float, current_set_ms: float, v_ego_ms: float) -> list[str]:
    if float(planner_ms) <= 0.1:
      return []

    materially_below_base = float(planner_ms) < (float(base_target_ms) - float(self._PLANNER_DRAG_MARGIN_MS))
    materially_below_ego = float(planner_ms) < (float(v_ego_ms) - float(self._PLANNER_BELOW_EGO_MARGIN_MS))
    materially_below_clear = materially_below_base and materially_below_ego
    lead_constraining = self._lead_is_constraining(
      base_target_ms=float(base_target_ms),
      v_ego_ms=float(v_ego_ms),
    )
    queue_fallback_active = self._lp_queue_fallback_active(
      now_ms=int(now_ms),
      base_target_ms=float(base_target_ms),
      planner_ms=float(planner_ms),
      v_ego_ms=float(v_ego_ms),
    )

    planner_tracks_set = (
      float(current_set_ms) > 0.1
      and float(planner_ms) >= (float(current_set_ms) - float(self._PLANNER_SET_TRACK_MARGIN_MS))
    )
    weak_lead_owner = False
    if bool(self._lead_present) and float(self._lead_drel) > 0.0:
      weak_gap_limit_m = min(
        90.0,
        max(float(self._LEAD_CONSTRAIN_GAP_MIN_M), float(v_ego_ms) * float(self._WEAK_LEAD_OWNER_TIME_GAP_S)),
      )
      weak_lead_owner = bool(
        float(self._lead_drel) >= float(weak_gap_limit_m)
        and float(self._lead_vrel) >= float(self._WEAK_LEAD_OWNER_VREL_MS)
        and float(self._lp_a_target) > float(self._WEAK_LEAD_OWNER_ATARGET_MS2)
      )

    if planner_tracks_set and ((lead_constraining and weak_lead_owner) or queue_fallback_active):
      return []

    reasons: list[str] = []
    if lead_constraining:
      reasons.append("lead")
    if queue_fallback_active:
      reasons.append("lp_hasLead")
    if float(self._lp_a_target) <= float(self._STRONG_DECEL_ATARGET_MS2) and (lead_constraining or queue_fallback_active):
      reasons.append("aTarget")
    if materially_below_clear and (lead_constraining or queue_fallback_active):
      reasons.append("planner_low")
    return reasons

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    controller_enabled = bool(enabled) and bool(getattr(CS, "enable_adaptive_cruise", False) or getattr(CS, "enableACC", False))
    if not controller_enabled:
      self._last_active = False
      self._enabled_since_ms = 0
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._reset_curve_hold()
      self._reset_lead_hold()
      self._curve_limit_guard_active = False
      self._curve_limit_guard_candidate_since_ms = 0
      self._curve_limit_guard_release_candidate_since_ms = 0
      self._curve_limit_guard_active = False
      self._curve_limit_guard_candidate_since_ms = 0
      self._curve_limit_guard_release_candidate_since_ms = 0
      return LongDecision(None, "gated: not enabled/adaptive")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL", "STANDBY"):
      self._last_active = False
      self._reset_curve_hold()
      self._reset_lead_hold()
      self._curve_limit_guard_active = False
      self._curve_limit_guard_candidate_since_ms = 0
      self._curve_limit_guard_release_candidate_since_ms = 0
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not self._last_active:
      self._enabled_since_ms = int(now)
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._reset_lead_hold()
    self._last_active = True

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    self._poll_plan_and_lead(now_ns=now_ns)
    lp_fresh = (
      (self._lp_target_last_ms is not None)
      and (int(self._lp_last_ns) > 0)
      and ((now_ns - int(self._lp_last_ns)) < int(self._LP_FRESH_NS))
    )

    if lp_fresh and int(self._lp_last_ns) != int(self._last_lp_seen_ns):
      self._stable_plan_samples = min(int(self._stable_plan_samples) + 1, 1000)
      self._last_lp_seen_ns = int(self._lp_last_ns)
    elif not lp_fresh:
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._lp_has_lead = False
      self._lp_a_target = 0.0
      self._lp_source = ""
      self._lp_target_last_ms = None
      self._lp_target_near_ms = None
      self._reset_curve_hold()
      self._reset_lead_hold()

    planner_last_ms = float(self._lp_target_last_ms) if (lp_fresh and self._lp_target_last_ms is not None) else float(current_set_ms)
    planner_near_ms = float(self._lp_target_near_ms) if (lp_fresh and self._lp_target_near_ms is not None) else float(planner_last_ms)
    desired_ms = float(planner_last_ms)
    src = "lp_last" if lp_fresh else "hold"

    startup_warmup = bool(self._enabled_since_ms and ((int(now) - int(self._enabled_since_ms)) < 1800))
    startup_invalid_clear = (
      startup_warmup
      and (not self._lead_present)
      and (
        (not lp_fresh)
        or (int(self._stable_plan_samples) < 2)
        or (float(planner_last_ms) <= 0.1)
        or (
          float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS)
          and float(planner_last_ms) < max(float(self.MIN_CRUISE_SPEED_MS) * 0.90, float(current_set_ms) - (4.0 * CV.KPH_TO_MS))
        )
      )
    )

    speed_limit_target_ms, set_speed_limit_active, ceiling_src = self._resolve_speed_limit_target_ms(CS, speed_units=speed_units)
    roadworks_cap_ms = self._roadworks_cap_ms()

    if set_speed_limit_active and speed_limit_target_ms is not None:
      base_target_ms = float(speed_limit_target_ms)
      if roadworks_cap_ms is not None:
        base_target_ms = min(float(base_target_ms), float(roadworks_cap_ms))
        ceiling_src = f"{ceiling_src}+roadworks_cap"
      desired_ms = float(base_target_ms)
      src = f"speed_limit_target[{ceiling_src}]"

      if lp_fresh and self._lp_target_last_ms is not None:
        suppress_planner_lead_owner = (
          self._lead_present
          and self._lead_is_opening_clear(
            base_target_ms=float(base_target_ms),
            v_ego_ms=float(v_ego_ms),
          )
          and str(self._lp_source or "") in ("cruise", "e2e")
        )
        drag_reasons = [] if suppress_planner_lead_owner else self._planner_drag_reasons(
          now_ms=int(now),
          base_target_ms=float(base_target_ms),
          planner_ms=float(planner_last_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lp_hasLead" in drag_reasons)
        if lead_owned:
          desired_ms = min(float(base_target_ms), float(planner_last_ms))
          src = f"{src}+planner[{'+'.join(drag_reasons)}]"
          self._lead_hold_until_ms = int(now) + int(self._LEAD_HOLD_PERSIST_MS)
          self._reset_curve_hold()
        elif self._lead_hold_active_now(
          now_ms=int(now),
          base_target_ms=float(base_target_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          desired_ms = min(float(base_target_ms), float(planner_last_ms))
          src = f"{src}+planner[lead_hold]"
          self._reset_curve_hold()
        elif (not self._lead_present) and (not self._lp_has_lead):
          self._reset_lead_hold()
          reference_ms = self._reference_speed_ms(
            base_target_ms=float(base_target_ms),
            current_set_ms=float(current_set_ms),
            v_ego_ms=float(v_ego_ms),
          )

          if self._should_force_curve_release(
            now_ms=int(now),
            now_ns=now_ns,
            reference_ms=float(reference_ms),
            planner_last_ms=float(planner_last_ms),
            planner_near_ms=float(planner_near_ms),
          ):
            self._reset_curve_hold()
            self._curve_recent_clear_until_ms = int(now) + int(self._CURVE_REENTRY_BLOCK_MS)
            desired_ms = float(base_target_ms)
            src = f"{src}+curve_clear(planner)"
          else:
            mapd_target_ms = self._mapd_curve_active_target_ms(now_ns=now_ns)
            gate_ms = float(self._NO_LEAD_MAPD_CURRENT_GATE_MS)
            if (
              mapd_target_ms is not None
              and float(mapd_target_ms) < (float(reference_ms) - float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS))
              and float(mapd_target_ms) < (float(v_ego_ms) - gate_ms)
            ):
              curve_target_ms, curve_state = self._stabilize_no_lead_curve_target(
                now_ms=int(now),
                raw_target_ms=float(mapd_target_ms),
                reference_ms=float(reference_ms),
              )
              current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
              curve_target_ms, curve_limit_guard, curve_limit_guard_reason = self._apply_curve_limit_guard(
                now_ms=int(now),
                current_angle_deg=float(current_angle_deg),
                curve_target_ms=float(curve_target_ms),
                reference_ms=float(reference_ms),
                v_ego_ms=float(v_ego_ms),
                no_lead=True,
              )
              if curve_limit_guard:
                guard_suffix = "curve_limit_guard"
                if curve_limit_guard_reason:
                  guard_suffix = f"{guard_suffix}[{curve_limit_guard_reason}]"
                curve_state = f"{curve_state}+{guard_suffix}"
              if float(curve_target_ms) < float(base_target_ms):
                desired_ms = min(float(base_target_ms), float(curve_target_ms))
                src = f"{src}+{curve_state}[mapd]"
            else:
              self._reset_curve_hold()
        else:
          self._reset_curve_hold()
          self._reset_lead_hold()

        if self._lead_follow_hold_needed(
          base_target_ms=float(base_target_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          lead_hold_cap_ms = max(float(current_set_ms), float(v_ego_ms))
          if float(desired_ms) > (float(lead_hold_cap_ms) + (0.25 * CV.MPH_TO_MS)):
            desired_ms = float(lead_hold_cap_ms)
            src = f"{src}+lead_present_hold"


        desired_ms, lead_nibble_held = self._apply_lead_nibble_hold(
          desired_ms=float(desired_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
          base_target_ms=float(base_target_ms),
        )
        if lead_nibble_held:
          src = f"{src}+lead_nibble_hold"
      elif self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(base_target_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"
      else:
        self._reset_curve_hold()
    else:
      resume_ceiling_ms = self._resume_ceiling_ms(current_set_ms=float(current_set_ms), v_ego_ms=float(v_ego_ms))
      if roadworks_cap_ms is not None:
        resume_ceiling_ms = min(float(resume_ceiling_ms), float(roadworks_cap_ms))
      desired_ms = float(resume_ceiling_ms)
      src = "hold+ceiling"
      if roadworks_cap_ms is not None:
        src = f"{src}+roadworks_cap"

      if lp_fresh:
        suppress_planner_lead_owner = (
          self._lead_present
          and self._lead_is_opening_clear(
            base_target_ms=float(resume_ceiling_ms),
            v_ego_ms=float(v_ego_ms),
          )
          and str(self._lp_source or "") in ("cruise", "e2e")
        )
        drag_reasons = [] if suppress_planner_lead_owner else self._planner_drag_reasons(
          now_ms=int(now),
          base_target_ms=float(resume_ceiling_ms),
          planner_ms=float(planner_last_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lp_hasLead" in drag_reasons)
        if lead_owned:
          desired_ms = float(planner_last_ms)
          src = "lp_last"
          self._lead_hold_until_ms = int(now) + int(self._LEAD_HOLD_PERSIST_MS)
          self._reset_curve_hold()
        elif self._lead_hold_active_now(
          now_ms=int(now),
          base_target_ms=float(resume_ceiling_ms),
          planner_ms=float(planner_last_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          desired_ms = float(planner_last_ms)
          src = "lp_last[lead_hold]"
          self._reset_curve_hold()
        elif (not self._lead_present) and (not self._lp_has_lead):
          self._reset_lead_hold()
          current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
          planner_curve_margin_ms = max(
            float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS),
            float(self._PLANNER_CURVE_ENTRY_MARGIN_MS),
          )
          planner_curve_active = float(planner_near_ms) < (
            float(resume_ceiling_ms) - float(planner_curve_margin_ms)
          )

          if (
            int(now) <= int(self._curve_recent_clear_until_ms)
            and current_angle_deg <= float(self._PLANNER_ONLY_REENTRY_ALLOW_STEER_DEG)
            and float(planner_near_ms) >= (float(resume_ceiling_ms) - float(self._PLANNER_ONLY_REENTRY_BLOCK_DROP_MS))
          ):
            planner_curve_active = False

          curve_specific_mapd_ms = self._mapd_entry_target_ms(
            now_ms=int(now),
            now_ns=now_ns,
            reference_ms=float(resume_ceiling_ms),
            planner_near_ms=float(planner_near_ms),
            v_ego_ms=float(v_ego_ms),
            current_angle_deg=float(current_angle_deg),
            planner_curve_active=bool(planner_curve_active),
          )

          curve_candidates_ms: list[float] = []
          curve_owner_parts: list[str] = []

          if planner_curve_active:
            curve_candidates_ms.append(float(planner_near_ms))
            curve_owner_parts.append("planner")

          if curve_specific_mapd_ms is not None:
            curve_candidates_ms.append(float(curve_specific_mapd_ms))
            curve_owner_parts.append("mapd")

          if curve_candidates_ms:
            raw_curve_target_ms = float(min(curve_candidates_ms))
          else:
            raw_curve_target_ms = float(planner_near_ms)

          curve_target_ms, curve_state = self._stabilize_no_lead_curve_target(
            now_ms=int(now),
            raw_target_ms=float(raw_curve_target_ms),
            reference_ms=float(resume_ceiling_ms),
          )

          if self._should_force_curve_release(
            now_ms=int(now),
            now_ns=now_ns,
            reference_ms=float(resume_ceiling_ms),
            planner_last_ms=float(planner_last_ms),
            planner_near_ms=float(planner_near_ms),
          ):
            self._reset_curve_hold()
            self._curve_recent_clear_until_ms = int(now) + int(self._CURVE_REENTRY_BLOCK_MS)
            curve_target_ms = float(resume_ceiling_ms)
            curve_state = "curve_clear(planner)"

          if curve_owner_parts:
            curve_state = f"{curve_state}|{'+'.join(curve_owner_parts)}"
          if float(curve_target_ms) < float(self.MIN_CRUISE_SPEED_MS):
            hold_floor_ms = float(self.MIN_CRUISE_SPEED_MS) - float(self._CURVE_MIN_CRUISE_HOLD_MARGIN_MS)
            if float(curve_target_ms) >= float(hold_floor_ms):
              curve_target_ms = float(self.MIN_CRUISE_SPEED_MS)
              curve_state = f"{curve_state}+min_hold"

          near_resume_tolerance_ms = max(
            float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS),
            5.5 * CV.MPH_TO_MS,
          )
          planner_near_resume_clear = float(planner_near_ms) >= (
            float(resume_ceiling_ms) - max(float(self._CURVE_RELEASE_NEAR_TARGET_MARGIN_MS), 0.8 * CV.MPH_TO_MS)
          )
          straightish_exit = float(current_angle_deg) <= float(self._CURVE_REENTRY_ALLOW_STEER_DEG)
          if float(curve_target_ms) >= (float(resume_ceiling_ms) - float(near_resume_tolerance_ms)) and (
            planner_near_resume_clear or curve_specific_mapd_ms is None or straightish_exit
          ):
            self._reset_curve_hold()
            self._curve_recent_clear_until_ms = int(now) + int(self._CURVE_REENTRY_BLOCK_MS)
            curve_target_ms = float(resume_ceiling_ms)
            curve_state = "curve_clear(snap)"

          curve_target_ms, curve_limit_guard, curve_limit_guard_reason = self._apply_curve_limit_guard(
            now_ms=int(now),
            current_angle_deg=float(current_angle_deg),
            curve_target_ms=float(curve_target_ms),
            reference_ms=float(resume_ceiling_ms),
            v_ego_ms=float(v_ego_ms),
            no_lead=True,
          )
          if curve_limit_guard:
            guard_suffix = "curve_limit_guard"
            if curve_limit_guard_reason:
              guard_suffix = f"{guard_suffix}[{curve_limit_guard_reason}]"
            curve_state = f"{curve_state}+{guard_suffix}"

          desired_ms = min(float(resume_ceiling_ms), float(curve_target_ms))
          src = f"lp_near[{curve_state}]"
        else:
          self._reset_curve_hold()
          self._reset_lead_hold()
          if (
            self._lead_present
            and self._lead_is_opening_clear(
              base_target_ms=float(resume_ceiling_ms),
              v_ego_ms=float(v_ego_ms),
            )
          ):
            desired_ms = float(resume_ceiling_ms)
            src = "hold+ceiling+lead_opening"
          else:
            desired_ms = float(current_set_ms if current_set_ms > 0.1 else resume_ceiling_ms)
            src = "hold+current_set+lead_present"

        if self._lead_present and self._lead_is_constraining(
          base_target_ms=float(resume_ceiling_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          constrain_target_ms = float(planner_last_ms)
          if lp_fresh and self._lp_target_near_ms is not None:
            constrain_target_ms = min(float(constrain_target_ms), float(self._lp_target_near_ms))
          constrain_target_ms = min(float(constrain_target_ms), float(current_set_ms if current_set_ms > 0.1 else resume_ceiling_ms))
          if float(constrain_target_ms) < float(desired_ms):
            desired_ms = float(constrain_target_ms)
            src = f"{src}+lead_constrain"

        if self._lead_follow_hold_needed(
          base_target_ms=float(resume_ceiling_ms),
          v_ego_ms=float(v_ego_ms),
        ):
          lead_hold_cap_ms = max(float(current_set_ms), float(v_ego_ms))
          if float(desired_ms) > (float(lead_hold_cap_ms) + (0.25 * CV.MPH_TO_MS)):
            desired_ms = float(lead_hold_cap_ms)
            src = f"{src}+lead_present_hold"


        desired_ms, lead_nibble_held = self._apply_lead_nibble_hold(
          desired_ms=float(desired_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
          base_target_ms=float(resume_ceiling_ms),
        )
        if lead_nibble_held:
          src = f"{src}+lead_nibble_hold"
      else:
        self._reset_curve_hold()
        if startup_invalid_clear:
          desired_ms = float(current_set_ms)
          src = "hold+startup_hold"
        else:
          desired_ms = float(resume_ceiling_ms)
          src = "hold+ceiling"

      if (not lp_fresh) and self._lead_present and (self._lead_drel < 80.0) and (self._lead_vrel < -0.5):
        lead_speed_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel))
        desired_ms = min(float(desired_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(lead_speed_ms)))
        src = f"{src}+stale_lead"

    no_lead_curve_context = (not self._lead_present) and (not self._lp_has_lead)
    if not no_lead_curve_context:
      self._curve_limit_guard_active = False
      self._curve_limit_guard_candidate_since_ms = 0
      self._curve_limit_guard_release_candidate_since_ms = 0
    if self._should_hold_min_cruise_for_curve(
      now_ns=now_ns,
      desired_ms=float(desired_ms),
      no_lead=bool(no_lead_curve_context),
    ):
      desired_ms = max(float(desired_ms), float(self.MIN_CRUISE_SPEED_MS))
      if "min_hold" not in src:
        src = f"{src}+min_hold"

    stock_cruise_enabled = stock_state in ("ENABLED", "OVERRIDE", "STANDSTILL")
    brake_pressed = bool(getattr(cs_out, "brakePressed", False))

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=True,
      stock_cruise_enabled=stock_cruise_enabled,
      stock_cruise_state=stock_state,
      speed_units=speed_units,
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_ms,
      desired_speed_ms=float(desired_ms),
      cruise_buttons=cruise_buttons,
      brake_pressed=brake_pressed,
      speed_limit_target_ms=speed_limit_target_ms,
      set_speed_limit_active=bool(set_speed_limit_active),
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, f"{decision.reason} src={src}")

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] src={src} uom={speed_units} "
      f"tgt={decision.target_kph * kph_to_u:.1f} cur={decision.current_kph * kph_to_u:.1f} "
      f"est={decision.est_kph * kph_to_u:.1f} btn={int(decision.button)} reason={decision.reason}"
    )
    self._rate_log(msg)
    return LongDecision(int(decision.button), msg)
