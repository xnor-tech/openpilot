# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""
Human-tuned stock cruise syncing for XNOR.

This keeps the stable owner split and the no-lead curve behavior, while
making lead-follow recovery smoother and less sticky:

v61 adds an explicit final LONG arbitration state machine.

v62 tunes follow-gap-aware lead release, curve comfort bias, and stale clear-road recovery.

- lead critical / lead follow / curve pre-entry / curve active / curve exit / cruise sync are resolved in one final pass
- stale lead/planner ownership is expired before it can hold speed down after curves
- map-only curve caps are ignored when vision/steering/planner do not confirm the bend
- curve exit releases cleanly so speed can resume after roundabouts

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
from cereal.services import SERVICE_LIST
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
  _LEAD_PLANNER_GUARD_MIN_DROP_MS = 1.0 * CV.MPH_TO_MS
  _LEAD_PLANNER_GUARD_EGO_MARGIN_MS = 0.35 * CV.MPH_TO_MS
  _LEAD_PLANNER_GUARD_TIME_GAP_S = 2.05
  _LEAD_PLANNER_GUARD_MIN_GAP_M = 38.0
  _LEAD_PLANNER_GUARD_MAX_GAP_M = 72.0

  _CURVE_ENTRY_PERSIST_MS = 240
  _CURVE_EXIT_PERSIST_MS = 420
  _CURVE_EXIT_RECOVERY_MS_PER_S = 3.2
  _CURVE_HOLD_ENTRY_FREEZE_MS = 900
  _CURVE_HOLD_DROP_PERSIST_MS = 260
  _CURVE_HOLD_DROP_RATE_MS_PER_S = 1.8
  _CURVE_HOLD_HARD_DROP_EXTRA_MS = 5.0 * CV.MPH_TO_MS
  _CURVE_ENTRY_PREVIEW_DROP_MS = 2.0 * CV.MPH_TO_MS
  _CURVE_PRE_ENTRY_MIN_DROP_MS = 4.0 * CV.MPH_TO_MS
  _CURVE_PRE_ENTRY_PREVIEW_DROP_MS = 5.0 * CV.MPH_TO_MS
  _CURVE_PRE_ENTRY_MIN_RATIO = 0.42
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
  _MAPD_DISAGREE_COMFORT_BLEND = 0.90
  _MAPD_DISAGREE_PLANNER_CONFIRM_MS = 2.0 * CV.MPH_TO_MS
  _MAPD_DISAGREE_STEER_CONFIRM_DEG = 3.5
  _MAPD_ONLY_HIGHWAY_SPEED_MS = 55.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_DROP_MS = 24.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_PLANNER_DELTA_MS = 16.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_PLANNER_SUPPORT_DROP_MS = 4.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_MAX_EXTRA_DROP_WITH_PLANNER_MS = 12.0 * CV.MPH_TO_MS
  _MAPD_ONLY_HIGHWAY_NEAR_STEER_DEG = 2.5
  _LEAD_CLEAR_MAPD_GRACE_MS = 520
  _LEAD_CLEAR_OPENING_GRACE_MS = 220
  _LEAD_CURVE_HOLD_MAX_MS = 180
  _LEAD_CURVE_HOLD_MAX_GAP_M = 65.0
  _LEAD_CURVE_HOLD_MAX_YREL_M = 2.8
  _LEAD_CURVE_HOLD_MIN_SPEED_MS = 6.0
  _LEAD_CURVE_HOLD_STEER_DEG = 6.0
  _LEAD_CURVE_HOLD_STEER_RATE_DEG = 12.0
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
  _CURVE_LIMIT_GUARD_MIN_DROP_MS = 0.75 * CV.MPH_TO_MS
  _CURVE_LIMIT_GUARD_MAX_DROP_MS = 2.75 * CV.MPH_TO_MS
  _CONTROLS_STATE_FRESH_NS = 500_000_000
  _CURVE_LIMIT_GUARD_FALLBACK_STEER_DEG = 7.5
  _MAPD_LOW_SPEED_ENTRY_SPEED_MS = 45.0 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_SHARP_DROP_MS = 3.0 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_PLANNER_HINT_DROP_MS = 0.4 * CV.MPH_TO_MS
  _MAPD_LOW_SPEED_STEER_HINT_DEG = 0.35
  _SHARP_CURVE_FAST_ENTRY_DROP_MS = 6.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_MIN_SPEED_MS = 18.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_MAX_SPEED_MS = 52.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_MIN_DROP_MS = 8.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_EGO_DROP_MS = 2.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_PERSIST_MS = 480
  _LEAD_CONTEXT_CURVE_CAP_PREVIEW_DROP_MS = 4.0 * CV.MPH_TO_MS
  _LEAD_CONTEXT_CURVE_CAP_TARGET_OFFSET_MS = 5.0 * CV.MPH_TO_MS
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
  _STALE_PLANNER_LEAD_CLEAR_MS = 900
  _STALE_PLANNER_LEAD_MAX_ATARGET_MS2 = -0.20
  _STALE_PLANNER_LEAD_MIN_DROP_MS = 2.0 * CV.MPH_TO_MS
  _STALE_PLANNER_LEAD_MAX_LIVE_DROP_MS = 0.75 * CV.MPH_TO_MS

  _CURVE_FORCE_ENTRY_MIN_SPEED_MS = 18.0 * CV.MPH_TO_MS
  _CURVE_FORCE_ENTRY_MIN_DROP_MS = 6.0 * CV.MPH_TO_MS
  _CURVE_FORCE_ENTRY_EGO_DROP_MS = 2.0 * CV.MPH_TO_MS
  _CURVE_FORCE_ENTRY_PERSIST_MS = 420
  _CURVE_FORCE_ENTRY_STEER_PERSIST_MS = 180
  _CURVE_FORCE_ENTRY_PREVIEW_DROP_MS = 3.5 * CV.MPH_TO_MS
  _CURVE_FORCE_ENTRY_TARGET_OFFSET_MS = 4.0 * CV.MPH_TO_MS
  _CURVE_ACCEL_BLOCK_STEER_DEG = 4.8
  _CURVE_ACCEL_BLOCK_STEER_RATE_DEG = 13.0
  _CURVE_ACCEL_BLOCK_MIN_DROP_MS = 4.0 * CV.MPH_TO_MS
  _CURVE_ACCEL_BLOCK_EGO_MARGIN_MS = 2.0 * CV.MPH_TO_MS
  _CURVE_STEER_LIMIT_HOLD_MS = 260
  _CURVE_STEER_LIMIT_HOLD_DROP_MS = 0.75 * CV.MPH_TO_MS

  _MAPD_STRAIGHT_STALE_DISAGREE_MS = 8.0 * CV.MPH_TO_MS
  _MAPD_STRAIGHT_STALE_VISION_CLEAR_MS = 6.0 * CV.MPH_TO_MS
  _MAPD_STRAIGHT_STALE_PLANNER_CLEAR_MS = 2.0 * CV.MPH_TO_MS
  _MAPD_STRAIGHT_STALE_STEER_DEG = 1.2
  _MAPD_STRAIGHT_STALE_SET_DROP_MS = 8.0 * CV.MPH_TO_MS
  _MAPD_STRAIGHT_STALE_BLOCK_MS = 8000
  _MAPD_MAP_ONLY_STRONG_DISAGREE_MS = 12.0 * CV.MPH_TO_MS
  _MAPD_MAP_ONLY_COMFORT_EXTRA_MS = 8.0 * CV.MPH_TO_MS
  _MAPD_MAP_ONLY_COMFORT_FLOOR_MS = 27.0 * CV.MPH_TO_MS
  _MAPD_MAP_ONLY_LOW_CONF_STEER_DEG = 24.0


  _LEAD_STUCK_CANCEL_MIN_DROP_MS = 4.0 * CV.MPH_TO_MS
  _LEAD_STUCK_CANCEL_PLANNER_MARGIN_MS = 2.0 * CV.MPH_TO_MS
  _LEAD_STUCK_CANCEL_SET_DROP_EPS_MS = 0.4 * CV.MPH_TO_MS
  _LEAD_STUCK_CANCEL_MIN_CLOSING_MS = -1.0
  _LEAD_STUCK_CANCEL_TIME_GAP_S = 1.45
  _LEAD_STUCK_CANCEL_MIN_GAP_M = 28.0
  _LEAD_STUCK_CANCEL_MAX_GAP_M = 48.0
  _LEAD_STUCK_CANCEL_MIN_ACTIVE_MS = 900
  _LEAD_STUCK_CANCEL_NO_DROP_MS = 650
  _LEAD_STUCK_CANCEL_REPEAT_MS = 1800

  _LEAD_APPROACH_CANCEL_MIN_SPEED_MS = 18.0 * CV.MPH_TO_MS
  _LEAD_APPROACH_CANCEL_MAX_SPEED_MS = 58.0 * CV.MPH_TO_MS
  _LEAD_APPROACH_CANCEL_MIN_DROP_MS = 4.0 * CV.MPH_TO_MS
  _LEAD_APPROACH_CANCEL_PLANNER_MARGIN_MS = 2.0 * CV.MPH_TO_MS
  _LEAD_APPROACH_CANCEL_SET_DROP_EPS_MS = 0.4 * CV.MPH_TO_MS
  _LEAD_APPROACH_CANCEL_CLOSING_MS = -0.50
  _LEAD_APPROACH_CANCEL_OPENING_RESET_MS = 0.70
  _LEAD_APPROACH_CANCEL_ATARGET_MS2 = -0.75
  _LEAD_APPROACH_CANCEL_STRONG_ATARGET_MS2 = -1.25
  _LEAD_APPROACH_CANCEL_TIME_GAP_S = 1.60
  _LEAD_APPROACH_CANCEL_MIN_GAP_M = 10.0
  _LEAD_APPROACH_CANCEL_MAX_GAP_M = 34.0
  _LEAD_APPROACH_CANCEL_CRITICAL_GAP_M = 18.0
  _LEAD_APPROACH_CANCEL_MIN_ACTIVE_MS = 650
  _LEAD_APPROACH_CANCEL_NO_DROP_MS = 500
  _LEAD_APPROACH_CANCEL_REPEAT_MS = 1800
  _LEAD_APPROACH_FORCE_CANCEL_MIN_DROP_MS = 8.0 * CV.MPH_TO_MS
  _LEAD_APPROACH_FORCE_CANCEL_MAX_GAP_M = 72.0
  _LEAD_APPROACH_FORCE_CANCEL_TIME_GAP_S = 2.50
  _LEAD_APPROACH_FORCE_CANCEL_CLOSING_MS = -0.10
  _LEAD_APPROACH_FORCE_CANCEL_ATARGET_MS2 = -0.45
  _LEAD_APPROACH_FORCE_CANCEL_ARM_MS = 160
  _LEAD_APPROACH_FORCE_CANCEL_REPEAT_MS = 1600

  _SECONDARY_LEAD_FALLBACK_MAX_YREL_M = 1.8
  _SECONDARY_LEAD_FALLBACK_MAX_GAP_M = 70.0
  _SECONDARY_LEAD_FALLBACK_LAST_DREL_DELTA_M = 18.0

  _LOW_SPEED_LEAD_BLOCK_MAX_SPEED_MS = 35.0 * CV.MPH_TO_MS
  _LOW_SPEED_LEAD_BLOCK_TIME_GAP_S = 1.35
  _LOW_SPEED_LEAD_BLOCK_MIN_GAP_M = 8.0
  _LOW_SPEED_LEAD_BLOCK_MAX_GAP_M = 16.0
  _LOW_SPEED_LEAD_BLOCK_OPENING_VREL_MS = 1.0
  _LOW_SPEED_LEAD_BLOCK_TARGET_MARGIN_MS = 0.35 * CV.MPH_TO_MS
  _ARBITRATION_LEAD_CRITICAL_TIME_GAP_S = 1.35
  _ARBITRATION_LEAD_FOLLOW_TIME_GAP_S = 2.35
  _ARBITRATION_LEAD_CLOSING_MS = -0.15
  _ARBITRATION_LEAD_PLANNER_DROP_MS = 1.5 * CV.MPH_TO_MS
  _ARBITRATION_STALE_LOW_TARGET_DROP_MS = 3.0 * CV.MPH_TO_MS
  _ARBITRATION_CURVE_MIN_DROP_MS = 3.0 * CV.MPH_TO_MS
  _ARBITRATION_CURVE_EGO_DROP_MS = 1.0 * CV.MPH_TO_MS
  _ARBITRATION_CURVE_MAP_VISION_DISAGREE_MS = 8.0 * CV.MPH_TO_MS
  _ARBITRATION_CURVE_EXIT_RELEASE_MS = 650
  _ARBITRATION_CURVE_ENTRY_STEP_MS = 2.2 * CV.MPH_TO_MS
  _ARBITRATION_CURVE_EXIT_STEP_MS = 3.6 * CV.MPH_TO_MS
  _FOLLOW_GAP_DEFAULT_S = 2.35
  _FOLLOW_GAP_MIN_S = 1.25
  _FOLLOW_GAP_MAX_S = 3.35
  _FOLLOW_GAP_STALK_STEP_S = 0.35
  _FOLLOW_GAP_RELEASE_HYSTERESIS_S = 0.85
  _FOLLOW_GAP_RELEASE_VREL_MS = 0.10
  _CURVE_CONFIRMED_COMFORT_BIAS_MS = 2.0 * CV.MPH_TO_MS
  _CURVE_MAPD_VISION_DISAGREE_EXTRA_BIAS_MS = 1.25 * CV.MPH_TO_MS
  _CLEAR_NO_LEAD_STALE_OWNER_DROP_MS = 1.0 * CV.MPH_TO_MS


  _LEAD_CLOSE_CANCEL_MIN_SPEED_MS = 5.0 * CV.MPH_TO_MS
  _LEAD_CLOSE_CANCEL_MAX_SPEED_MS = 32.0 * CV.MPH_TO_MS
  _LEAD_CLOSE_CANCEL_TIME_GAP_S = 1.10
  _LEAD_CLOSE_CANCEL_MIN_GAP_M = 6.0
  _LEAD_CLOSE_CANCEL_MAX_GAP_M = 11.0
  _LEAD_CLOSE_CANCEL_OPENING_VREL_MS = 0.75
  _LEAD_CLOSE_CANCEL_MIN_ACTIVE_MS = 360
  _LEAD_CLOSE_CANCEL_REPEAT_MS = 1600

  def __init__(self) -> None:
    self.acc = ACCController()
    services = ["longitudinalPlan", "radarState", "controlsState"]
    self._has_mapd = "mapdOut" in SERVICE_LIST
    if self._has_mapd:
      services.insert(2, "mapdOut")
    else:
      cloudlog.warning("Tesla LONG_module: mapdOut not present, disabling map-based speed inputs")

    self._sm = messaging.SubMaster(services)

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
    self._mapd_comfort_bias_active: bool = False

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
    self._lead_curve_hold_until_ms: int = 0
    self._lead_curve_hold_started_ms: int = 0
    self._lead_curve_hold_drel: float = 0.0
    self._lead_curve_hold_vrel: float = 0.0
    self._lead_curve_hold_yrel: float = 0.0
    self._lead_curve_hold_active: bool = False
    self._lead_raw_seen_ms: int = 0
    self._lead_context_curve_cap_candidate_since_ms: int = 0
    self._curve_recent_clear_until_ms: int = 0
    self._curve_limit_guard_candidate_since_ms: int = 0
    self._curve_limit_guard_release_candidate_since_ms: int = 0
    self._curve_limit_guard_active: bool = False
    self._curve_force_entry_candidate_since_ms: int = 0
    self._curve_steer_limit_hold_until_ms: int = 0
    self._mapd_stale_block_until_ms: int = 0
    self._controls_state_last_ns: int = 0
    self._lat_limit_saturated: bool = False
    self._lat_limit_severity: float = 0.0
    self._lat_limit_source: str = ""

    self._lead_stuck_candidate_since_ms: int = 0
    self._lead_stuck_last_drop_ms: int = 0
    self._lead_stuck_last_set_ms: float = 0.0
    self._lead_stuck_last_cancel_ms: int = 0
    self._lead_approach_candidate_since_ms: int = 0
    self._lead_approach_last_drop_ms: int = 0
    self._lead_approach_last_set_ms: float = 0.0
    self._lead_approach_last_cancel_ms: int = 0
    self._lead_approach_force_candidate_since_ms: int = 0
    self._lead_approach_force_last_cancel_ms: int = 0
    self._lead_close_cancel_candidate_since_ms: int = 0
    self._lead_close_cancel_last_cancel_ms: int = 0
    self._arbitration_state: str = "INIT"
    self._arbitration_state_since_ms: int = 0
    self._arbitration_last_update_ms: int = 0
    self._arbitration_curve_target_ms: float = 0.0


  def _reset_lead_stuck_cancel(self) -> None:
    self._lead_stuck_candidate_since_ms = 0
    self._lead_stuck_last_drop_ms = 0
    self._lead_stuck_last_set_ms = 0.0

  def _reset_lead_approach_cancel(self) -> None:
    self._lead_approach_candidate_since_ms = 0
    self._lead_approach_last_drop_ms = 0
    self._lead_approach_last_set_ms = 0.0
    self._lead_approach_force_candidate_since_ms = 0

  def _reset_lead_close_cancel(self) -> None:
    self._lead_close_cancel_candidate_since_ms = 0


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


  def _curve_specific_mapd_target_ms(
    self,
    *,
    now_ns: int,
    reference_ms: Optional[float] = None,
    planner_near_ms: Optional[float] = None,
    current_angle_deg: float = 0.0,
    planner_curve_active: bool = False,
  ) -> Optional[float]:
    self._mapd_comfort_bias_active = False
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
    if len(candidates) == 1:
      return float(candidates[0])

    low_ms = float(min(candidates))
    high_ms = float(max(candidates))
    disagreement_ms = high_ms - low_ms
    if disagreement_ms <= float(self._MAPD_DUAL_SOURCE_AGREE_MS):
      return low_ms

    planner_confirms_low = bool(
      planner_near_ms is not None
      and float(planner_near_ms) > 0.1
      and float(planner_near_ms) <= (low_ms + float(self._MAPD_DISAGREE_PLANNER_CONFIRM_MS))
    )
    steering_confirms_low = abs(float(current_angle_deg)) >= float(self._MAPD_DISAGREE_STEER_CONFIRM_DEG)
    strong_map_only_disagreement = bool(
      self._mapd_map_curve_ms is not None
      and float(self._mapd_map_curve_ms) <= (low_ms + 0.05)
      and disagreement_ms >= float(self._MAPD_MAP_ONLY_STRONG_DISAGREE_MS)
    )
    if not strong_map_only_disagreement:
      if bool(planner_curve_active) and (planner_confirms_low or steering_confirms_low):
        return low_ms
      if planner_confirms_low and steering_confirms_low:
        return low_ms
    elif planner_confirms_low and steering_confirms_low and abs(float(current_angle_deg)) >= float(self._MAPD_MAP_ONLY_LOW_CONF_STEER_DEG):
      return low_ms

    self._mapd_comfort_bias_active = True
    return float(low_ms + (disagreement_ms * float(self._MAPD_DISAGREE_COMFORT_BLEND)))

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


  def _map_only_low_confidence_curve_ms(
    self,
    *,
    raw_map_ms: Optional[float],
    raw_vision_ms: Optional[float],
    reference_ms: float,
    current_angle_deg: float = 0.0,
  ) -> Optional[float]:
    if raw_map_ms is None or raw_vision_ms is None:
      return None
    map_ms = float(raw_map_ms)
    vision_ms = float(raw_vision_ms)
    if not (math.isfinite(map_ms) and math.isfinite(vision_ms) and map_ms > 0.1 and vision_ms > 0.1):
      return None
    if vision_ms <= map_ms:
      return None
    if (vision_ms - map_ms) < float(self._MAPD_MAP_ONLY_STRONG_DISAGREE_MS):
      return None
    if vision_ms < (float(reference_ms) - float(self._MAPD_STRAIGHT_STALE_VISION_CLEAR_MS)):
      return None
    if abs(float(current_angle_deg)) >= float(self._MAPD_MAP_ONLY_LOW_CONF_STEER_DEG):
      return None
    return max(
      float(self.MIN_CRUISE_SPEED_MS),
      float(self._MAPD_MAP_ONLY_COMFORT_FLOOR_MS),
      map_ms + float(self._MAPD_MAP_ONLY_COMFORT_EXTRA_MS),
    )


  def _mapd_straight_false_positive(
    self,
    *,
    now_ns: int,
    reference_ms: float,
    planner_near_ms: float,
    current_angle_deg: float,
  ) -> bool:
    """Reject map-only curve caps that look stale on a confirmed straight."""
    raw_map_ms, raw_vision_ms = self._curve_specific_mapd_sources(now_ns=int(now_ns))
    if raw_map_ms is None or raw_vision_ms is None:
      return False

    reference_ms = float(reference_ms)
    planner_near_ms = float(planner_near_ms)
    current_angle_deg = abs(float(current_angle_deg))
    raw_map_ms = float(raw_map_ms)
    raw_vision_ms = float(raw_vision_ms)

    if raw_map_ms >= (reference_ms - float(self._MAPD_STRAIGHT_STALE_PLANNER_CLEAR_MS)):
      return False
    if (raw_vision_ms - raw_map_ms) < float(self._MAPD_STRAIGHT_STALE_DISAGREE_MS):
      return False
    if raw_vision_ms < (reference_ms - float(self._MAPD_STRAIGHT_STALE_VISION_CLEAR_MS)):
      return False
    if planner_near_ms > 0.1 and planner_near_ms < (reference_ms - float(self._MAPD_STRAIGHT_STALE_PLANNER_CLEAR_MS)):
      return False
    if current_angle_deg > float(self._MAPD_STRAIGHT_STALE_STEER_DEG):
      return False
    if bool(self._lat_limit_saturated):
      return False
    return True


  def _mapd_curve_active_target_ms(
    self,
    *,
    now_ns: int,
    reference_ms: Optional[float] = None,
    planner_near_ms: Optional[float] = None,
    current_angle_deg: float = 0.0,
    planner_curve_active: bool = False,
  ) -> Optional[float]:
    # Only curve-specific mapd sources are allowed to own active no-lead curve
    # slowdown. Generic suggestedSpeed repeatedly re-opened curve_hold[mapd] on
    # straight 30/40 roads and held the set at ~26/30 mph in the watcher logs.
    curve_specific_ms = self._curve_specific_mapd_target_ms(
      now_ns=now_ns,
      reference_ms=reference_ms,
      planner_near_ms=planner_near_ms,
      current_angle_deg=current_angle_deg,
      planner_curve_active=planner_curve_active,
    )
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
    curve_specific_ms = self._curve_specific_mapd_target_ms(
      now_ns=now_ns,
      reference_ms=reference_ms,
      planner_near_ms=planner_near_ms,
      current_angle_deg=current_angle_deg,
      planner_curve_active=planner_curve_active,
    )
    if curve_specific_ms is None:
      self._mapd_entry_candidate_since_ms = 0
      return None

    reference_ms = float(reference_ms)
    planner_near_ms = float(planner_near_ms)
    v_ego_ms = float(v_ego_ms)
    current_angle_deg = abs(float(current_angle_deg))

    if int(now_ms) <= int(self._mapd_stale_block_until_ms) and self._mapd_straight_false_positive(
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=float(planner_near_ms),
      current_angle_deg=float(current_angle_deg),
    ):
      self._mapd_entry_candidate_since_ms = 0
      return None

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
        or current_angle_deg >= max(float(self._MAPD_LOW_SPEED_STEER_HINT_DEG), 1.2)
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
    self._curve_force_entry_candidate_since_ms = 0
    self._curve_steer_limit_hold_until_ms = 0

  def _reset_lead_hold(self) -> None:
    self._lead_hold_until_ms = 0

  def _reset_lead_curve_hold(self) -> None:
    self._lead_curve_hold_until_ms = 0
    self._lead_curve_hold_started_ms = 0
    self._lead_curve_hold_drel = 0.0
    self._lead_curve_hold_vrel = 0.0
    self._lead_curve_hold_yrel = 0.0
    self._lead_curve_hold_active = False

  def _lead_curve_hold_steer_busy(self, cs_out) -> bool:
    current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
    current_rate_deg = abs(float(getattr(cs_out, "steeringRateDeg", 0.0) or 0.0))
    return bool(
      current_angle_deg >= float(self._LEAD_CURVE_HOLD_STEER_DEG)
      or current_rate_deg >= float(self._LEAD_CURVE_HOLD_STEER_RATE_DEG)
    )

  def _lead_curve_hold_should_arm(self, *, now_ms: int, v_ego_ms: float, cs_out) -> bool:
    if int(self._lead_curve_hold_started_ms) > 0:
      if int(now_ms) > int(self._lead_curve_hold_until_ms):
        self._reset_lead_curve_hold()
      return False
    # Only bridge a fresh raw-radar dropout. Do not let a virtual held lead re-arm itself.
    if int(self._lead_raw_seen_ms) <= 0 or (int(now_ms) - int(self._lead_raw_seen_ms)) > 240:
      return False
    if float(v_ego_ms) < float(self._LEAD_CURVE_HOLD_MIN_SPEED_MS):
      return False
    if not self._lead_curve_hold_steer_busy(cs_out):
      return False
    if float(self._lead_last_drel) <= 0.0:
      return False
    if float(self._lead_last_drel) > float(self._LEAD_CURVE_HOLD_MAX_GAP_M):
      return False
    if abs(float(self._lead_last_yrel)) > float(self._LEAD_CURVE_HOLD_MAX_YREL_M):
      return False
    if self._lead_last_sample_was_opening_clear():
      return False
    if int(now_ms) <= int(self._lead_curve_hold_until_ms):
      return False
    return True

  def _lead_curve_hold_use(self, *, now_ms: int, v_ego_ms: float, cs_out) -> bool:
    if int(now_ms) > int(self._lead_curve_hold_until_ms):
      self._reset_lead_curve_hold()
      return False
    if float(v_ego_ms) < float(self._LEAD_CURVE_HOLD_MIN_SPEED_MS):
      self._reset_lead_curve_hold()
      return False
    if not self._lead_curve_hold_steer_busy(cs_out):
      self._reset_lead_curve_hold()
      return False
    if float(self._lead_curve_hold_drel) <= 0.0:
      self._reset_lead_curve_hold()
      return False
    if abs(float(self._lead_curve_hold_yrel)) > float(self._LEAD_CURVE_HOLD_MAX_YREL_M):
      self._reset_lead_curve_hold()
      return False
    return True

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
      strong_pre_entry = delta_ms >= float(self._CURVE_PRE_ENTRY_MIN_DROP_MS)
      preview_limit_ms = (
        float(self._CURVE_PRE_ENTRY_PREVIEW_DROP_MS)
        if strong_pre_entry
        else float(self._CURVE_ENTRY_PREVIEW_DROP_MS)
      )

      if delta_ms >= hard_entry_threshold_ms:
        self._curve_hold_active = True
        self._curve_hold_target_ms = max(
          float(raw_target_ms),
          float(reference_ms) - min(float(preview_limit_ms), float(delta_ms)),
        )
        self._curve_hold_last_update_ms = int(now_ms)
        self._curve_entry_candidate_since_ms = 0
        self._curve_exit_candidate_since_ms = 0
        state = "curve_pre_entry(hard)" if strong_pre_entry else "curve_hold(hard_entry)"
        return float(self._curve_hold_target_ms), state

      if delta_ms >= entry_threshold_ms:
        if int(self._curve_entry_candidate_since_ms) == 0:
          self._curve_entry_candidate_since_ms = int(now_ms)
        elapsed_ms = max(0, int(now_ms) - int(self._curve_entry_candidate_since_ms))
        preview_ratio = min(1.0, float(elapsed_ms) / max(1.0, float(self._CURVE_ENTRY_PERSIST_MS)))
        if strong_pre_entry:
          preview_ratio = max(float(self._CURVE_PRE_ENTRY_MIN_RATIO), preview_ratio)
        preview_drop_ms = min(float(preview_limit_ms), float(delta_ms)) * preview_ratio
        preview_target_ms = max(float(raw_target_ms), float(reference_ms) - preview_drop_ms)
        if elapsed_ms >= int(self._CURVE_ENTRY_PERSIST_MS):
          self._curve_hold_active = True
          self._curve_hold_target_ms = float(preview_target_ms)
          self._curve_hold_last_update_ms = int(now_ms)
          self._curve_entry_candidate_since_ms = 0
          self._curve_exit_candidate_since_ms = 0
          state = "curve_pre_entry(entry)" if strong_pre_entry else "curve_hold(entry)"
          return float(self._curve_hold_target_ms), state
        state = "curve_pre_entry(wait)" if strong_pre_entry else "curve_gate(wait)"
        return float(preview_target_ms), state
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

  @staticmethod
  def _lead_values(lead) -> tuple[bool, float, float, float]:
    if lead is None or not bool(getattr(lead, "status", False)):
      return False, 0.0, 0.0, 0.0
    try:
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)
      y_rel = float(getattr(lead, "yRel", 0.0) or 0.0)
    except Exception:
      return False, 0.0, 0.0, 0.0
    return bool(d_rel > 0.0), d_rel, v_rel, y_rel

  def _secondary_lead_fallback_allowed(self, *, d_rel: float, y_rel: float) -> bool:
    if float(d_rel) <= 0.0:
      return False
    if float(d_rel) > float(self._SECONDARY_LEAD_FALLBACK_MAX_GAP_M):
      return False
    if abs(float(y_rel)) > float(self._SECONDARY_LEAD_FALLBACK_MAX_YREL_M):
      return False
    if float(self._lead_last_drel) <= 0.0:
      return True
    return abs(float(d_rel) - float(self._lead_last_drel)) <= float(self._SECONDARY_LEAD_FALLBACK_LAST_DREL_DELTA_M)

  def _poll_plan_and_lead(self, *, now_ns: int, cs_out) -> None:
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
      now_ms = int(now_ns // 1_000_000)
      v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
      self._lead_present = False
      self._lead_drel = 0.0
      self._lead_vrel = 0.0
      self._lead_yrel = 0.0
      self._lead_curve_hold_active = False

      raw_lead_present = False
      raw_d_rel = 0.0
      raw_v_rel = 0.0
      raw_y_rel = 0.0

      if bool(self._sm.valid.get("radarState", False)):
        rs = self._sm["radarState"]
        lead_one = getattr(rs, "leadOne", None)
        lead_two = getattr(rs, "leadTwo", None)

        raw_lead_present, raw_d_rel, raw_v_rel, raw_y_rel = self._lead_values(lead_one)
        if not raw_lead_present:
          lead_two_present, lead_two_d_rel, lead_two_v_rel, lead_two_y_rel = self._lead_values(lead_two)
          if lead_two_present and self._secondary_lead_fallback_allowed(d_rel=lead_two_d_rel, y_rel=lead_two_y_rel):
            raw_lead_present = True
            raw_d_rel = lead_two_d_rel
            raw_v_rel = lead_two_v_rel
            raw_y_rel = lead_two_y_rel

      if raw_lead_present:
        self._lead_raw_seen_ms = int(now_ms)
        self._lead_present = True
        self._lead_drel = raw_d_rel
        self._lead_vrel = raw_v_rel
        self._lead_yrel = raw_y_rel
        self._lead_last_drel = raw_d_rel
        self._lead_last_vrel = raw_v_rel
        self._lead_last_yrel = raw_y_rel
        self._reset_lead_curve_hold()
      else:
        if prev_lead_present and self._lead_curve_hold_should_arm(
          now_ms=now_ms,
          v_ego_ms=float(v_ego_ms),
          cs_out=cs_out,
        ):
          self._lead_curve_hold_started_ms = int(now_ms)
          self._lead_curve_hold_until_ms = int(now_ms) + int(self._LEAD_CURVE_HOLD_MAX_MS)
          self._lead_curve_hold_drel = float(self._lead_last_drel)
          self._lead_curve_hold_vrel = float(self._lead_last_vrel)
          self._lead_curve_hold_yrel = float(self._lead_last_yrel)

        if self._lead_curve_hold_use(
          now_ms=now_ms,
          v_ego_ms=float(v_ego_ms),
          cs_out=cs_out,
        ):
          elapsed_s = max(0.0, float(now_ms - int(self._lead_curve_hold_started_ms)) / 1000.0)
          est_d_rel = float(self._lead_curve_hold_drel) + (float(self._lead_curve_hold_vrel) * elapsed_s)
          if est_d_rel > 0.1:
            self._lead_present = True
            self._lead_drel = est_d_rel
            self._lead_vrel = float(self._lead_curve_hold_vrel)
            self._lead_yrel = float(self._lead_curve_hold_yrel)
            self._lead_curve_hold_active = True
          else:
            self._reset_lead_curve_hold()
        else:
          self._reset_lead_curve_hold()
    except Exception:
      self._reset_lead_curve_hold()

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
      if self._has_mapd and bool(self._sm.valid.get("mapdOut", False)):
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
    if bool(self._lead_curve_hold_active) and float(self._lead_drel) > 0.0:
      return True
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



  def _lead_context_mapd_curve_cap_ms(
    self,
    *,
    now_ms: int,
    now_ns: int,
    desired_ms: float,
    reference_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
  ) -> tuple[Optional[float], str]:
    """Return a curve cap when lead context would otherwise mask a roundabout curve."""
    if float(v_ego_ms) < float(self._LEAD_CONTEXT_CURVE_CAP_MIN_SPEED_MS):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""
    if float(v_ego_ms) > float(self._LEAD_CONTEXT_CURVE_CAP_MAX_SPEED_MS):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    in_lead_context = bool(self._lead_present) or bool(self._lp_has_lead) or int(now_ms) <= int(self._lead_recently_cleared_until_ms)
    if not in_lead_context:
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    if int(self._mapd_last_ns) <= 0 or (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    raw_map_ms: Optional[float] = None
    if self._mapd_map_curve_ms is not None:
      raw = float(self._mapd_map_curve_ms)
      if math.isfinite(raw) and raw > 0.1:
        raw_map_ms = raw

    raw_vision_ms: Optional[float] = None
    if self._mapd_vision_curve_ms is not None:
      raw = float(self._mapd_vision_curve_ms)
      if math.isfinite(raw) and raw > 0.1:
        raw_vision_ms = raw

    if raw_map_ms is None and raw_vision_ms is None:
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    raw_candidates = [v for v in (raw_map_ms, raw_vision_ms) if v is not None]
    raw_low_ms = float(min(raw_candidates))
    map_owned_low = raw_map_ms is not None and raw_map_ms <= (raw_low_ms + 0.05)
    if not map_owned_low:
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    reference_ms = float(reference_ms)
    desired_ms = float(desired_ms)
    v_ego_ms = float(v_ego_ms)
    map_only_comfort_ms = self._map_only_low_confidence_curve_ms(
      raw_map_ms=raw_map_ms,
      raw_vision_ms=raw_vision_ms,
      reference_ms=float(reference_ms),
      current_angle_deg=float(current_angle_deg),
    )
    if map_only_comfort_ms is not None:
      raw_low_ms = max(float(raw_low_ms), float(map_only_comfort_ms))

    if raw_low_ms >= (reference_ms - float(self._LEAD_CONTEXT_CURVE_CAP_MIN_DROP_MS)):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""
    if raw_low_ms >= (v_ego_ms - float(self._LEAD_CONTEXT_CURVE_CAP_EGO_DROP_MS)):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""
    if desired_ms <= (raw_low_ms + float(self._LEAD_CONTEXT_CURVE_CAP_TARGET_OFFSET_MS)):
      self._lead_context_curve_cap_candidate_since_ms = 0
      return None, ""

    if int(self._lead_context_curve_cap_candidate_since_ms) == 0:
      self._lead_context_curve_cap_candidate_since_ms = int(now_ms)
      return None, ""

    elapsed_ms = int(now_ms) - int(self._lead_context_curve_cap_candidate_since_ms)
    if elapsed_ms < int(self._LEAD_CONTEXT_CURVE_CAP_PERSIST_MS):
      return None, ""

    preview_cap_ms = max(
      float(self.MIN_CRUISE_SPEED_MS),
      min(
        raw_low_ms + float(self._LEAD_CONTEXT_CURVE_CAP_TARGET_OFFSET_MS),
        reference_ms - min(
          float(self._LEAD_CONTEXT_CURVE_CAP_PREVIEW_DROP_MS),
          max(0.0, reference_ms - raw_low_ms),
        ),
      ),
    )

    if raw_vision_ms is not None and raw_vision_ms <= (reference_ms - float(self._LEAD_CONTEXT_CURVE_CAP_MIN_DROP_MS)):
      state = "mapd_lead_curve[map+vision]"
    elif int(now_ms) <= int(self._lead_recently_cleared_until_ms):
      state = "mapd_lead_curve[recent_clear]"
    else:
      state = "mapd_lead_curve[map_persist]"

    return float(preview_cap_ms), state


  def _steer_busy_for_curve(self, *, current_angle_deg: float, steering_rate_deg: float) -> bool:
    return bool(
      abs(float(current_angle_deg)) >= float(self._CURVE_ACCEL_BLOCK_STEER_DEG)
      or abs(float(steering_rate_deg)) >= float(self._CURVE_ACCEL_BLOCK_STEER_RATE_DEG)
      or bool(self._lat_limit_saturated)
    )

  def _curve_force_entry_cap_ms(
    self,
    *,
    now_ms: int,
    now_ns: int,
    desired_ms: float,
    reference_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
    steering_rate_deg: float,
    lead_context: bool,
  ) -> tuple[Optional[float], str]:
    """Force a curve cap when speed/lead ownership would otherwise mask entry."""
    if float(v_ego_ms) < float(self._CURVE_FORCE_ENTRY_MIN_SPEED_MS):
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    if int(self._mapd_last_ns) <= 0 or (int(now_ns) - int(self._mapd_last_ns)) >= int(self._MAPD_FRESH_NS):
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    raw_map_ms, raw_vision_ms = self._curve_specific_mapd_sources(now_ns=int(now_ns))
    raw_candidates = [float(v) for v in (raw_map_ms, raw_vision_ms) if v is not None and math.isfinite(float(v)) and float(v) > 0.1]
    if not raw_candidates:
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    raw_low_ms = float(min(raw_candidates))
    map_owned_low = bool(raw_map_ms is not None and float(raw_map_ms) <= (raw_low_ms + 0.05))
    vision_supports = bool(raw_vision_ms is not None and float(raw_vision_ms) <= (float(reference_ms) - float(self._CURVE_FORCE_ENTRY_MIN_DROP_MS)))

    curve_specific_ms = self._curve_specific_mapd_target_ms(
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=None,
      current_angle_deg=float(current_angle_deg),
      planner_curve_active=False,
    )
    if curve_specific_ms is None:
      curve_specific_ms = raw_low_ms

    map_only_comfort_ms = self._map_only_low_confidence_curve_ms(
      raw_map_ms=raw_map_ms,
      raw_vision_ms=raw_vision_ms,
      reference_ms=float(reference_ms),
      current_angle_deg=float(current_angle_deg),
    )
    if map_only_comfort_ms is not None:
      curve_specific_ms = max(float(curve_specific_ms), float(map_only_comfort_ms))

    # For confirmed roundabouts, the map source can be right while vision still sees a straight road.
    # Do not force raw map speed from lead context alone; require strong steering or vision support.
    if bool(map_owned_low) and (
      bool(vision_supports)
      or abs(float(current_angle_deg)) >= float(self._MAPD_MAP_ONLY_LOW_CONF_STEER_DEG)
      or bool(self._lat_limit_saturated)
    ):
      curve_specific_ms = min(float(curve_specific_ms), float(raw_low_ms) + float(self._CURVE_FORCE_ENTRY_TARGET_OFFSET_MS))

    reference_ms = float(reference_ms)
    desired_ms = float(desired_ms)
    v_ego_ms = float(v_ego_ms)
    curve_specific_ms = float(curve_specific_ms)

    meaningful_drop = curve_specific_ms <= (reference_ms - float(self._CURVE_FORCE_ENTRY_MIN_DROP_MS))
    ego_too_fast = curve_specific_ms <= (v_ego_ms - float(self._CURVE_FORCE_ENTRY_EGO_DROP_MS))
    if not (meaningful_drop and ego_too_fast):
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    steer_busy = self._steer_busy_for_curve(current_angle_deg=current_angle_deg, steering_rate_deg=steering_rate_deg)
    map_only_without_confirmation = bool(map_owned_low and not (vision_supports or steer_busy or bool(self._lat_limit_saturated)))
    if map_only_without_confirmation:
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    allowed_context = bool(lead_context or steer_busy or vision_supports or bool(self._lat_limit_saturated))
    if not allowed_context:
      self._curve_force_entry_candidate_since_ms = 0
      return None, ""

    persist_ms = int(self._CURVE_FORCE_ENTRY_PERSIST_MS)
    if steer_busy or bool(self._lat_limit_saturated):
      persist_ms = min(persist_ms, int(self._CURVE_FORCE_ENTRY_STEER_PERSIST_MS))

    if int(self._curve_force_entry_candidate_since_ms) == 0:
      self._curve_force_entry_candidate_since_ms = int(now_ms)
      return None, ""

    elapsed_ms = int(now_ms) - int(self._curve_force_entry_candidate_since_ms)
    if elapsed_ms < int(persist_ms):
      return None, ""

    cap_ms = max(
      float(self.MIN_CRUISE_SPEED_MS),
      min(
        float(curve_specific_ms) + float(self._CURVE_FORCE_ENTRY_TARGET_OFFSET_MS),
        reference_ms - min(float(self._CURVE_FORCE_ENTRY_PREVIEW_DROP_MS), max(0.0, reference_ms - curve_specific_ms)),
      ),
    )

    state = "curve_pre_entry[brake_distance]"
    if bool(lead_context):
      state = "mapd_lead_curve[force_entry]"
    if steer_busy:
      state = f"{state}+curve_accel_block[steer_busy]"

    if bool(self._lat_limit_saturated) or int(now_ms) <= int(self._curve_steer_limit_hold_until_ms):
      self._curve_steer_limit_hold_until_ms = int(now_ms) + int(self._CURVE_STEER_LIMIT_HOLD_MS)
      cap_ms = max(float(self.MIN_CRUISE_SPEED_MS), float(cap_ms) - float(self._CURVE_STEER_LIMIT_HOLD_DROP_MS))
      state = f"{state}+curve_steer_limit_hold"

    if float(cap_ms) >= (desired_ms - (0.25 * CV.MPH_TO_MS)) and not steer_busy:
      return None, ""

    return float(cap_ms), state

  def _curve_accel_block_target(
    self,
    *,
    now_ns: int,
    desired_ms: float,
    reference_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
    steering_rate_deg: float,
  ) -> tuple[float, bool]:
    if not self._steer_busy_for_curve(current_angle_deg=current_angle_deg, steering_rate_deg=steering_rate_deg):
      return float(desired_ms), False

    curve_specific_ms = self._curve_specific_mapd_target_ms(
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=None,
      current_angle_deg=float(current_angle_deg),
      planner_curve_active=False,
    )
    if curve_specific_ms is None:
      return float(desired_ms), False

    if float(curve_specific_ms) > (float(reference_ms) - float(self._CURVE_ACCEL_BLOCK_MIN_DROP_MS)):
      return float(desired_ms), False
    if float(curve_specific_ms) > (float(v_ego_ms) - float(self._CURVE_ACCEL_BLOCK_EGO_MARGIN_MS)):
      return float(desired_ms), False

    accel_block_ms = max(0.0, max(float(current_set_ms), float(v_ego_ms)) - (0.25 * CV.MPH_TO_MS))
    if float(desired_ms) <= (float(accel_block_ms) + 0.01):
      return float(desired_ms), False
    return float(accel_block_ms), True


  def _apply_lead_present_mapd_cap(
    self,
    *,
    now_ms: int,
    now_ns: int,
    desired_ms: float,
    reference_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
  ) -> tuple[float, bool]:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return float(desired_ms), False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return float(desired_ms), False

    if self._lead_is_constraining(base_target_ms=float(reference_ms), v_ego_ms=float(v_ego_ms)):
      return float(desired_ms), False
    if self._lead_planner_guard_active(
      base_target_ms=float(reference_ms),
      planner_ms=float(planner_last_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
    ):
      return float(desired_ms), False
    if self._lead_follow_hold_needed(base_target_ms=float(reference_ms), v_ego_ms=float(v_ego_ms)):
      return float(desired_ms), False

    mapd_target_ms = self._mapd_entry_target_ms(
      now_ms=int(now_ms),
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=float(planner_near_ms),
      v_ego_ms=float(v_ego_ms),
      current_angle_deg=float(current_angle_deg),
      planner_curve_active=False,
    )
    if mapd_target_ms is None:
      return float(desired_ms), False

    capped_ms = min(float(desired_ms), max(float(self.MIN_CRUISE_SPEED_MS), float(mapd_target_ms)))
    if capped_ms >= (float(desired_ms) - (0.25 * CV.MPH_TO_MS)):
      return float(desired_ms), False
    return float(capped_ms), True


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

  def _lead_planner_guard_active(self, *, base_target_ms: float, planner_ms: float, current_set_ms: float, v_ego_ms: float) -> bool:
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return False
    if float(planner_ms) <= 0.1:
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return False

    guard_gap_m = min(
      float(self._LEAD_PLANNER_GUARD_MAX_GAP_M),
      max(float(self._LEAD_PLANNER_GUARD_MIN_GAP_M), float(v_ego_ms) * float(self._LEAD_PLANNER_GUARD_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(guard_gap_m):
      return False

    reference_ms = max(float(base_target_ms), float(current_set_ms), float(v_ego_ms))
    planner_below_ref = float(planner_ms) < (float(reference_ms) - float(self._LEAD_PLANNER_GUARD_MIN_DROP_MS))
    planner_below_ego = float(planner_ms) < (float(v_ego_ms) - float(self._LEAD_PLANNER_GUARD_EGO_MARGIN_MS))
    planner_decel = float(self._lp_a_target) <= float(self._LEAD_OPENING_RELAX_ATARGET_MS2)

    return bool(planner_below_ref and (planner_below_ego or planner_decel))


  def _low_speed_lead_block_target(
    self,
    *,
    now_ms: int,
    desired_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
  ) -> tuple[float, bool]:
    del now_ms
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      return float(desired_ms), False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      return float(desired_ms), False
    if float(v_ego_ms) > float(self._LOW_SPEED_LEAD_BLOCK_MAX_SPEED_MS):
      return float(desired_ms), False
    if self._lead_is_opening_clear(base_target_ms=max(float(current_set_ms), float(desired_ms)), v_ego_ms=float(v_ego_ms)):
      return float(desired_ms), False

    gap_limit_m = min(
      float(self._LOW_SPEED_LEAD_BLOCK_MAX_GAP_M),
      max(float(self._LOW_SPEED_LEAD_BLOCK_MIN_GAP_M), float(v_ego_ms) * float(self._LOW_SPEED_LEAD_BLOCK_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(gap_limit_m):
      return float(desired_ms), False
    if float(self._lead_vrel) > float(self._LOW_SPEED_LEAD_BLOCK_OPENING_VREL_MS):
      return float(desired_ms), False

    no_accel_ceiling_ms = max(0.0, float(v_ego_ms) - float(self._LOW_SPEED_LEAD_BLOCK_TARGET_MARGIN_MS))
    if float(current_set_ms) > 0.1:
      no_accel_ceiling_ms = min(float(no_accel_ceiling_ms), float(current_set_ms))
    blocked_ms = min(float(desired_ms), float(no_accel_ceiling_ms))
    return float(blocked_ms), bool(blocked_ms < (float(desired_ms) - 0.05))

  def _lead_close_cancel_needed(
    self,
    *,
    now_ms: int,
    current_set_ms: float,
    v_ego_ms: float,
    brake_pressed: bool,
  ) -> bool:
    if bool(brake_pressed):
      self._reset_lead_close_cancel()
      return False
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      self._reset_lead_close_cancel()
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      self._reset_lead_close_cancel()
      return False
    if float(v_ego_ms) < float(self._LEAD_CLOSE_CANCEL_MIN_SPEED_MS):
      self._reset_lead_close_cancel()
      return False
    if float(v_ego_ms) > float(self._LEAD_CLOSE_CANCEL_MAX_SPEED_MS):
      self._reset_lead_close_cancel()
      return False
    if float(current_set_ms) <= 0.1:
      self._reset_lead_close_cancel()
      return False
    if self._lead_is_opening_clear(base_target_ms=float(current_set_ms), v_ego_ms=float(v_ego_ms)):
      self._reset_lead_close_cancel()
      return False
    if float(self._lead_vrel) > float(self._LEAD_CLOSE_CANCEL_OPENING_VREL_MS):
      self._reset_lead_close_cancel()
      return False

    gap_limit_m = min(
      float(self._LEAD_CLOSE_CANCEL_MAX_GAP_M),
      max(float(self._LEAD_CLOSE_CANCEL_MIN_GAP_M), float(v_ego_ms) * float(self._LEAD_CLOSE_CANCEL_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(gap_limit_m):
      self._reset_lead_close_cancel()
      return False

    now = int(now_ms)
    if self._lead_close_cancel_candidate_since_ms <= 0:
      self._lead_close_cancel_candidate_since_ms = now
      return False

    if (now - int(self._lead_close_cancel_last_cancel_ms)) < int(self._LEAD_CLOSE_CANCEL_REPEAT_MS):
      return False
    if (now - int(self._lead_close_cancel_candidate_since_ms)) < int(self._LEAD_CLOSE_CANCEL_MIN_ACTIVE_MS):
      return False

    self._lead_close_cancel_last_cancel_ms = now
    return True


  def _lead_approach_force_cancel_needed(
    self,
    *,
    now_ms: int,
    desired_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
    brake_pressed: bool,
  ) -> bool:
    if bool(brake_pressed):
      self._lead_approach_force_candidate_since_ms = 0
      return False
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      self._lead_approach_force_candidate_since_ms = 0
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      self._lead_approach_force_candidate_since_ms = 0
      return False
    if float(v_ego_ms) < float(self._LEAD_APPROACH_CANCEL_MIN_SPEED_MS):
      self._lead_approach_force_candidate_since_ms = 0
      return False
    if float(v_ego_ms) > float(self._LEAD_APPROACH_CANCEL_MAX_SPEED_MS):
      self._lead_approach_force_candidate_since_ms = 0
      return False
    if float(current_set_ms) <= 0.1 or float(desired_ms) <= 0.1:
      self._lead_approach_force_candidate_since_ms = 0
      return False

    set_drop_needed_ms = float(current_set_ms) - float(desired_ms)
    if set_drop_needed_ms < float(self._LEAD_APPROACH_FORCE_CANCEL_MIN_DROP_MS):
      self._lead_approach_force_candidate_since_ms = 0
      return False

    planner_floor_ms = min(float(planner_last_ms), float(planner_near_ms))
    planner_supported = bool(
      planner_floor_ms > 0.1
      and planner_floor_ms < (float(current_set_ms) - float(self._LEAD_APPROACH_CANCEL_PLANNER_MARGIN_MS))
      and (
        planner_floor_ms < (float(v_ego_ms) - (0.25 * CV.MPH_TO_MS))
        or float(self._lp_a_target) <= float(self._LEAD_APPROACH_FORCE_CANCEL_ATARGET_MS2)
        or "lead" in str(self._lp_source or "")
      )
    )
    if not planner_supported:
      self._lead_approach_force_candidate_since_ms = 0
      return False

    gap_limit_m = min(
      float(self._LEAD_APPROACH_FORCE_CANCEL_MAX_GAP_M),
      max(float(self._LEAD_APPROACH_CANCEL_MAX_GAP_M), float(v_ego_ms) * float(self._LEAD_APPROACH_FORCE_CANCEL_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(gap_limit_m):
      self._lead_approach_force_candidate_since_ms = 0
      return False

    closing_or_decel = bool(
      float(self._lead_vrel) <= float(self._LEAD_APPROACH_FORCE_CANCEL_CLOSING_MS)
      or float(self._lp_a_target) <= float(self._LEAD_APPROACH_FORCE_CANCEL_ATARGET_MS2)
      or "lead+atarget" in str(self._lp_source or "")
    )
    if not closing_or_decel:
      if float(self._lead_vrel) >= float(self._LEAD_APPROACH_CANCEL_OPENING_RESET_MS) and float(self._lp_a_target) > float(self._LEAD_APPROACH_CANCEL_ATARGET_MS2):
        self._lead_approach_force_candidate_since_ms = 0
      return False

    now = int(now_ms)
    if (now - int(self._lead_approach_force_last_cancel_ms)) < int(self._LEAD_APPROACH_FORCE_CANCEL_REPEAT_MS):
      return False

    severe_drop = set_drop_needed_ms >= (float(self._LEAD_APPROACH_FORCE_CANCEL_MIN_DROP_MS) + (3.0 * CV.MPH_TO_MS))
    if bool(severe_drop and closing_or_decel):
      self._lead_approach_force_last_cancel_ms = now
      self._lead_approach_force_candidate_since_ms = 0
      return True

    if int(self._lead_approach_force_candidate_since_ms) == 0:
      self._lead_approach_force_candidate_since_ms = now
      return False

    if (now - int(self._lead_approach_force_candidate_since_ms)) >= int(self._LEAD_APPROACH_FORCE_CANCEL_ARM_MS):
      self._lead_approach_force_last_cancel_ms = now
      self._lead_approach_force_candidate_since_ms = 0
      return True

    return False


  def _lead_approach_cancel_needed(
    self,
    *,
    now_ms: int,
    desired_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
    brake_pressed: bool,
  ) -> bool:
    if bool(brake_pressed):
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False
    if float(v_ego_ms) < float(self._LEAD_APPROACH_CANCEL_MIN_SPEED_MS):
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False
    if float(v_ego_ms) > float(self._LEAD_APPROACH_CANCEL_MAX_SPEED_MS):
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False
    if float(current_set_ms) <= 0.1 or float(desired_ms) <= 0.1:
      self._reset_lead_approach_cancel()
      self._reset_lead_close_cancel()
      return False

    set_drop_needed_ms = float(current_set_ms) - float(desired_ms)
    if set_drop_needed_ms < float(self._LEAD_APPROACH_CANCEL_MIN_DROP_MS):
      self._reset_lead_approach_cancel()
      return False

    planner_floor_ms = min(float(planner_last_ms), float(planner_near_ms))
    planner_supported = bool(
      planner_floor_ms > 0.1
      and planner_floor_ms < (float(current_set_ms) - float(self._LEAD_APPROACH_CANCEL_PLANNER_MARGIN_MS))
      and (
        planner_floor_ms < (float(v_ego_ms) - (0.4 * CV.MPH_TO_MS))
        or float(self._lp_a_target) <= float(self._LEAD_APPROACH_CANCEL_ATARGET_MS2)
      )
    )
    if not planner_supported:
      self._reset_lead_approach_cancel()
      return False

    gap_limit_m = min(
      float(self._LEAD_APPROACH_CANCEL_MAX_GAP_M),
      max(float(self._LEAD_APPROACH_CANCEL_MIN_GAP_M), float(v_ego_ms) * float(self._LEAD_APPROACH_CANCEL_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(gap_limit_m):
      self._reset_lead_approach_cancel()
      return False

    critical_gap = float(self._lead_drel) <= float(self._LEAD_APPROACH_CANCEL_CRITICAL_GAP_M)
    closing_now = float(self._lead_vrel) <= float(self._LEAD_APPROACH_CANCEL_CLOSING_MS)
    strong_decel_plan = float(self._lp_a_target) <= float(self._LEAD_APPROACH_CANCEL_STRONG_ATARGET_MS2)
    if not (closing_now or (critical_gap and strong_decel_plan)):
      if float(self._lead_vrel) >= float(self._LEAD_APPROACH_CANCEL_OPENING_RESET_MS) and float(self._lp_a_target) > float(self._LEAD_APPROACH_CANCEL_ATARGET_MS2):
        self._reset_lead_approach_cancel()
      return False

    now = int(now_ms)
    if self._lead_approach_candidate_since_ms <= 0:
      self._lead_approach_candidate_since_ms = now
      self._lead_approach_last_drop_ms = now
      self._lead_approach_last_set_ms = float(current_set_ms)
      return False

    drop_eps_ms = float(self._LEAD_APPROACH_CANCEL_SET_DROP_EPS_MS)
    if float(current_set_ms) < (float(self._lead_approach_last_set_ms) - drop_eps_ms):
      self._lead_approach_last_drop_ms = now
      self._lead_approach_last_set_ms = float(current_set_ms)
      return False
    if float(current_set_ms) > (float(self._lead_approach_last_set_ms) + drop_eps_ms):
      self._lead_approach_last_set_ms = float(current_set_ms)

    if (now - int(self._lead_approach_last_cancel_ms)) < int(self._LEAD_APPROACH_CANCEL_REPEAT_MS):
      return False

    active_long_enough = (now - int(self._lead_approach_candidate_since_ms)) >= int(self._LEAD_APPROACH_CANCEL_MIN_ACTIVE_MS)
    no_set_drop_long_enough = (now - int(self._lead_approach_last_drop_ms)) >= int(self._LEAD_APPROACH_CANCEL_NO_DROP_MS)
    if bool(active_long_enough and no_set_drop_long_enough):
      self._lead_approach_last_cancel_ms = now
      return True

    return False


  def _lead_stuck_cancel_needed(
    self,
    *,
    now_ms: int,
    desired_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
    brake_pressed: bool,
  ) -> bool:
    if bool(brake_pressed):
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False
    if (not self._lead_present) or float(self._lead_drel) <= 0.0:
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False
    if abs(float(self._lead_yrel)) >= float(self._LEAD_OFFLANE_YREL_M):
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False
    if float(current_set_ms) <= 0.1 or float(desired_ms) <= 0.1:
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False

    set_drop_needed_ms = float(current_set_ms) - float(desired_ms)
    if set_drop_needed_ms < float(self._LEAD_STUCK_CANCEL_MIN_DROP_MS):
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False

    planner_floor_ms = min(float(planner_last_ms), float(planner_near_ms))
    planner_supported = bool(
      planner_floor_ms > 0.1
      and planner_floor_ms < (float(current_set_ms) - float(self._LEAD_STUCK_CANCEL_PLANNER_MARGIN_MS))
    )
    if not planner_supported:
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False

    if float(self._lead_vrel) > float(self._LEAD_STUCK_CANCEL_MIN_CLOSING_MS):
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False

    gap_limit_m = min(
      float(self._LEAD_STUCK_CANCEL_MAX_GAP_M),
      max(float(self._LEAD_STUCK_CANCEL_MIN_GAP_M), float(v_ego_ms) * float(self._LEAD_STUCK_CANCEL_TIME_GAP_S)),
    )
    if float(self._lead_drel) > float(gap_limit_m):
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      return False

    now = int(now_ms)
    if self._lead_stuck_candidate_since_ms <= 0:
      self._lead_stuck_candidate_since_ms = now
      self._lead_stuck_last_drop_ms = now
      self._lead_stuck_last_set_ms = float(current_set_ms)
      return False

    drop_eps_ms = float(self._LEAD_STUCK_CANCEL_SET_DROP_EPS_MS)
    if float(current_set_ms) < (float(self._lead_stuck_last_set_ms) - drop_eps_ms):
      self._lead_stuck_last_drop_ms = now
      self._lead_stuck_last_set_ms = float(current_set_ms)
      return False
    if float(current_set_ms) > (float(self._lead_stuck_last_set_ms) + drop_eps_ms):
      self._lead_stuck_last_set_ms = float(current_set_ms)

    if (now - int(self._lead_stuck_last_cancel_ms)) < int(self._LEAD_STUCK_CANCEL_REPEAT_MS):
      return False

    active_long_enough = (now - int(self._lead_stuck_candidate_since_ms)) >= int(self._LEAD_STUCK_CANCEL_MIN_ACTIVE_MS)
    no_set_drop_long_enough = (now - int(self._lead_stuck_last_drop_ms)) >= int(self._LEAD_STUCK_CANCEL_NO_DROP_MS)
    if bool(active_long_enough and no_set_drop_long_enough):
      self._lead_stuck_last_cancel_ms = now
      return True

    return False


  def _stale_planner_lead_without_live_lead(
    self,
    *,
    now_ms: int,
    base_target_ms: float,
    planner_ms: float,
    v_ego_ms: float,
  ) -> bool:
    if bool(self._lead_present) or not bool(self._lp_has_lead):
      return False
    if int(now_ms) <= int(self._lead_recently_cleared_until_ms):
      return False
    if (int(now_ms) - int(self._lead_raw_seen_ms)) < int(self._STALE_PLANNER_LEAD_CLEAR_MS):
      return False
    if str(self._lp_source or "") in ("lead0", "lead1"):
      return False
    if float(self._lp_a_target) <= float(self._STALE_PLANNER_LEAD_MAX_ATARGET_MS2):
      return False

    # Keep genuine model-led decel, but drop stale planner-lead ownership that
    # only tracks the current set speed after radar has cleared.
    live_drop_ms = max(0.0, min(float(base_target_ms), float(v_ego_ms)) - float(planner_ms))
    if live_drop_ms > float(self._STALE_PLANNER_LEAD_MAX_LIVE_DROP_MS):
      return False

    set_drop_ms = max(0.0, float(base_target_ms) - float(planner_ms))
    return bool(set_drop_ms <= float(self._STALE_PLANNER_LEAD_MIN_DROP_MS) or float(planner_ms) >= (float(v_ego_ms) - 0.25 * CV.MPH_TO_MS))


  def _raw_follow_gap_value(self, cs_out) -> float:
    candidates: list[object] = [
      getattr(cs_out, "followDistance", None),
      getattr(cs_out, "cruiseGap", None),
      getattr(cs_out, "distanceSetting", None),
      getattr(cs_out, "accDistance", None),
      getattr(cs_out, "stockFollowDistance", None),
      getattr(cs_out, "teslaFollowDistance", None),
    ]
    cruise_state = getattr(cs_out, "cruiseState", None)
    if cruise_state is not None:
      candidates.extend([
        getattr(cruise_state, "gap", None),
        getattr(cruise_state, "followDistance", None),
        getattr(cruise_state, "modeSel", None),
      ])

    for value in candidates:
      try:
        gap = float(value)
      except Exception:
        continue
      if math.isfinite(gap) and gap > 0.01:
        return float(gap)
    return 0.0


  def _desired_follow_time_s(self, cs_out) -> float:
    raw_gap = self._raw_follow_gap_value(cs_out)
    if raw_gap <= 0.01:
      return float(self._FOLLOW_GAP_DEFAULT_S)

    if 1.0 <= raw_gap <= 7.0 and abs(raw_gap - round(raw_gap)) < 0.05:
      return float(max(
        self._FOLLOW_GAP_MIN_S,
        min(
          self._FOLLOW_GAP_MAX_S,
          self._FOLLOW_GAP_MIN_S + (round(raw_gap) - 1.0) * self._FOLLOW_GAP_STALK_STEP_S,
        ),
      ))

    if 0.8 <= raw_gap <= 4.5:
      return float(max(self._FOLLOW_GAP_MIN_S, min(self._FOLLOW_GAP_MAX_S, raw_gap)))

    return float(self._FOLLOW_GAP_DEFAULT_S)


  def _set_arbitration_state(self, *, state: str, now_ms: int) -> None:
    if str(self._arbitration_state) != str(state):
      self._arbitration_state = str(state)
      self._arbitration_state_since_ms = int(now_ms)
    self._arbitration_last_update_ms = int(now_ms)


  def _reset_arbitration_state(self) -> None:
    self._arbitration_state = "INIT"
    self._arbitration_state_since_ms = 0
    self._arbitration_last_update_ms = 0
    self._arbitration_curve_target_ms = 0.0


  def _arbitration_curve_candidate(
    self,
    *,
    now_ns: int,
    reference_ms: float,
    planner_near_ms: float,
    v_ego_ms: float,
    current_angle_deg: float,
    steering_rate_deg: float,
    lp_fresh: bool,
    live_lead_context: bool,
  ) -> tuple[Optional[float], str]:
    planner_curve_active = bool(
      bool(lp_fresh)
      and float(planner_near_ms) > 0.1
      and float(planner_near_ms) < (float(reference_ms) - float(self._ARBITRATION_CURVE_MIN_DROP_MS))
    )

    raw_map_ms, raw_vision_ms = self._curve_specific_mapd_sources(now_ns=int(now_ns))
    map_supports = bool(
      raw_map_ms is not None
      and float(raw_map_ms) > 0.1
      and float(raw_map_ms) < (float(reference_ms) - float(self._ARBITRATION_CURVE_MIN_DROP_MS))
    )
    vision_supports = bool(
      raw_vision_ms is not None
      and float(raw_vision_ms) > 0.1
      and float(raw_vision_ms) < (float(reference_ms) - float(self._ARBITRATION_CURVE_MIN_DROP_MS))
    )
    steer_busy = self._steer_busy_for_curve(
      current_angle_deg=float(current_angle_deg),
      steering_rate_deg=float(steering_rate_deg),
    )

    if not (planner_curve_active or map_supports or vision_supports):
      return None, "no_curve"

    map_only_low = bool(
      map_supports
      and not vision_supports
      and (
        raw_vision_ms is None
        or float(raw_vision_ms) > (float(raw_map_ms) + float(self._ARBITRATION_CURVE_MAP_VISION_DISAGREE_MS))
      )
    )

    if map_only_low and not (planner_curve_active or steer_busy or bool(self._lat_limit_saturated) or bool(live_lead_context)):
      return None, "map_only_unconfirmed"

    curve_candidates: list[float] = []
    owner_parts: list[str] = []

    if planner_curve_active:
      curve_candidates.append(float(planner_near_ms))
      owner_parts.append("planner")

    curve_specific_ms = self._curve_specific_mapd_target_ms(
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=float(planner_near_ms) if planner_curve_active else None,
      current_angle_deg=float(current_angle_deg),
      planner_curve_active=bool(planner_curve_active),
    )
    if curve_specific_ms is not None and float(curve_specific_ms) > 0.1:
      curve_candidates.append(float(curve_specific_ms))
      owner_parts.append("mapd_comfort" if bool(self._mapd_comfort_bias_active) else "mapd")

    if not curve_candidates:
      return None, "no_curve_target"

    target_ms = float(min(curve_candidates))
    comfort_bias_ms = float(self._CURVE_CONFIRMED_COMFORT_BIAS_MS)
    if (
      raw_map_ms is not None
      and raw_vision_ms is not None
      and float(raw_vision_ms) > (float(raw_map_ms) + float(self._ARBITRATION_CURVE_MAP_VISION_DISAGREE_MS))
    ):
      comfort_bias_ms += float(self._CURVE_MAPD_VISION_DISAGREE_EXTRA_BIAS_MS)
    target_ms = min(float(reference_ms), float(target_ms) + float(comfort_bias_ms))
    if float(target_ms) > (float(reference_ms) - float(self._ARBITRATION_CURVE_MIN_DROP_MS)):
      return None, "curve_too_small"
    if float(target_ms) > (float(v_ego_ms) - float(self._ARBITRATION_CURVE_EGO_DROP_MS)) and not steer_busy:
      return None, "curve_not_urgent"

    return float(target_ms), "+".join(owner_parts) if owner_parts else "curve"


  def _arbitrate_target_state_machine(
    self,
    *,
    now_ms: int,
    now_ns: int,
    desired_ms: float,
    src: str,
    reference_ms: float,
    current_set_ms: float,
    v_ego_ms: float,
    planner_last_ms: float,
    planner_near_ms: float,
    lp_fresh: bool,
    cs_out,
  ) -> tuple[float, str]:
    current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
    steering_rate_deg = abs(float(getattr(cs_out, "steeringRateDeg", 0.0) or 0.0))
    reference_ms = max(float(reference_ms), float(current_set_ms), float(v_ego_ms), float(self.MIN_CRUISE_SPEED_MS))

    live_lead = bool(
      bool(self._lead_present)
      and float(self._lead_drel) > 0.0
      and abs(float(self._lead_yrel)) < float(self._LEAD_OFFLANE_YREL_M)
    )
    lead_time_gap_s = float(self._lead_drel) / max(float(v_ego_ms), 0.1) if live_lead else 99.0
    desired_follow_s = self._desired_follow_time_s(cs_out)
    lead_critical_gap_s = min(
      float(self._ARBITRATION_LEAD_CRITICAL_TIME_GAP_S),
      max(1.05, float(desired_follow_s) - 0.85),
    )
    lead_follow_gap_s = max(float(self._ARBITRATION_LEAD_FOLLOW_TIME_GAP_S), float(desired_follow_s))
    lead_release_gap_s = min(
      float(self._FOLLOW_GAP_MAX_S) + 0.75,
      float(desired_follow_s) + float(self._FOLLOW_GAP_RELEASE_HYSTERESIS_S),
    )
    lead_opening_clear = bool(
      live_lead
      and self._lead_is_opening_clear(base_target_ms=float(reference_ms), v_ego_ms=float(v_ego_ms))
    )
    lead_closing = bool(
      live_lead
      and (
        float(self._lead_vrel) <= float(self._ARBITRATION_LEAD_CLOSING_MS)
        or float(self._lp_a_target) <= float(self._LEAD_OPENING_RELAX_ATARGET_MS2)
      )
    )

    stale_planner_lead = self._stale_planner_lead_without_live_lead(
      now_ms=int(now_ms),
      base_target_ms=float(reference_ms),
      planner_ms=float(planner_last_ms),
      v_ego_ms=float(v_ego_ms),
    )
    planner_lead_valid = bool(bool(lp_fresh) and bool(self._lp_has_lead) and not stale_planner_lead)
    planner_floor_ms = min(float(planner_last_ms), float(planner_near_ms))
    planner_below_reference = bool(
      float(planner_floor_ms) > 0.1
      and float(planner_floor_ms) < (float(reference_ms) - float(self._ARBITRATION_LEAD_PLANNER_DROP_MS))
    )

    planner_curve_input_ms = float(planner_near_ms)
    if stale_planner_lead and not live_lead:
      planner_curve_input_ms = float(reference_ms)

    curve_candidate_ms, curve_source = self._arbitration_curve_candidate(
      now_ns=int(now_ns),
      reference_ms=float(reference_ms),
      planner_near_ms=float(planner_curve_input_ms),
      v_ego_ms=float(v_ego_ms),
      current_angle_deg=float(current_angle_deg),
      steering_rate_deg=float(steering_rate_deg),
      lp_fresh=bool(lp_fresh),
      live_lead_context=bool(live_lead or planner_lead_valid or int(now_ms) <= int(self._lead_recently_cleared_until_ms)),
    )
    curve_confirmed = curve_candidate_ms is not None

    lead_critical = bool(
      live_lead
      and not lead_opening_clear
      and (
        lead_time_gap_s <= float(lead_critical_gap_s)
        or (
          lead_closing
          and planner_below_reference
          and float(planner_floor_ms) < (float(current_set_ms) - float(self._ARBITRATION_LEAD_PLANNER_DROP_MS))
        )
        or float(self._lp_a_target) <= float(self._LEAD_APPROACH_CANCEL_STRONG_ATARGET_MS2)
      )
    )
    lead_follow = bool(
      live_lead
      and not lead_opening_clear
      and (
        lead_critical
        or lead_closing
        or lead_time_gap_s <= float(lead_follow_gap_s)
        or (
          planner_lead_valid
          and (
            planner_below_reference
            or lead_time_gap_s <= float(lead_release_gap_s)
          )
        )
      )
    )
    lead_clear_for_recovery = bool(
      live_lead
      and not lead_closing
      and (
        lead_time_gap_s >= float(lead_release_gap_s)
        or (
          lead_opening_clear
          and float(self._lead_vrel) >= float(self._FOLLOW_GAP_RELEASE_VREL_MS)
        )
      )
    )

    out_ms = float(desired_ms)
    out_src = str(src)

    if lead_critical:
      self._set_arbitration_state(state="LEAD_CRITICAL", now_ms=int(now_ms))
      self._reset_curve_hold()
      self._reset_lead_curve_hold()
      if float(planner_floor_ms) > 0.1:
        out_ms = min(float(out_ms), float(planner_floor_ms))
      if float(current_set_ms) > 0.1:
        out_ms = min(float(out_ms), float(current_set_ms))
      if lead_closing:
        lead_speed_cap_ms = max(0.0, float(v_ego_ms) + float(self._lead_vrel) + (0.5 * CV.MPH_TO_MS))
        if float(lead_speed_cap_ms) > 0.1:
          out_ms = min(float(out_ms), float(lead_speed_cap_ms))
      out_src = f"{out_src}+state[LEAD_CRITICAL]"
      return float(out_ms), out_src

    if lead_follow:
      self._set_arbitration_state(state="LEAD_FOLLOW", now_ms=int(now_ms))
      self._reset_curve_hold()
      if planner_below_reference and float(planner_floor_ms) > 0.1:
        out_ms = min(float(out_ms), float(planner_floor_ms))
      if lead_closing and float(current_set_ms) > 0.1:
        out_ms = min(float(out_ms), max(0.0, float(current_set_ms)))
      if lead_closing and ("speed_limit_target" in out_src) and ("planner[" not in out_src):
        out_ms = min(float(out_ms), max(0.0, float(current_set_ms) if float(current_set_ms) > 0.1 else float(v_ego_ms)))
        out_src = f"{out_src}+lead_flap_block_resume"
      out_src = f"{out_src}+state[LEAD_FOLLOW]"
      return float(out_ms), out_src

    no_live_lead = bool(not live_lead and not planner_lead_valid)
    stale_low_target = bool(
      no_live_lead
      and not curve_confirmed
      and float(out_ms) < (float(reference_ms) - float(self._ARBITRATION_STALE_LOW_TARGET_DROP_MS))
      and (
        "planner[lead" in out_src
        or "lead_hold" in out_src
        or "lead_present_hold" in out_src
        or "lead_curve_hold" in out_src
        or stale_planner_lead
      )
    )
    if stale_low_target:
      self._reset_lead_hold()
      self._reset_lead_curve_hold()
      self._reset_curve_hold()
      self._set_arbitration_state(state="RECOVERY", now_ms=int(now_ms))
      out_ms = float(reference_ms)
      out_src = f"{out_src}+state[RECOVERY]+stale_lead_release"
      return float(out_ms), out_src

    if curve_confirmed and float(curve_candidate_ms) < (float(reference_ms) - float(self._ARBITRATION_CURVE_MIN_DROP_MS)):
      state = "CURVE_ACTIVE" if str(self._arbitration_state).startswith("CURVE") else "CURVE_PRE_ENTRY"
      previous_update_ms = int(self._arbitration_last_update_ms)
      previous_curve_ms = float(self._arbitration_curve_target_ms) if float(self._arbitration_curve_target_ms) > 0.1 else float(out_ms)
      self._set_arbitration_state(state=state, now_ms=int(now_ms))
      target_ms = max(float(self.MIN_CRUISE_SPEED_MS), float(curve_candidate_ms))

      dt_s = 0.2
      if previous_update_ms > 0:
        dt_s = max(0.05, min(1.0, (int(now_ms) - previous_update_ms) / 1000.0))
      if target_ms < previous_curve_ms:
        target_ms = max(float(target_ms), previous_curve_ms - float(self._ARBITRATION_CURVE_ENTRY_STEP_MS) * dt_s)
      else:
        target_ms = min(float(target_ms), previous_curve_ms + float(self._ARBITRATION_CURVE_EXIT_STEP_MS) * dt_s)

      self._arbitration_curve_target_ms = float(target_ms)
      out_ms = min(float(out_ms), float(target_ms))
      out_src = f"{out_src}+state[{state}:{curve_source}]"
      return float(out_ms), out_src

    previous_curve_state = str(self._arbitration_state).startswith("CURVE")
    if previous_curve_state and no_live_lead:
      self._reset_curve_hold()
      if (int(now_ms) - int(self._arbitration_state_since_ms)) >= int(self._ARBITRATION_CURVE_EXIT_RELEASE_MS):
        self._set_arbitration_state(state="CRUISE_SYNC", now_ms=int(now_ms))
        self._arbitration_curve_target_ms = 0.0
        out_ms = max(float(out_ms), min(float(reference_ms), max(float(current_set_ms), float(v_ego_ms))))
        out_src = f"{out_src}+state[CURVE_EXIT_RELEASE]"
      else:
        self._set_arbitration_state(state="CURVE_EXIT", now_ms=int(now_ms))
        exit_target_ms = float(self._arbitration_curve_target_ms) if float(self._arbitration_curve_target_ms) > 0.1 else float(out_ms)
        exit_target_ms = min(float(reference_ms), exit_target_ms + float(self._ARBITRATION_CURVE_EXIT_STEP_MS) * 0.2)
        self._arbitration_curve_target_ms = float(exit_target_ms)
        out_ms = max(float(out_ms), float(exit_target_ms))
        out_src = f"{out_src}+state[CURVE_EXIT]"
      return float(out_ms), out_src

    if no_live_lead:
      self._reset_lead_hold()
      self._reset_lead_curve_hold()
      stale_clear_owner = bool(
        "planner[lead" in out_src
        or "lead_hold" in out_src
        or "lead_guard" in out_src
        or "lead_present_hold" in out_src
        or "lead_curve_hold" in out_src
        or stale_planner_lead
      )
      if stale_clear_owner and float(out_ms) < (float(reference_ms) - float(self._CLEAR_NO_LEAD_STALE_OWNER_DROP_MS)):
        out_ms = max(float(out_ms), min(float(reference_ms), max(float(current_set_ms), float(v_ego_ms))))
        out_src = f"{out_src}+stale_clear_release"
      if "map_only_unconfirmed" in curve_source:
        self._reset_curve_hold()
        self._mapd_stale_block_until_ms = int(now_ms) + int(self._MAPD_STRAIGHT_STALE_BLOCK_MS)
        out_ms = max(float(out_ms), min(float(reference_ms), max(float(current_set_ms), float(v_ego_ms))))
        out_src = f"{out_src}+state[CRUISE_SYNC]+map_only_unconfirmed"
      self._set_arbitration_state(state="CRUISE_SYNC", now_ms=int(now_ms))
      self._arbitration_curve_target_ms = 0.0
      return float(out_ms), out_src

    if lead_clear_for_recovery:
      self._reset_lead_hold()
      self._reset_lead_curve_hold()
      self._set_arbitration_state(state="CRUISE_SYNC", now_ms=int(now_ms))
      self._arbitration_curve_target_ms = 0.0
      if (
        ("planner[lead" in out_src or "lead_hold" in out_src or "lead_guard" in out_src)
        and float(out_ms) < (float(reference_ms) - float(self._CLEAR_NO_LEAD_STALE_OWNER_DROP_MS))
      ):
        out_ms = max(float(out_ms), min(float(reference_ms), max(float(current_set_ms), float(v_ego_ms))))
        out_src = f"{out_src}+lead_gap_release"
      return float(out_ms), f"{out_src}+state[CRUISE_SYNC]+lead_release[g={lead_time_gap_s:.1f}/tf={desired_follow_s:.1f}]"

    self._set_arbitration_state(state="LEAD_FOLLOW", now_ms=int(now_ms))
    out_ms = min(float(out_ms), max(0.0, float(current_set_ms) if float(current_set_ms) > 0.1 else float(v_ego_ms)))
    return float(out_ms), f"{out_src}+state[LEAD_FOLLOW_HOLD:g={lead_time_gap_s:.1f}/tf={desired_follow_s:.1f}]"


  def _planner_drag_reasons(self, *, now_ms: int, base_target_ms: float, planner_ms: float, current_set_ms: float, v_ego_ms: float) -> list[str]:
    if float(planner_ms) <= 0.1:
      return []
    if self._stale_planner_lead_without_live_lead(
      now_ms=int(now_ms),
      base_target_ms=float(base_target_ms),
      planner_ms=float(planner_ms),
      v_ego_ms=float(v_ego_ms),
    ):
      return []

    materially_below_base = float(planner_ms) < (float(base_target_ms) - float(self._PLANNER_DRAG_MARGIN_MS))
    materially_below_ego = float(planner_ms) < (float(v_ego_ms) - float(self._PLANNER_BELOW_EGO_MARGIN_MS))
    materially_below_clear = materially_below_base and materially_below_ego
    lead_constraining = self._lead_is_constraining(
      base_target_ms=float(base_target_ms),
      v_ego_ms=float(v_ego_ms),
    )
    lead_planner_guard = self._lead_planner_guard_active(
      base_target_ms=float(base_target_ms),
      planner_ms=float(planner_ms),
      current_set_ms=float(current_set_ms),
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
    elif lead_planner_guard:
      reasons.append("lead_guard")
    if queue_fallback_active:
      reasons.append("lp_hasLead")
    if float(self._lp_a_target) <= float(self._STRONG_DECEL_ATARGET_MS2) and (lead_constraining or lead_planner_guard or queue_fallback_active):
      reasons.append("aTarget")
    if materially_below_clear and (lead_constraining or lead_planner_guard or queue_fallback_active):
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
      self._reset_lead_curve_hold()
      self._reset_lead_curve_hold()
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      self._reset_arbitration_state()
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
      self._reset_lead_curve_hold()
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      self._reset_arbitration_state()
      self._curve_limit_guard_active = False
      self._curve_limit_guard_candidate_since_ms = 0
      self._curve_limit_guard_release_candidate_since_ms = 0
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    if not self._last_active:
      self._enabled_since_ms = int(now)
      self._stable_plan_samples = 0
      self._last_lp_seen_ns = 0
      self._reset_lead_hold()
      self._reset_lead_stuck_cancel()
      self._reset_lead_approach_cancel()
      self._reset_arbitration_state()
    self._last_active = True

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    self._poll_plan_and_lead(now_ns=now_ns, cs_out=cs_out)
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
          and not self._lead_planner_guard_active(
            base_target_ms=float(base_target_ms),
            planner_ms=float(planner_last_ms),
            current_set_ms=float(current_set_ms),
            v_ego_ms=float(v_ego_ms),
          )
        )
        drag_reasons = [] if suppress_planner_lead_owner else self._planner_drag_reasons(
          now_ms=int(now),
          base_target_ms=float(base_target_ms),
          planner_ms=float(planner_last_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lead_guard" in drag_reasons) or ("lp_hasLead" in drag_reasons)
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
          self._reset_lead_curve_hold()
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
            current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
            planner_curve_active = bool(
              lp_fresh
              and float(planner_near_ms) < (float(reference_ms) - float(self._PLANNER_CURVE_ENTRY_MARGIN_MS))
            )
            mapd_target_ms = self._mapd_curve_active_target_ms(
              now_ns=now_ns,
              reference_ms=float(reference_ms),
              planner_near_ms=float(planner_near_ms),
              current_angle_deg=float(current_angle_deg),
              planner_curve_active=planner_curve_active,
            )
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
                mapd_src = "mapd_comfort" if bool(self._mapd_comfort_bias_active) else "mapd"
                src = f"{src}+{curve_state}[{mapd_src}]"
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
          if bool(self._lead_curve_hold_active) and "lead_curve_hold" not in src:
            src = f"{src}+lead_curve_hold"

        desired_ms, lead_nibble_held = self._apply_lead_nibble_hold(
          desired_ms=float(desired_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
          base_target_ms=float(base_target_ms),
        )
        if lead_nibble_held:
          src = f"{src}+lead_nibble_hold"

        desired_ms, lead_mapd_capped = self._apply_lead_present_mapd_cap(
          now_ms=int(now),
          now_ns=int(now_ns),
          desired_ms=float(desired_ms),
          reference_ms=float(base_target_ms),
          planner_last_ms=float(planner_last_ms),
          planner_near_ms=float(planner_near_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
          current_angle_deg=abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0)),
        )
        if lead_mapd_capped:
          mapd_cap_src = "lead_comfort" if bool(self._mapd_comfort_bias_active) else "lead"
          src = f"{src}+mapd_cap[{mapd_cap_src}]"

        lead_context_curve_cap_ms, lead_context_curve_state = self._lead_context_mapd_curve_cap_ms(
          now_ms=int(now),
          now_ns=int(now_ns),
          desired_ms=float(desired_ms),
          reference_ms=float(base_target_ms),
          v_ego_ms=float(v_ego_ms),
          current_angle_deg=abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0)),
        )
        if lead_context_curve_cap_ms is not None and float(lead_context_curve_cap_ms) < (float(desired_ms) - (0.25 * CV.MPH_TO_MS)):
          desired_ms = max(float(self.MIN_CRUISE_SPEED_MS), min(float(desired_ms), float(lead_context_curve_cap_ms)))
          src = f"{src}+{lead_context_curve_state}"
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
          and not self._lead_planner_guard_active(
            base_target_ms=float(resume_ceiling_ms),
            planner_ms=float(planner_last_ms),
            current_set_ms=float(current_set_ms),
            v_ego_ms=float(v_ego_ms),
          )
        )
        drag_reasons = [] if suppress_planner_lead_owner else self._planner_drag_reasons(
          now_ms=int(now),
          base_target_ms=float(resume_ceiling_ms),
          planner_ms=float(planner_last_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
        )
        lead_owned = ("lead" in drag_reasons) or ("lead_guard" in drag_reasons) or ("lp_hasLead" in drag_reasons)
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
          self._reset_lead_curve_hold()
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

        desired_ms, lead_mapd_capped = self._apply_lead_present_mapd_cap(
          now_ms=int(now),
          now_ns=int(now_ns),
          desired_ms=float(desired_ms),
          reference_ms=float(resume_ceiling_ms),
          planner_last_ms=float(planner_last_ms),
          planner_near_ms=float(planner_near_ms),
          current_set_ms=float(current_set_ms),
          v_ego_ms=float(v_ego_ms),
          current_angle_deg=abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0)),
        )
        if lead_mapd_capped:
          mapd_cap_src = "lead_comfort" if bool(self._mapd_comfort_bias_active) else "lead"
          src = f"{src}+mapd_cap[{mapd_cap_src}]"
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

    current_angle_deg = abs(float(getattr(cs_out, "steeringAngleDeg", 0.0) or 0.0))
    steering_rate_deg = abs(float(getattr(cs_out, "steeringRateDeg", 0.0) or 0.0))
    curve_lead_context = bool(self._lead_present) or bool(self._lp_has_lead) or int(now) <= int(self._lead_recently_cleared_until_ms)
    curve_reference_ms = float(speed_limit_target_ms if (set_speed_limit_active and speed_limit_target_ms is not None) else max(float(desired_ms), float(current_set_ms), float(v_ego_ms)))
    stale_lead_curve_release = bool(
      bool(self._lead_curve_hold_active)
      and (not bool(self._lead_present))
      and (not bool(self._lp_has_lead))
      and int(now) > int(self._lead_curve_hold_until_ms)
    )
    if stale_lead_curve_release:
      self._reset_lead_curve_hold()
      self._reset_lead_hold()

    mapd_stale_release = bool(
      (not bool(self._lead_present))
      and (not bool(self._lp_has_lead))
      and bool(self._curve_hold_active)
      and float(current_set_ms) > 0.1
      and float(current_set_ms) < (float(curve_reference_ms) - float(self._MAPD_STRAIGHT_STALE_SET_DROP_MS))
      and self._mapd_straight_false_positive(
        now_ns=int(now_ns),
        reference_ms=float(curve_reference_ms),
        planner_near_ms=float(planner_near_ms),
        current_angle_deg=float(current_angle_deg),
      )
    )
    if mapd_stale_release:
      self._reset_curve_hold()
      self._mapd_stale_block_until_ms = int(now) + int(self._MAPD_STRAIGHT_STALE_BLOCK_MS)
      desired_ms = float(curve_reference_ms)
      src = f"{src}+mapd_stale_release"
    else:
      if stale_lead_curve_release:
        src = f"{src}+lead_curve_release[stale]"
      curve_force_cap_ms, curve_force_state = self._curve_force_entry_cap_ms(
        now_ms=int(now),
        now_ns=int(now_ns),
        desired_ms=float(desired_ms),
        reference_ms=float(curve_reference_ms),
        v_ego_ms=float(v_ego_ms),
        current_angle_deg=float(current_angle_deg),
        steering_rate_deg=float(steering_rate_deg),
        lead_context=bool(curve_lead_context),
      )
      if curve_force_cap_ms is not None and float(curve_force_cap_ms) < (float(desired_ms) - (0.10 * CV.MPH_TO_MS)):
        desired_ms = min(float(desired_ms), float(curve_force_cap_ms))
        src = f"{src}+{curve_force_state}"

      desired_ms, curve_accel_blocked = self._curve_accel_block_target(
        now_ns=int(now_ns),
        desired_ms=float(desired_ms),
        reference_ms=float(curve_reference_ms),
        current_set_ms=float(current_set_ms),
        v_ego_ms=float(v_ego_ms),
        current_angle_deg=float(current_angle_deg),
        steering_rate_deg=float(steering_rate_deg),
      )
      if curve_accel_blocked and "curve_accel_block[steer_busy]" not in src:
        src = f"{src}+curve_accel_block[steer_busy]"

    desired_ms, src = self._arbitrate_target_state_machine(
      now_ms=int(now),
      now_ns=int(now_ns),
      desired_ms=float(desired_ms),
      src=str(src),
      reference_ms=float(curve_reference_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
      planner_last_ms=float(planner_last_ms),
      planner_near_ms=float(planner_near_ms),
      lp_fresh=bool(lp_fresh),
      cs_out=cs_out,
    )

    desired_ms, lead_low_speed_blocked = self._low_speed_lead_block_target(
      now_ms=int(now),
      desired_ms=float(desired_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
    )
    if lead_low_speed_blocked:
      src = f"{src}+lead_low_speed_block"

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

    if stock_cruise_enabled and self._lead_approach_force_cancel_needed(
      now_ms=int(now),
      desired_ms=float(desired_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
      planner_last_ms=float(planner_last_ms),
      planner_near_ms=float(planner_near_ms),
      brake_pressed=bool(brake_pressed),
    ):
      kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
      target_u = float(desired_ms) * CV.MS_TO_KPH * kph_to_u
      current_u = float(current_set_ms) * CV.MS_TO_KPH * kph_to_u
      msg = (
        f"[XNOR_CRUISE_SYNC] src={src}+lead_approach_force_cancel uom={speed_units} "
        f"tgt={target_u:.1f} cur={current_u:.1f} est=0.0 "
        f"btn={int(CruiseButtons.CANCEL)} reason=cancel[lead_approach_force]"
      )
      self._rate_log(msg)
      return LongDecision(int(CruiseButtons.CANCEL), msg)

    if stock_cruise_enabled and self._lead_close_cancel_needed(
      now_ms=int(now),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
      brake_pressed=bool(brake_pressed),
    ):
      kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
      target_u = float(desired_ms) * CV.MS_TO_KPH * kph_to_u
      current_u = float(current_set_ms) * CV.MS_TO_KPH * kph_to_u
      msg = (
        f"[XNOR_CRUISE_SYNC] src={src}+lead_close_cancel uom={speed_units} "
        f"tgt={target_u:.1f} cur={current_u:.1f} est=0.0 "
        f"btn={int(CruiseButtons.CANCEL)} reason=cancel[lead_close]"
      )
      self._rate_log(msg)
      return LongDecision(int(CruiseButtons.CANCEL), msg)

    if stock_cruise_enabled and self._lead_approach_cancel_needed(
      now_ms=int(now),
      desired_ms=float(desired_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
      planner_last_ms=float(planner_last_ms),
      planner_near_ms=float(planner_near_ms),
      brake_pressed=bool(brake_pressed),
    ):
      kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
      target_u = float(desired_ms) * CV.MS_TO_KPH * kph_to_u
      current_u = float(current_set_ms) * CV.MS_TO_KPH * kph_to_u
      msg = (
        f"[XNOR_CRUISE_SYNC] src={src}+lead_approach_cancel uom={speed_units} "
        f"tgt={target_u:.1f} cur={current_u:.1f} est=0.0 "
        f"btn={int(CruiseButtons.CANCEL)} reason=cancel[lead_approach]"
      )
      self._rate_log(msg)
      return LongDecision(int(CruiseButtons.CANCEL), msg)

    if stock_cruise_enabled and self._lead_stuck_cancel_needed(
      now_ms=int(now),
      desired_ms=float(desired_ms),
      current_set_ms=float(current_set_ms),
      v_ego_ms=float(v_ego_ms),
      planner_last_ms=float(planner_last_ms),
      planner_near_ms=float(planner_near_ms),
      brake_pressed=bool(brake_pressed),
    ):
      kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
      target_u = float(desired_ms) * CV.MS_TO_KPH * kph_to_u
      current_u = float(current_set_ms) * CV.MS_TO_KPH * kph_to_u
      msg = (
        f"[XNOR_CRUISE_SYNC] src={src}+lead_stuck_cancel uom={speed_units} "
        f"tgt={target_u:.1f} cur={current_u:.1f} est=0.0 "
        f"btn={int(CruiseButtons.CANCEL)} reason=cancel[lead_stuck]"
      )
      self._rate_log(msg)
      return LongDecision(int(CruiseButtons.CANCEL), msg)

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
