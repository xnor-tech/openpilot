"""/data/openpilot/selfdrive/car/modules/ALC_module.py

Unity-style Auto Lane Change helper for xnor.

xnor lane change state lives in `modelV2.meta` (preferred). Some builds expose `lateralPlan`.
This module supports both schemas.
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

from cereal import log


def _lane_change_enums() -> Tuple[Any, Any]:
  state = getattr(log, "LaneChangeState", None)
  direction = getattr(log, "LaneChangeDirection", None)
  if state is None or direction is None:
    lp = getattr(log, "LateralPlan", None)
    state = getattr(lp, "LaneChangeState", None)
    direction = getattr(lp, "LaneChangeDirection", None)
  return state, direction


LaneChangeState, LaneChangeDirection = _lane_change_enums()


def _extract_lane_change(msg: Any) -> Tuple[Optional[Any], Optional[Any]]:
  if msg is None:
    return None, None

  model = getattr(msg, "modelV2", None)
  if model is None and getattr(msg, "meta", None) is not None:
    model = msg
  if model is not None and getattr(model, "meta", None) is not None:
    meta = model.meta
    return meta.laneChangeState, meta.laneChangeDirection

  lat_plan = getattr(msg, "lateralPlan", None)
  if lat_plan is None and (getattr(msg, "laneChangeState", None) is not None):
    lat_plan = msg
  if lat_plan is not None:
    return lat_plan.laneChangeState, lat_plan.laneChangeDirection

  return None, None


class ALCController:
  def __init__(self) -> None:
    self._pre_lc_start_frame = 0

  def update(self, enabled: bool, CS: Any, frame: int, plan_msg: Any) -> None:
    if not enabled or not getattr(CS, "enableALC", False):
      CS.alca_pre_engage = False
      CS.prev_alca_pre_engage = False
      CS.alca_engaged = False
      CS.alca_done = False
      CS.alca_need_engagement = False
      CS.alca_direction = 0
      self._pre_lc_start_frame = 0
      return

    lc_state, lc_dir = _extract_lane_change(plan_msg)
    if LaneChangeState is None or LaneChangeDirection is None or lc_state is None:
      CS.alca_pre_engage = False
      CS.alca_engaged = False
      CS.alca_done = False
      CS.alca_need_engagement = False
      CS.alca_direction = 0
      return

    CS.alca_pre_engage = lc_state in (LaneChangeState.preLaneChange,)
    CS.alca_engaged = lc_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    CS.alca_done = lc_state in (LaneChangeState.laneChangeFinishing,)

    if (CS.alca_pre_engage or CS.alca_engaged) and not CS.alca_done:
      if lc_dir == LaneChangeDirection.left:
        CS.alca_direction = 1
      elif lc_dir == LaneChangeDirection.right:
        CS.alca_direction = 2
      else:
        CS.alca_direction = 0
    else:
      CS.alca_direction = 0

    if CS.alca_pre_engage:
      if CS.alca_pre_engage != getattr(CS, "prev_alca_pre_engage", False):
        self._pre_lc_start_frame = frame

      delay_s = float(getattr(CS, "autoStartAlcaDelay", 0.0) or 0.0)
      CS.alca_need_engagement = (
        delay_s > 0.0 and self._pre_lc_start_frame > 0 and
        (frame - self._pre_lc_start_frame) > int(delay_s * 100)
      )
    else:
      CS.alca_need_engagement = False
      self._pre_lc_start_frame = frame

    CS.prev_alca_pre_engage = bool(CS.alca_pre_engage)
