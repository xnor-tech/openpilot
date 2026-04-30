#!/usr/bin/env python3
# /data/openpilot/tools/clear_path_watch.py
from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from cereal import messaging

RUNNING = True


def _sig_handler(signum, frame) -> None:
  del signum, frame
  global RUNNING
  RUNNING = False


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _wall_iso() -> str:
  return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def _safe_float(value: Any, default: float = 0.0) -> float:
  try:
    v = float(value)
  except Exception:
    return float(default)
  return float(v) if math.isfinite(v) else float(default)


def _safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return int(default)


def _safe_bool(value: Any) -> bool:
  try:
    return bool(value)
  except Exception:
    return False


def _safe_str(value: Any, default: str = "") -> str:
  if value is None:
    return default
  try:
    return str(value)
  except Exception:
    return default


def _take_numeric(seq: Any, limit: int) -> list[float]:
  out: list[float] = []
  try:
    items = list(seq)[:limit] if seq is not None else []
  except Exception:
    return out
  for item in items:
    v = _safe_float(item, float("nan"))
    if math.isfinite(v):
      out.append(float(v))
  return out


def _maybe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _enum_name(value: Any) -> str:
  try:
    name = getattr(value, "name")
    if name is not None:
      return str(name)
  except Exception:
    pass
  try:
    return str(value).split(".")[-1]
  except Exception:
    return ""


def _button_events_summary(cs: Any) -> list[dict[str, Any]]:
  out: list[dict[str, Any]] = []
  try:
    events = list(_maybe_attr(cs, "buttonEvents", []) or [])
  except Exception:
    return out
  for event in events[-12:]:
    event_type = _maybe_attr(event, "type", None)
    out.append({
      "type": _enum_name(event_type),
      "pressed": _safe_bool(_maybe_attr(event, "pressed", False)),
      "rawType": _safe_str(event_type),
    })
  return out


def _interesting_named_fields(obj: Any, tokens: tuple[str, ...]) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if obj is None:
    return out

  try:
    raw = obj.to_dict()
  except Exception:
    raw = None
  if isinstance(raw, dict):
    for key, value in raw.items():
      key_s = str(key)
      if any(token in key_s.lower() for token in tokens):
        if isinstance(value, (bool, int, float, str)):
          out[key_s] = value
    return out

  for name in dir(obj):
    if name.startswith("_"):
      continue
    name_l = name.lower()
    if not any(token in name_l for token in tokens):
      continue
    try:
      value = getattr(obj, name)
    except Exception:
      continue
    if callable(value):
      continue
    if isinstance(value, (bool, int, float, str)):
      out[str(name)] = value
  return out


def _lead_summary(lead: Any) -> dict[str, Any]:
  if lead is None:
    return {"status": False}
  return {
    "status": _safe_bool(_maybe_attr(lead, "status", False)),
    "dRel": _safe_float(_maybe_attr(lead, "dRel", 0.0)),
    "yRel": _safe_float(_maybe_attr(lead, "yRel", 0.0)),
    "vRel": _safe_float(_maybe_attr(lead, "vRel", 0.0)),
    "aRel": _safe_float(_maybe_attr(lead, "aRel", 0.0)),
    "vLead": _safe_float(_maybe_attr(lead, "vLead", 0.0)),
    "vLeadK": _safe_float(_maybe_attr(lead, "vLeadK", 0.0)),
    "aLeadK": _safe_float(_maybe_attr(lead, "aLeadK", 0.0)),
    "dPath": _safe_float(_maybe_attr(lead, "dPath", 0.0)),
    "vLat": _safe_float(_maybe_attr(lead, "vLat", 0.0)),
    "aLeadTau": _safe_float(_maybe_attr(lead, "aLeadTau", 0.0)),
    "modelProb": _safe_float(_maybe_attr(lead, "modelProb", 0.0)),
    "prob": _safe_float(_maybe_attr(lead, "prob", 0.0)),
    "radar": _safe_bool(_maybe_attr(lead, "radar", False)),
    "fcw": _safe_bool(_maybe_attr(lead, "fcw", False)),
    "source": _safe_str(_maybe_attr(lead, "source", "")),
    "trackId": _safe_int(_maybe_attr(lead, "trackId", -1), -1),
  }


def _car_state_summary(cs: Any) -> dict[str, Any]:
  cruise = _maybe_attr(cs, "cruiseState", None)
  out = {
    "vEgo": _safe_float(_maybe_attr(cs, "vEgo", 0.0)),
    "aEgo": _safe_float(_maybe_attr(cs, "aEgo", 0.0)),
    "standstill": _safe_bool(_maybe_attr(cs, "standstill", False)),
    "gasPressed": _safe_bool(_maybe_attr(cs, "gasPressed", False)),
    "brakePressed": _safe_bool(_maybe_attr(cs, "brakePressed", False)),
    "steeringAngleDeg": _safe_float(_maybe_attr(cs, "steeringAngleDeg", 0.0)),
    "steeringRateDeg": _safe_float(_maybe_attr(cs, "steeringRateDeg", 0.0)),
    "steeringTorque": _safe_float(_maybe_attr(cs, "steeringTorque", 0.0)),
    "steeringTorqueEps": _safe_float(_maybe_attr(cs, "steeringTorqueEps", 0.0)),
    "steeringPressed": _safe_bool(_maybe_attr(cs, "steeringPressed", False)),
    "yawRate": _safe_float(_maybe_attr(cs, "yawRate", 0.0)),
    "leftBlinker": _safe_bool(_maybe_attr(cs, "leftBlinker", False)),
    "rightBlinker": _safe_bool(_maybe_attr(cs, "rightBlinker", False)),
    "speedLimit": _safe_float(_maybe_attr(cs, "speedLimit", 0.0)),
    "speedLimitOffset": _safe_float(_maybe_attr(cs, "speedLimitOffset", 0.0)),
    "cruiseButtons": _safe_int(_maybe_attr(cs, "cruiseButtons", 0), 0),
    "cruiseButtonsCounter": _safe_int(_maybe_attr(cs, "cruiseButtonsCounter", 0), 0),
    "followDistance": _safe_float(_maybe_attr(cs, "followDistance", 0.0)),
    "cruiseGap": _safe_float(_maybe_attr(cs, "cruiseGap", 0.0)),
    "distanceSetting": _safe_float(_maybe_attr(cs, "distanceSetting", 0.0)),
    "accDistance": _safe_float(_maybe_attr(cs, "accDistance", 0.0)),
    "stockFollowDistance": _safe_float(_maybe_attr(cs, "stockFollowDistance", 0.0)),
    "teslaFollowDistance": _safe_float(_maybe_attr(cs, "teslaFollowDistance", 0.0)),
    "gapAdjustCruiseTr": _safe_float(_maybe_attr(cs, "gapAdjustCruiseTr", 0.0)),
    "accFollowDistance": _safe_float(_maybe_attr(cs, "accFollowDistance", 0.0)),
    "followTime": _safe_float(_maybe_attr(cs, "followTime", 0.0)),
    "followTimeGap": _safe_float(_maybe_attr(cs, "followTimeGap", 0.0)),
    "modeSel": _safe_float(_maybe_attr(cs, "modeSel", 0.0)),
    "accMode": _safe_float(_maybe_attr(cs, "accMode", 0.0)),
    "gapSetting": _safe_float(_maybe_attr(cs, "gapSetting", 0.0)),
    "timeGap": _safe_float(_maybe_attr(cs, "timeGap", 0.0)),
    "apFollowDistance": _safe_float(_maybe_attr(cs, "apFollowDistance", 0.0)),
    "apFollowTime": _safe_float(_maybe_attr(cs, "apFollowTime", 0.0)),
    "followDistanceStock": _safe_float(_maybe_attr(cs, "followDistanceStock", 0.0)),
    "stockGap": _safe_float(_maybe_attr(cs, "stockGap", 0.0)),
    "teslaGap": _safe_float(_maybe_attr(cs, "teslaGap", 0.0)),
    "dasFollowDistance": _safe_float(_maybe_attr(cs, "dasFollowDistance", 0.0)),
    "dasFollowTime": _safe_float(_maybe_attr(cs, "dasFollowTime", 0.0)),
    "DAS_followDistance": _safe_float(_maybe_attr(cs, "DAS_followDistance", 0.0)),
    "DAS_timeGap": _safe_float(_maybe_attr(cs, "DAS_timeGap", 0.0)),
    "cruiseFollowDistance": _safe_float(_maybe_attr(cs, "cruiseFollowDistance", 0.0)),
    "cruiseTimeGap": _safe_float(_maybe_attr(cs, "cruiseTimeGap", 0.0)),
    "longitudinalControlGap": _safe_float(_maybe_attr(cs, "longitudinalControlGap", 0.0)),
    "buttonEvents": _button_events_summary(cs),
    "rawGapFields": _interesting_named_fields(cs, ("gap", "follow", "distance", "timegap", "modesel")),
  }
  if cruise is not None:
    out["cruiseState"] = {
      "enabled": _safe_bool(_maybe_attr(cruise, "enabled", False)),
      "available": _safe_bool(_maybe_attr(cruise, "available", False)),
      "standstill": _safe_bool(_maybe_attr(cruise, "standstill", False)),
      "speed": _safe_float(_maybe_attr(cruise, "speed", 0.0)),
      "speedCluster": _safe_float(_maybe_attr(cruise, "speedCluster", 0.0)),
      "modeSel": _safe_float(_maybe_attr(cruise, "modeSel", 0.0)),
      "gap": _safe_float(_maybe_attr(cruise, "gap", 0.0)),
      "followDistance": _safe_float(_maybe_attr(cruise, "followDistance", 0.0)),
      "distanceSetting": _safe_float(_maybe_attr(cruise, "distanceSetting", 0.0)),
      "timeGap": _safe_float(_maybe_attr(cruise, "timeGap", 0.0)),
      "followTime": _safe_float(_maybe_attr(cruise, "followTime", 0.0)),
      "cruiseGap": _safe_float(_maybe_attr(cruise, "cruiseGap", 0.0)),
      "gapSetting": _safe_float(_maybe_attr(cruise, "gapSetting", 0.0)),
      "accDistance": _safe_float(_maybe_attr(cruise, "accDistance", 0.0)),
      "rawGapFields": _interesting_named_fields(cruise, ("gap", "follow", "distance", "timegap", "modesel")),
    }
  return out


def _controls_state_summary(cs: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  for name in (
    "enabled",
    "active",
    "vCruise",
    "vCruiseCluster",
    "longControlState",
    "forceDecel",
    "alertSize",
    "alertStatus",
  ):
    value = _maybe_attr(cs, name, None)
    if isinstance(value, (bool, int, str)):
      out[name] = value
    elif value is not None:
      out[name] = _safe_float(value, 0.0)

  lateral_state = _maybe_attr(cs, "lateralControlState", None)
  if lateral_state is not None:
    state_name = ""
    try:
      state_name = lateral_state.which()
    except Exception:
      state_name = ""
    out["lateralStateName"] = _safe_str(state_name)
    if state_name:
      lat_state = _maybe_attr(lateral_state, state_name, None)
      if lat_state is not None:
        out["lateralState"] = {
          "saturated": _safe_bool(_maybe_attr(lat_state, "saturated", False)),
          "steeringAngleDeg": _safe_float(_maybe_attr(lat_state, "steeringAngleDeg", 0.0)),
          "output": _safe_float(_maybe_attr(lat_state, "output", 0.0)),
          "desiredCurvature": _safe_float(_maybe_attr(lat_state, "desiredCurvature", 0.0)),
          "actualCurvature": _safe_float(_maybe_attr(lat_state, "actualCurvature", 0.0)),
        }
  return out


def _plan_summary(lp: Any) -> dict[str, Any]:
  speeds = _take_numeric(_maybe_attr(lp, "speeds", None), 33)
  accels = _take_numeric(_maybe_attr(lp, "accels", None), 16)
  jerks = _take_numeric(_maybe_attr(lp, "jerks", None), 16)

  p_last = float(speeds[-1]) if speeds else None
  near_window = max(3, int(math.ceil(len(speeds) * 0.35))) if speeds else 0
  preview_window = max(5, int(math.ceil(len(speeds) * 0.70))) if speeds else 0
  p_near = min(speeds[:near_window]) if near_window > 0 else None
  p_preview = min(speeds[:preview_window]) if preview_window > 0 else None

  return {
    "hasLead": _safe_bool(_maybe_attr(lp, "hasLead", False)),
    "aTarget": _safe_float(_maybe_attr(lp, "aTarget", 0.0)),
    "vTarget": _safe_float(_maybe_attr(lp, "vTarget", 0.0)),
    "vCruise": _safe_float(_maybe_attr(lp, "vCruise", 0.0)),
    "vCruiseCluster": _safe_float(_maybe_attr(lp, "vCruiseCluster", 0.0)),
    "desiredTF": _safe_float(_maybe_attr(lp, "desiredFollowDistance", 0.0)),
    "longitudinalPlanSource": _safe_str(_maybe_attr(lp, "longitudinalPlanSource", "")),
    "xState": _safe_str(_maybe_attr(lp, "xState", "")),
    "trafficState": _safe_str(_maybe_attr(lp, "trafficState", "")),
    "fcw": _safe_bool(_maybe_attr(lp, "fcw", False)),
    "speeds": speeds,
    "accels": accels,
    "jerks": jerks,
    "p_last": p_last,
    "p_near": p_near,
    "p_preview": p_preview,
  }


def _mapd_summary(mo: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  for name in (
    "suggestedSpeed",
    "speedLimit",
    "nextSpeedLimit",
    "distToSpeedLimit",
    "curveSpeed",
    "visionCurveSpeed",
    "mapCurveSpeed",
  ):
    value = _maybe_attr(mo, name, None)
    if value is None:
      continue
    out[name] = _safe_float(value, 0.0)

  for name in ("speedLimitControlState", "source"):
    value = _maybe_attr(mo, name, None)
    if value is not None:
      out[name] = _safe_str(value)
  return out


def _first_nonzero_float(*values: Any) -> float:
  for value in values:
    out = _safe_float(value, 0.0)
    if abs(out) > 0.001:
      return float(out)
  return 0.0


def _follow_gap_summary(car: dict[str, Any], plan: dict[str, Any], lead1: dict[str, Any], lead2: dict[str, Any]) -> dict[str, Any]:
  cruise = car.get("cruiseState", {}) if isinstance(car.get("cruiseState"), dict) else {}
  primary = lead1 if bool(lead1.get("status")) else lead2
  v_ego = _safe_float(car.get("vEgo"), 0.0)
  d_rel = _safe_float(primary.get("dRel"), 0.0)
  actual_gap_s = d_rel / max(v_ego, 0.1) if d_rel > 0.1 and v_ego > 0.1 else 0.0

  raw_candidates = {
    "followDistance": _safe_float(car.get("followDistance"), 0.0),
    "cruiseGap": _safe_float(car.get("cruiseGap"), 0.0),
    "distanceSetting": _safe_float(car.get("distanceSetting"), 0.0),
    "accDistance": _safe_float(car.get("accDistance"), 0.0),
    "stockFollowDistance": _safe_float(car.get("stockFollowDistance"), 0.0),
    "teslaFollowDistance": _safe_float(car.get("teslaFollowDistance"), 0.0),
    "gapAdjustCruiseTr": _safe_float(car.get("gapAdjustCruiseTr"), 0.0),
    "accFollowDistance": _safe_float(car.get("accFollowDistance"), 0.0),
    "followTime": _safe_float(car.get("followTime"), 0.0),
    "followTimeGap": _safe_float(car.get("followTimeGap"), 0.0),
    "modeSel": _safe_float(car.get("modeSel"), 0.0),
    "accMode": _safe_float(car.get("accMode"), 0.0),
    "gapSetting": _safe_float(car.get("gapSetting"), 0.0),
    "timeGap": _safe_float(car.get("timeGap"), 0.0),
    "apFollowDistance": _safe_float(car.get("apFollowDistance"), 0.0),
    "apFollowTime": _safe_float(car.get("apFollowTime"), 0.0),
    "followDistanceStock": _safe_float(car.get("followDistanceStock"), 0.0),
    "stockGap": _safe_float(car.get("stockGap"), 0.0),
    "teslaGap": _safe_float(car.get("teslaGap"), 0.0),
    "dasFollowDistance": _safe_float(car.get("dasFollowDistance"), 0.0),
    "dasFollowTime": _safe_float(car.get("dasFollowTime"), 0.0),
    "DAS_followDistance": _safe_float(car.get("DAS_followDistance"), 0.0),
    "DAS_timeGap": _safe_float(car.get("DAS_timeGap"), 0.0),
    "cruiseFollowDistance": _safe_float(car.get("cruiseFollowDistance"), 0.0),
    "cruiseTimeGap": _safe_float(car.get("cruiseTimeGap"), 0.0),
    "longitudinalControlGap": _safe_float(car.get("longitudinalControlGap"), 0.0),
    "cruiseState.gap": _safe_float(cruise.get("gap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.followDistance": _safe_float(cruise.get("followDistance") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.distanceSetting": _safe_float(cruise.get("distanceSetting") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.modeSel": _safe_float(cruise.get("modeSel") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.timeGap": _safe_float(cruise.get("timeGap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.followTime": _safe_float(cruise.get("followTime") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.cruiseGap": _safe_float(cruise.get("cruiseGap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.gapSetting": _safe_float(cruise.get("gapSetting") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.accDistance": _safe_float(cruise.get("accDistance") if isinstance(cruise, dict) else 0.0, 0.0),
  }
  for key, value in (car.get("rawGapFields") or {}).items():
    raw_candidates[f"raw.{key}"] = _safe_float(value, 0.0)
  if isinstance(cruise, dict):
    for key, value in (cruise.get("rawGapFields") or {}).items():
      raw_candidates[f"cruiseState.raw.{key}"] = _safe_float(value, 0.0)
  active_name = ""
  stalk_gap = 0.0
  for name, value in raw_candidates.items():
    if abs(float(value)) > 0.001:
      active_name = str(name)
      stalk_gap = float(value)
      break

  button_events = car.get("buttonEvents", []) if isinstance(car.get("buttonEvents"), list) else []
  gap_button_events = [
    event for event in button_events
    if any(token in _safe_str(event.get("type", "")).lower() for token in ("gap", "distance", "follow"))
  ]

  return {
    "desiredTF": _safe_float(plan.get("desiredTF"), 0.0),
    "actualGapS": actual_gap_s,
    "dRel": d_rel,
    "stalkGap": stalk_gap,
    "stalkGapField": active_name,
    "rawCandidates": raw_candidates,
    "gapButtonEvents": gap_button_events[-6:],
    "buttonEvents": button_events[-8:],
    "cruiseButtons": _safe_int(car.get("cruiseButtons"), 0),
    "cruiseButtonsCounter": _safe_int(car.get("cruiseButtonsCounter"), 0),
  }



def _model_summary(model: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  leads_out: list[dict[str, Any]] = []
  try:
    leads = list(_maybe_attr(model, "leadsV3", []) or [])[:2]
  except Exception:
    leads = []

  for lead in leads:
    probs = _take_numeric(_maybe_attr(lead, "prob", None), 3)
    xs = _take_numeric(_maybe_attr(lead, "x", None), 3)
    ys = _take_numeric(_maybe_attr(lead, "y", None), 3)
    vs = _take_numeric(_maybe_attr(lead, "v", None), 3)
    leads_out.append({
      "prob": probs[0] if probs else None,
      "x": xs[0] if xs else None,
      "y": ys[0] if ys else None,
      "v": vs[0] if vs else None,
    })

  lane_lines: list[dict[str, Any]] = []
  try:
    lines = list(_maybe_attr(model, "laneLines", []) or [])[:4]
  except Exception:
    lines = []
  try:
    line_probs = list(_maybe_attr(model, "laneLineProbs", []) or [])[:4]
  except Exception:
    line_probs = []

  for i, line in enumerate(lines):
    xs = _take_numeric(_maybe_attr(line, "x", None), 3)
    ys = _take_numeric(_maybe_attr(line, "y", None), 3)
    lane_lines.append({
      "prob": _safe_float(line_probs[i], 0.0) if i < len(line_probs) else 0.0,
      "x0": xs[0] if xs else None,
      "y0": ys[0] if ys else None,
    })

  position = _maybe_attr(model, "position", None)
  out["position"] = {
    "x0": (_take_numeric(_maybe_attr(position, "x", None), 1) or [None])[0],
    "y0": (_take_numeric(_maybe_attr(position, "y", None), 1) or [None])[0],
  }
  out["leadsV3"] = leads_out
  out["laneLines"] = lane_lines
  return out


def _derive_flags(*, car: dict[str, Any], plan: dict[str, Any], lead1: dict[str, Any], lead2: dict[str, Any], long_log: dict[str, Any] | None) -> dict[str, Any]:
  cruise = car.get("cruiseState", {}) if isinstance(car.get("cruiseState"), dict) else {}
  current_set = _safe_float(cruise.get("speed"), 0.0)
  v_ego = _safe_float(car.get("vEgo"), 0.0)
  p_near = plan.get("p_near")
  p_preview = plan.get("p_preview")
  a_target = _safe_float(plan.get("aTarget"), 0.0)
  plan_drop = False
  if isinstance(p_near, (float, int)) and current_set > 0.0:
    plan_drop = float(p_near) < (current_set - 1.0)
  no_actual_lead = not bool(lead1.get("status")) and not bool(lead2.get("status"))
  long_src = _safe_str((long_log or {}).get("src", ""))
  vrels = []
  for val in (_safe_float(lead1.get("vRel"), 0.0), _safe_float(lead2.get("vRel"), 0.0)):
    if val != 0.0:
      vrels.append(val)
  primary_vrel = min(vrels) if vrels else 0.0
  follow_gap = _follow_gap_summary(car, plan, lead1, lead2)
  return {
    "clearRoadCandidate": bool(no_actual_lead and not _safe_bool(car.get("brakePressed", False)) and v_ego > 4.0),
    "plannerDropVsSet": bool(plan_drop),
    "longHasPlannerOwner": ("planner" in long_src),
    "longHasCurveOwner": ("curve_hold" in long_src or "hard_entry" in long_src),
    "longSource": long_src,
    "currentSet": current_set,
    "vEgo": v_ego,
    "pNear": p_near,
    "pPreview": p_preview,
    "followGap": follow_gap,
    "closingLead": bool((lead1.get("status") or lead2.get("status")) and (primary_vrel < -0.5 or a_target < -0.15)),
    "steeringBusy": bool(abs(_safe_float(car.get("steeringAngleDeg"), 0.0)) > 8.0 or abs(_safe_float(car.get("steeringRateDeg"), 0.0)) > 25.0),
  }


class FlapTracker:
  def __init__(self) -> None:
    self.last_primary = "none"
    self.last_present = False
    self.last_status_change_ms: int | None = None
    self.primary_switches = 0
    self.presence_toggles = 0
    self.dropout_short = 0
    self.last_dropout_ms: int | None = None

  def update(self, mono_ms: int, lead1: dict[str, Any], lead2: dict[str, Any]) -> dict[str, Any]:
    primary = "lead1" if lead1.get("status") else ("lead2" if lead2.get("status") else "none")
    present = primary != "none"

    if primary != self.last_primary:
      self.primary_switches += 1
      if self.last_primary != "none" and primary == "none":
        self.last_dropout_ms = mono_ms
      elif self.last_primary == "none" and primary != "none" and self.last_dropout_ms is not None:
        gap_ms = mono_ms - self.last_dropout_ms
        if gap_ms <= 1200:
          self.dropout_short += 1
      self.last_primary = primary

    if present != self.last_present:
      self.presence_toggles += 1
      self.last_status_change_ms = mono_ms
      self.last_present = present

    return {
      "primary": primary,
      "present": present,
      "primarySwitches": self.primary_switches,
      "presenceToggles": self.presence_toggles,
      "shortDropouts": self.dropout_short,
      "sinceStatusChangeMs": 0 if self.last_status_change_ms is None else max(0, mono_ms - self.last_status_change_ms),
    }


class SwaglogTail:
  KEYWORDS = (
    "[XNOR_CRUISE_SYNC]",
    "lead_guard",
    "lead_stuck_cancel",
    "lead_approach_force",
    "autoengage_mismatch_reset",
    "autoengage_stale_block",
    "lead_flap_block_resume",
    "lead_takeover_after_autoengage",
    "mapd_cap",
    "mapd_comfort",
  )

  def __init__(self) -> None:
    self._lock = threading.Lock()
    self._recent: deque[dict[str, Any]] = deque(maxlen=80)
    self._positions: dict[str, int] = {}
    self._thread = threading.Thread(target=self._run, daemon=True)

  def start(self) -> None:
    self._thread.start()

  def recent(self) -> list[dict[str, Any]]:
    with self._lock:
      return list(self._recent)

  def latest(self) -> dict[str, Any] | None:
    with self._lock:
      return self._recent[-1] if self._recent else None

  def _log_files(self) -> list[Path]:
    out: list[Path] = []
    folder = Path("/data/log")
    if not folder.exists():
      return out
    try:
      for path in sorted(folder.glob("swaglog*")):
        if path.is_file():
          out.append(path)
    except Exception:
      pass
    return out

  def _run(self) -> None:
    while RUNNING:
      try:
        for path in self._log_files():
          self._consume_file(path)
      except Exception:
        pass
      time.sleep(0.2)

  def _consume_file(self, path: Path) -> None:
    key = str(path)
    try:
      current_size = path.stat().st_size
    except Exception:
      return

    pos = self._positions.get(key, 0)
    if pos > current_size:
      pos = 0

    try:
      with path.open("r", encoding="utf-8", errors="ignore") as f:
        f.seek(pos)
        while True:
          line = f.readline()
          if not line:
            break
          pos = f.tell()
          self._process_line(line.rstrip("\n"))
    except Exception:
      return

    self._positions[key] = pos

  def _process_line(self, line: str) -> None:
    if not any(keyword in line for keyword in self.KEYWORDS):
      return

    raw_message = line
    created = None
    filename = ""
    module = ""
    msg_s = line

    try:
      payload = json.loads(line)
      raw_message = line
      created = _safe_float(payload.get("created", 0.0), 0.0)
      filename = _safe_str(payload.get("filename", ""))
      module = _safe_str(payload.get("module", ""))
      msg_s = _safe_str(payload.get("msg$s", line))
    except Exception:
      pass

    record = {
      "created": created,
      "filename": filename,
      "module": module,
      "message": msg_s,
      "raw": raw_message,
    }

    msg = msg_s
    for token in msg.split():
      if "=" not in token:
        continue
      k, v = token.split("=", 1)
      if k in ("src", "uom", "reason", "mode", "state"):
        record[k] = v
      elif k in ("tgt", "cur", "est", "gap", "delta", "cooldown", "stuck", "vrel", "drel"):
        record[k] = _safe_float(v, 0.0)
      elif k == "btn":
        record[k] = _safe_int(v, 0)

    with self._lock:
      self._recent.append(record)


def _write_summary_line(txt_f, record: dict[str, Any]) -> None:
  flags = record["derived"]
  long_log = record.get("long_log") or {}
  flap = record.get("leadState", {})
  follow = flags.get("followGap", {}) if isinstance(flags.get("followGap"), dict) else {}
  line = (
    f"{record['wall_time']} "
    f"vEgo={flags['vEgo']:.2f} "
    f"set={flags['currentSet']:.2f} "
    f"clear={int(flags['clearRoadCandidate'])} "
    f"plannerDrop={int(flags['plannerDropVsSet'])} "
    f"closing={int(flags['closingLead'])} "
    f"steerBusy={int(flags['steeringBusy'])} "
    f"lead1={int(record['radarState']['leadOne'].get('status', False))} "
    f"lead2={int(record['radarState']['leadTwo'].get('status', False))} "
    f"leadSel={_safe_str(flap.get('primary', '-'))} "
    f"toggles={_safe_int(flap.get('presenceToggles', 0))} "
    f"gapS={_safe_float(follow.get('actualGapS'), 0.0):.2f} "
    f"desiredTF={_safe_float(follow.get('desiredTF'), 0.0):.2f} "
    f"stalkGap={_safe_float(follow.get('stalkGap'), 0.0):.1f} "
    f"stalkField={_safe_str(follow.get('stalkGapField'), '-') or '-'} "
    f"gapBtn={','.join(_safe_str(e.get('type'), '') for e in (follow.get('gapButtonEvents') or [])[-2:]) or '-'} "
    f"btnEvt={','.join(_safe_str(e.get('type'), '') for e in (follow.get('buttonEvents') or [])[-3:]) or '-'} "
    f"src={flags['longSource'] or '-'} "
    f"tgt={_safe_float(long_log.get('tgt'), 0.0):.2f} "
    f"cur={_safe_float(long_log.get('cur'), 0.0):.2f} "
    f"btn={_safe_int(long_log.get('btn'), 0)} "
    f"reason={_safe_str(long_log.get('reason'), '-')}"
  )
  txt_f.write(line + "\n")
  txt_f.flush()


def main() -> int:
  parser = argparse.ArgumentParser(description="Watch clear-road handoff, lead stability, and LONG owner output.")
  parser.add_argument("duration_s", nargs="?", type=int, default=300, help="Run duration in seconds")
  parser.add_argument("--interval-ms", type=int, default=100, help="Sampling interval in milliseconds")
  args = parser.parse_args()

  signal.signal(signal.SIGINT, _sig_handler)
  signal.signal(signal.SIGTERM, _sig_handler)

  sm = messaging.SubMaster([
    "carState",
    "controlsState",
    "radarState",
    "longitudinalPlan",
    "modelV2",
    "mapdOut",
  ])

  tail = SwaglogTail()
  tail.start()
  flap_tracker = FlapTracker()

  ts = datetime.now().strftime("%Y%m%d_%H%M%S")
  out_dir = Path("/data")
  jsonl_path = out_dir / f"clear_path_watch_{ts}.jsonl"
  txt_path = out_dir / f"clear_path_watch_{ts}.txt"

  started = _now_ms()
  next_sample_ms = started

  with jsonl_path.open("w", encoding="utf-8") as jsonl_f, txt_path.open("w", encoding="utf-8") as txt_f:
    txt_f.write("# clear_path_watch\n")
    txt_f.write(f"# started { _wall_iso() }\n")

    while RUNNING and (_now_ms() - started) < (int(args.duration_s) * 1000):
      sm.update(0)
      now_ms = _now_ms()
      if now_ms < next_sample_ms:
        time.sleep(0.01)
        continue
      next_sample_ms = now_ms + int(args.interval_ms)

      car = _car_state_summary(sm["carState"]) if sm.seen["carState"] else {}
      controls = _controls_state_summary(sm["controlsState"]) if sm.seen["controlsState"] else {}
      radar = {
        "leadOne": _lead_summary(_maybe_attr(sm["radarState"], "leadOne", None)) if sm.seen["radarState"] else {"status": False},
        "leadTwo": _lead_summary(_maybe_attr(sm["radarState"], "leadTwo", None)) if sm.seen["radarState"] else {"status": False},
      }
      plan = _plan_summary(sm["longitudinalPlan"]) if sm.seen["longitudinalPlan"] else {}
      model = _model_summary(sm["modelV2"]) if sm.seen["modelV2"] else {}
      mapd = _mapd_summary(sm["mapdOut"]) if sm.seen["mapdOut"] else {}
      long_log = tail.latest()
      lead_state = flap_tracker.update(now_ms, radar["leadOne"], radar["leadTwo"])

      record = {
        "wall_time": _wall_iso(),
        "mono_ms": now_ms,
        "valid": {k: _safe_bool(v) for k, v in sm.valid.items()},
        "alive": {k: _safe_bool(v) for k, v in sm.alive.items()},
        "carState": car,
        "controlsState": controls,
        "radarState": radar,
        "longitudinalPlan": plan,
        "modelV2": model,
        "mapdOut": mapd,
        "leadState": lead_state,
        "followGap": _follow_gap_summary(car, plan, radar["leadOne"], radar["leadTwo"]),
        "long_log": long_log,
        "recent_long_logs": tail.recent()[-12:],
      }
      record["derived"] = _derive_flags(
        car=car,
        plan=plan,
        lead1=radar["leadOne"],
        lead2=radar["leadTwo"],
        long_log=long_log,
      )

      jsonl_f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
      jsonl_f.flush()

      d = record["derived"]
      interesting = False
      if d["clearRoadCandidate"] and d["plannerDropVsSet"]:
        interesting = True
      if d["longHasPlannerOwner"] or d["longHasCurveOwner"]:
        interesting = True
      if lead_state["present"] and (lead_state["sinceStatusChangeMs"] < 1500 or lead_state["shortDropouts"] > 0):
        interesting = True
      if d["closingLead"] and d["plannerDropVsSet"]:
        interesting = True
      if any(marker in d["longSource"] for marker in ("autoengage", "lead_flap", "lead_takeover")):
        interesting = True

      if interesting:
        _write_summary_line(txt_f, record)

      time.sleep(0.001)

    txt_f.write(f"# finished { _wall_iso() }\n")

  print(str(jsonl_path))
  print(str(txt_path))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
