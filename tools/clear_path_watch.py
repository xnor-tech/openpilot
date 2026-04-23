#!/usr/bin/env python3
# /data/openpilot/tools/clear_path_watch.py
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
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
    "modelProb": _safe_float(_maybe_attr(lead, "modelProb", 0.0)),
    "radar": _safe_bool(_maybe_attr(lead, "radar", False)),
    "fcw": _safe_bool(_maybe_attr(lead, "fcw", False)),
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
    "steeringPressed": _safe_bool(_maybe_attr(cs, "steeringPressed", False)),
    "leftBlinker": _safe_bool(_maybe_attr(cs, "leftBlinker", False)),
    "rightBlinker": _safe_bool(_maybe_attr(cs, "rightBlinker", False)),
    "speedLimit": _safe_float(_maybe_attr(cs, "speedLimit", 0.0)),
    "speedLimitOffset": _safe_float(_maybe_attr(cs, "speedLimitOffset", 0.0)),
  }
  if cruise is not None:
    out["cruiseState"] = {
      "enabled": _safe_bool(_maybe_attr(cruise, "enabled", False)),
      "available": _safe_bool(_maybe_attr(cruise, "available", False)),
      "standstill": _safe_bool(_maybe_attr(cruise, "standstill", False)),
      "speed": _safe_float(_maybe_attr(cruise, "speed", 0.0)),
      "speedCluster": _safe_float(_maybe_attr(cruise, "speedCluster", 0.0)),
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

  out = {
    "hasLead": _safe_bool(_maybe_attr(lp, "hasLead", False)),
    "aTarget": _safe_float(_maybe_attr(lp, "aTarget", 0.0)),
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
  return out


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

  out["leadsV3"] = leads_out
  return out


def _derive_flags(*, car: dict[str, Any], plan: dict[str, Any], lead1: dict[str, Any], lead2: dict[str, Any], long_log: dict[str, Any] | None) -> dict[str, Any]:
  cruise = car.get("cruiseState", {}) if isinstance(car.get("cruiseState"), dict) else {}
  current_set = _safe_float(cruise.get("speed"), 0.0)
  v_ego = _safe_float(car.get("vEgo"), 0.0)
  p_near = plan.get("p_near")
  p_preview = plan.get("p_preview")
  plan_drop = False
  if isinstance(p_near, (float, int)) and current_set > 0.0:
    plan_drop = float(p_near) < (current_set - 1.0)
  no_actual_lead = not bool(lead1.get("status")) and not bool(lead2.get("status"))
  long_src = _safe_str((long_log or {}).get("src", ""))
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
  }


class SwaglogTail:
  KEYWORD = "[XNOR_CRUISE_SYNC]"

  def __init__(self) -> None:
    self._lock = threading.Lock()
    self._recent: deque[dict[str, Any]] = deque(maxlen=40)
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
    files = [Path("/data/log")]
    out: list[Path] = []
    for folder in files:
      if not folder.exists():
        continue
      try:
        for path in sorted(folder.glob("swaglog*")):
          if path.is_file():
            out.append(path)
      except Exception:
        continue
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
    if self.KEYWORD not in line:
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
      if k in ("src", "uom", "reason"):
        record[k] = v
      elif k in ("tgt", "cur", "est"):
        record[k] = _safe_float(v, 0.0)
      elif k == "btn":
        record[k] = _safe_int(v, 0)

    with self._lock:
      self._recent.append(record)


def _write_summary_line(txt_f, record: dict[str, Any]) -> None:
  flags = record["derived"]
  long_log = record.get("long_log") or {}
  line = (
    f"{record['wall_time']} "
    f"vEgo={flags['vEgo']:.2f} "
    f"set={flags['currentSet']:.2f} "
    f"clear={int(flags['clearRoadCandidate'])} "
    f"plannerDrop={int(flags['plannerDropVsSet'])} "
    f"lead1={int(record['radarState']['leadOne'].get('status', False))} "
    f"lead2={int(record['radarState']['leadTwo'].get('status', False))} "
    f"src={flags['longSource'] or '-'} "
    f"tgt={_safe_float(long_log.get('tgt'), 0.0):.2f} "
    f"cur={_safe_float(long_log.get('cur'), 0.0):.2f}"
  )
  txt_f.write(line + "\n")
  txt_f.flush()


def main() -> int:
  parser = argparse.ArgumentParser(description="Watch clear-road handoff inputs + LONG owner output.")
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
        "long_log": long_log,
        "recent_long_logs": tail.recent()[-8:],
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

      interesting = False
      d = record["derived"]
      if d["clearRoadCandidate"] and d["plannerDropVsSet"]:
        interesting = True
      if d["longHasPlannerOwner"] or d["longHasCurveOwner"]:
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
