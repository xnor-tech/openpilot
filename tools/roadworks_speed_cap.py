#!/usr/bin/env python3
# /data/openpilot/tools/roadworks_speed_cap.py
from __future__ import annotations

import argparse
import os
import sys

CAP_FILE = "/data/xnor_roadworks_speed_cap_kph.txt"
PRESET_FILE = "/data/xnor_roadworks_speed_cap_preset_kph.txt"

def mph_to_kph(v: float) -> float:
  return float(v) * 1.609344

def fmt_kph(kph: float | None) -> str:
  if kph is None:
    return "OFF"
  return f"{float(kph):.3f} kph"

def read_float(path: str) -> float | None:
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
  return value if value > 0.0 else None

def write_float(path: str, value_kph: float) -> None:
  tmp_path = f"{path}.tmp"
  with open(tmp_path, "w", encoding="utf-8") as f:
    f.write(f"{float(value_kph):.3f}")
  os.replace(tmp_path, path)

def clear_file(path: str) -> None:
  try:
    os.remove(path)
  except OSError:
    pass

def main(argv: list[str]) -> int:
  parser = argparse.ArgumentParser(description="Manage XNOR temporary roadworks speed cap")
  sub = parser.add_subparsers(dest="cmd", required=True)

  p_set = sub.add_parser("set", help="Enable active temporary cap")
  p_set.add_argument("value", type=float)
  p_set.add_argument("--mph", action="store_true")
  p_set.add_argument("--kph", action="store_true")

  p_preset = sub.add_parser("preset", help="Set triple-pull preset cap")
  p_preset.add_argument("value", type=float)
  p_preset.add_argument("--mph", action="store_true")
  p_preset.add_argument("--kph", action="store_true")

  sub.add_parser("clear", help="Disable active temporary cap")
  sub.add_parser("status", help="Show active cap + preset")

  args = parser.parse_args(argv)

  if args.cmd == "set":
    value_kph = mph_to_kph(args.value) if args.mph else float(args.value)
    write_float(CAP_FILE, value_kph)
    print(f"active cap set: {fmt_kph(value_kph)}")
    return 0

  if args.cmd == "preset":
    value_kph = mph_to_kph(args.value) if args.mph else float(args.value)
    write_float(PRESET_FILE, value_kph)
    print(f"preset cap set: {fmt_kph(value_kph)}")
    return 0

  if args.cmd == "clear":
    clear_file(CAP_FILE)
    print("active cap cleared")
    return 0

  active = read_float(CAP_FILE)
  preset = read_float(PRESET_FILE)
  print(f"active={fmt_kph(active)} preset={fmt_kph(preset)}")
  return 0

if __name__ == "__main__":
  raise SystemExit(main(sys.argv[1:]))
