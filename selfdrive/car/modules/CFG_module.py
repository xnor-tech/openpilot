"""Config helpers backed by Params (Unity compatibility shim)."""
from __future__ import annotations

try:
  from openpilot.common.params import Params
except ImportError:  # pragma: no cover
  from common.params import Params

_P = Params()


def save_bool_param(param: str, val: bool) -> None:
  try:
    _P.put_bool(param, bool(val))
  except Exception:
    pass


def load_bool_param(param: str, default_val: bool) -> bool:
  try:
    v = _P.get(param)
  except Exception:
    return bool(default_val)
  if v is None:
    try:
      _P.put_bool(param, bool(default_val))
    except Exception:
      pass
    return bool(default_val)
  try:
    return _P.get_bool(param)
  except Exception:
    return bool(default_val)


def save_float_param(param: str, val: float) -> None:
  try:
    _P.put(param, str(float(val)))
  except Exception:
    pass


def load_float_param(param: str, default_val: float) -> float:
  try:
    v = _P.get(param)
  except Exception:
    return float(default_val)
  if v is None:
    try:
      _P.put(param, str(float(default_val)))
    except Exception:
      pass
    return float(default_val)
  try:
    return float(v)
  except Exception:
    try:
      _P.put(param, str(float(default_val)))
    except Exception:
      pass
    return float(default_val)


def save_str_param(param: str, val: str) -> None:
  try:
    _P.put(param, str(val))
  except Exception:
    pass


def load_str_param(param: str, default_val: str) -> str:
  try:
    v = _P.get(param)
  except Exception:
    return str(default_val)
  if v is None:
    try:
      _P.put(param, str(default_val))
    except Exception:
      pass
    return str(default_val)
  try:
    return v.decode() if isinstance(v, (bytes, bytearray)) else str(v)
  except Exception:
    try:
      _P.put(param, str(default_val))
    except Exception:
      pass
    return str(default_val)
