"""Tesla feature flags (temporary).

This module is intentionally small and defensive:
- All flags default to False.
- Unknown params keys (not yet compiled into params_pyx) do not crash.
- Values are cached briefly to reduce Params I/O.

Params keys (all optional):
- TeslaRadarFilteringEnabled
- TeslaLeadSelectionEnabled
- TeslaSetSpeedOwnershipEnabled
- TeslaAccBlendEnabled
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import ClassVar, Optional

from openpilot.common.params import Params

try:
  from common.params_pyx import UnknownKeyName  # type: ignore
except Exception:  # pragma: no cover
  UnknownKeyName = Exception  # type: ignore


def _truthy(v: Optional[str]) -> bool:
  if v is None:
    return False
  v = v.strip().lower()
  return v in ("1", "true", "yes", "on", "y", "t")


def _safe_get(p: Params, key: str) -> Optional[str]:
  try:
    return p.get(key, encoding="utf8")
  except UnknownKeyName:
    return None
  except Exception:
    return None


@dataclass(frozen=True, slots=True)
class TeslaFeatureFlags:
  radar_filtering: bool = False
  lead_selection: bool = False
  set_speed_ownership: bool = False
  acc_blend: bool = False

  _cached: ClassVar[Optional["TeslaFeatureFlags"]] = None
  _cached_ts: ClassVar[float] = 0.0
  _ttl_s: ClassVar[float] = 1.0

  @classmethod
  def load(cls) -> "TeslaFeatureFlags":
    now = time.monotonic()
    cached = cls._cached
    if cached is not None and (now - cls._cached_ts) < cls._ttl_s:
      return cached

    p = Params()
    flags = TeslaFeatureFlags(
      radar_filtering=_truthy(_safe_get(p, "TeslaRadarFilteringEnabled")),
      lead_selection=_truthy(_safe_get(p, "TeslaLeadSelectionEnabled")),
      set_speed_ownership=_truthy(_safe_get(p, "TeslaSetSpeedOwnershipEnabled")),
      acc_blend=_truthy(_safe_get(p, "TeslaAccBlendEnabled")),
    )

    cls._cached = flags
    cls._cached_ts = now
    return flags
