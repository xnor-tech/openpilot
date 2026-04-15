#!/usr/bin/env python3
"""
Tesla internal 0x659 "carrier" message generator (XNOR Tesla Legacy).

Why this exists
---------------
Tesla Legacy safety in XNOR consumes a *synthetic* 0x659 CAN frame in the panda
TX hook as an internal contract between userspace and the safety hooks.

In this fork's Tesla Legacy safety (opendbc_repo/opendbc/safety/modes/tesla_legacy.h),
byte5 is interpreted as:
  - bit 0x80: op_autopilot_disabled (gate for allowing OP actuation on AP HW cars)
  - bit 0x20: pedal_enabled (optional)
  - bit 0x02: OP stalk MAIN edge (used to call pcm_cruise_check(true) when OP_STALK_ENABLE flag is set)
  - bit 0x01: OP stalk CANCEL edge (used to call pcm_cruise_check(false) when OP_STALK_ENABLE flag is set)

Important for dual-panda HW2 setups
-----------------------------------
Both pandas run Tesla Legacy safety and must observe the same 0x659 edge events;
otherwise one panda never transitions controlsAllowed=True, causing controlsMismatch.

This module is intentionally **not** a publisher. It only returns CanData frames
to be sent by an existing sendcan publisher (card.py), avoiding MultiplePublishersError.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Sequence

from opendbc.car.can_definitions import CanData

try:
  from openpilot.common.params import Params
except Exception:  # pragma: no cover
  Params = None  # type: ignore


ADDR_CARRIER = 0x659
ADDR_STALK = 0x045  # STW_ACTN_RQ

# Param keys are fork-specific; we probe and cache whichever exist.
AP_DISABLED_KEYS = (
  "TinklaAutopilotDisabled",
  "AutopilotDisabled",
)
PEDAL_ENABLED_KEYS = (
  "TinklaPedalEnabled",
  "PedalEnabled",
)


def _spdctrl(dat: bytes) -> int:
  # STW_ACTN_RQ: your captures show spdctrl in the low 6 bits of byte0 (0..63)
  return (dat[0] & 0x3F) if dat else -1


def _safe_get_bool(params_obj, key: str) -> bool | None:
  if params_obj is None:
    return None
  try:
    return bool(params_obj.get_bool(key))
  except Exception:
    return None


@dataclass
class _Edges:
  main_edge: bool = False
  cancel_edge: bool = False


class Tesla659Carrier:
  """Build synthetic 0x659 frames for one or more sendcan buses."""

  def __init__(
    self,
    buses: Sequence[int] = (0, 4),
    send_hz: float = 10.0,
  ) -> None:
    self._buses = tuple(int(b) for b in buses)
    self._send_period_s = 1.0 / float(send_hz) if send_hz > 0 else 0.0

    self._last_send_ts = 0.0

    self._last_spd = 0
    self._spd = 0
    self._edges = _Edges()

    self._params = Params() if Params is not None else None
    self._params_cache_ts = 0.0
    self._ap_disabled = True   # conservative default for AP HW cars
    self._pedal_enabled = False

  @property
  def spdctrl(self) -> int:
    return self._spd

  @property
  def edges(self) -> _Edges:
    return self._edges

  def update_from_can(self, can_list: Iterable[CanData]) -> None:
    """Parse STW_ACTN_RQ (0x045) to derive MAIN/CANCEL edges."""
    main_edge = False
    cancel_edge = False
    saw_stw = False

    for m in can_list:
      if int(m.address) != ADDR_STALK:
        continue
      saw_stw = True
      dat = bytes(m.dat)
      spd = _spdctrl(dat)

      # MAIN edge: transition into RWD pull (spd==2)
      if spd == 2 and self._last_spd != 2:
        main_edge = True

      # CANCEL edge: transition away from RWD pull (including to IDLE)
      if spd != 2 and self._last_spd == 2:
        cancel_edge = True

      self._spd = spd
      self._last_spd = spd

    if not saw_stw:
      # keep previous spdctrl when STW isn't present in this can batch
      pass

    self._edges = _Edges(main_edge=main_edge, cancel_edge=cancel_edge)

  def _refresh_params(self) -> None:
    """Throttle params reads to ~2 Hz."""
    now = time.monotonic()
    if (now - self._params_cache_ts) < 0.5:
      return
    self._params_cache_ts = now

    for k in AP_DISABLED_KEYS:
      v = _safe_get_bool(self._params, k)
      if v is not None:
        self._ap_disabled = v
        break

    for k in PEDAL_ENABLED_KEYS:
      v = _safe_get_bool(self._params, k)
      if v is not None:
        self._pedal_enabled = v
        break

  def _build_b5(self) -> int:
    self._refresh_params()

    b5 = 0
    if self._ap_disabled:
      b5 |= 0x80
    if self._pedal_enabled:
      b5 |= 0x20
    if self._edges.main_edge:
      b5 |= 0x02
    if self._edges.cancel_edge:
      b5 |= 0x01
    return b5

  def build_msgs(self) -> list[CanData]:
    b5 = self._build_b5()
    dat = bytes([0, 0, 0, 0, 0, b5, 0, 0])
    return [CanData(ADDR_CARRIER, dat, bus) for bus in self._buses]

  def tick(self, can_list: Sequence[CanData]) -> list[CanData]:
    self.update_from_can(can_list)

    if self._send_period_s <= 0:
      return self.build_msgs()

    now = time.monotonic()
    if (now - self._last_send_ts) >= self._send_period_s:
      self._last_send_ts = now
      return self.build_msgs()
    return []
