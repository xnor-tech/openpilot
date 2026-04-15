"""/data/openpilot/selfdrive/car/modules/BLNK_module.py

Unity-style "tap blinker" detection for Tesla ALC parity.

- A short stalk tap is remembered as `tap_direction`.
- The tap stays alive while the lane change is actively owned in that direction.
- Once the comfort-blink window has elapsed and ALC is no longer using that direction, the tap clears.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class _TapState:
  tap_direction: int = 0
  blinker_on_frame_start: int = 0
  blinker_on_frame_end: int = 0
  prev_turnSignalStalkState: int = 0


class BLNKController:
  def __init__(self, tap_detect_frames: int = 55) -> None:
    self.tap_detect_frames = int(tap_detect_frames)
    self._s = _TapState()

  @property
  def tap_direction(self) -> int:
    return int(self._s.tap_direction)

  @tap_direction.setter
  def tap_direction(self, v: int) -> None:
    self._s.tap_direction = int(v)

  def _reset(self) -> None:
    self._s.tap_direction = 0
    self._s.blinker_on_frame_start = 0
    self._s.blinker_on_frame_end = 0

  def update_state(self, CS, frame: int) -> None:
    stalk = int(getattr(CS, "turnSignalStalkState", 0) or 0)
    alca_direction = int(getattr(CS, "alca_direction", 0) or 0)

    if self._s.tap_direction and stalk and self._s.tap_direction != stalk:
      self._reset()

    if stalk and self._s.prev_turnSignalStalkState == 0:
      self._s.blinker_on_frame_start = int(frame)
    elif stalk == 0 and self._s.prev_turnSignalStalkState > 0:
      if (int(frame) - int(self._s.blinker_on_frame_start)) <= self.tap_detect_frames:
        self._s.tap_direction = int(self._s.prev_turnSignalStalkState)
        self._s.blinker_on_frame_end = int(frame)
      else:
        self._reset()

    if (
      self._s.tap_direction > 0
      and (int(frame) - int(self._s.blinker_on_frame_start)) > self.tap_detect_frames
      and (int(frame) - int(self._s.blinker_on_frame_end)) > self.tap_detect_frames
      and int(alca_direction) != int(self._s.tap_direction)
    ):
      self._reset()

    self._s.prev_turnSignalStalkState = stalk
