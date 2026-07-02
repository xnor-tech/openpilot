"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state

ANGLE_COLOR = rl.Color(0x3a, 0xdb, 0x6d, 255)   # green
TORQUE_COLOR = rl.Color(0x4d, 0x9d, 0xff, 255)  # blue

# frames of zero applied torque before we call it angle mode, avoids flicker on torque zero crossings
ZERO_TORQUE_HOLD = 10


class LateralMode:
  # shared Rivian angle/torque inference, computed once per UI frame
  def __init__(self):
    self.mode: str | None = None
    self.zero_torque_cnt = ZERO_TORQUE_HOLD
    self._frame = -1

  def update(self):
    sm = ui_state.sm
    if sm.frame == self._frame:
      return
    self._frame = sm.frame

    cp = ui_state.CP
    if (cp is None or cp.brand != "rivian" or sm.recv_frame["carControl"] < ui_state.started_frame
        or not sm["carControl"].latActive):
      self.mode = None
      return

    # Rivian commands no torque while it steers on its angle channel
    if sm["carOutput"].actuatorsOutput.torque == 0:
      self.zero_torque_cnt = min(self.zero_torque_cnt + 1, ZERO_TORQUE_HOLD)
    else:
      self.zero_torque_cnt = 0

    self.mode = "angle" if self.zero_torque_cnt >= ZERO_TORQUE_HOLD else "torque"

  @property
  def wheel_tint(self) -> "rl.Color | None":
    if self.mode == "angle":
      return ANGLE_COLOR
    if self.mode == "torque":
      return TORQUE_COLOR
    return None


lateral_mode = LateralMode()
