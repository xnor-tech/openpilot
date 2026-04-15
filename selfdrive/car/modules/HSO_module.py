"""openpilot/selfdrive/car/modules/HSO_module.py

Human Steering Override (HSO) — Unity parity.

When enabled, lateral control goes 'numb' for a short period after the driver steers
(defined by handsOnLevel >= TinklaHandsOnLevel). During the numb period we stop
commanding steering, but keep the system engaged so it can resume automatically.

Called from CarInterface.post_update().
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class HSOController:
  human_control: bool = False
  frame_human_steered: int = 0

  def update_stat(self, CS, enabled: bool, actuators, frame: int) -> bool:
    if (not enabled) or (not bool(getattr(CS, "enableHSO", False))):
      self.human_control = False
      return False

    pressed = bool(getattr(CS, "HSOSteeringPressed", False))
    numb_s = float(getattr(CS, "hsoNumbPeriod", 1.5) or 1.5)
    numb_frames = int(numb_s * 100)

    if pressed:
      self.frame_human_steered = int(frame)
    else:
      stalk = int(getattr(CS, "turnSignalStalkState", 0) or 0)
      if (frame - self.frame_human_steered) < numb_frames and stalk > 0:
        self.frame_human_steered = int(frame)
      elif (frame - self.frame_human_steered) < numb_frames:
        try:
          apply_steer = float(getattr(actuators, "steeringAngleDeg", 0.0))
          angle_diff = abs(apply_steer - float(getattr(CS.out, "steeringAngleDeg", 0.0)))
          if angle_diff > 30.0:
            self.frame_human_steered = int(frame)
        except Exception:
          pass

    self.human_control = (frame - self.frame_human_steered) < numb_frames
    return bool(self.human_control and enabled)
