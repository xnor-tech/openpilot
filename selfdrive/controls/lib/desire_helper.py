from __future__ import annotations

import time

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.0

_TINKLA_PARAM_POLL_S = 1.0


DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self) -> None:
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Unity ALC parity (tap indicator -> auto start after delay)
    self._params = Params()
    self._tinkla_enable_alc = False
    self._tinkla_alc_delay_s = 2.0
    self._last_param_poll_t = 0.0
    self._pre_lane_change_start_t: float | None = None

  def _poll_tinkla_params(self) -> None:
    now = time.monotonic()
    if now - self._last_param_poll_t < _TINKLA_PARAM_POLL_S:
      return

    self._last_param_poll_t = now
    try:
      self._tinkla_enable_alc = self._params.get_bool("TinklaEnableALC")
    except UnknownKeyName:
      self._tinkla_enable_alc = False

    try:
      raw = self._params.get("TinklaAlcDelay")
      if raw is None:
        raw = self._params.get("TinklaALCDelay")
      if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="ignore")
      self._tinkla_alc_delay_s = float(raw) if raw not in (None, "") else 2.0
    except (UnknownKeyName, ValueError, TypeError):
      self._tinkla_alc_delay_s = 2.0

    # clamp for safety
    self._tinkla_alc_delay_s = max(0.0, min(self._tinkla_alc_delay_s, 10.0))

  def update(self, carstate, lateral_active: bool, lane_change_prob: float) -> None:
    self._poll_tinkla_params()

    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self._pre_lane_change_start_t = None
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self._pre_lane_change_start_t = time.monotonic()

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and                          ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        auto_start = False
        if self._tinkla_enable_alc and self._pre_lane_change_start_t is not None:
          auto_start = (time.monotonic() - self._pre_lane_change_start_t) >= self._tinkla_alc_delay_s

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self._pre_lane_change_start_t = None
        elif (torque_applied or auto_start) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self._pre_lane_change_start_t = None

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self._pre_lane_change_start_t = time.monotonic()
          else:
            self.lane_change_state = LaneChangeState.off
            self._pre_lane_change_start_t = None

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker
    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
