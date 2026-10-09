"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.params import Params

# Speed-scheduled lateral action smoothing: high-bandwidth angle-control plants (Rivian EPAS)
# sustain a model-in-the-loop limit cycle (~2.5 Hz) at crawl speeds where the steering authority
# is largest. Low-pass the action where the loop gain is highest; the smoothing constant is added
# to lat_delay so the plan is read further ahead, compensating the filter lag.
LAT_SMOOTH_BP = [2.0, 8.0]  # m/s
LAT_SMOOTH_MAX = 0.4  # s, at crawl speed


def get_lat_smooth_seconds(v_ego: float, base: float = 0.0) -> float:
  return max(base, float(np.interp(v_ego, LAT_SMOOTH_BP, [LAT_SMOOTH_MAX, 0.0])))


class ModelStateBase:
  def __init__(self):
    self.lat_delay = Params().get("LagdValueCache", return_default=True)
