"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs

MadsDataSP = namedtuple("MadsDataSP",
                        ["enable_mads", "use_lka_mode"])


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False, False)

  @staticmethod
  def mads_status_update(CC_SP: structs.CarControlSP) -> MadsDataSP:
    enable_mads = CC_SP.mads.available
    use_lka_mode = enable_mads

    return MadsDataSP(enable_mads, use_lka_mode)

  def update(self, CC_SP: structs.CarControlSP):
    self.mads = self.mads_status_update(CC_SP)
