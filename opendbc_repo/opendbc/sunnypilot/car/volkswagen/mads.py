"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser
from opendbc.car.volkswagen.values import GearShifter

ButtonType = structs.CarState.ButtonEvent.Type

TOLERANCE_MAX     = 100 # 1 second
TEMP_CRUISE_FAULT = 6


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.tolerance_counter = TOLERANCE_MAX

  def update_mads(self, ret: structs.CarState, can_parser_pt: CANParser, hca_status) -> None:

    # Block user unintended MADS enablement
    # Safely block MADS from detecting a cruise state transition from parked mode while cruise is temporary not available
    temp_cruise_fault = can_parser_pt.vl["Motor_51"]["TSK_Status"] == TEMP_CRUISE_FAULT
    drive_mode        = ret.gearShifter == GearShifter.drive
    
    if temp_cruise_fault and ret.parkingBrake and not drive_mode:
      ret.cruiseState.available = True
      self.tolerance_counter    = 0
      
    elif self.tolerance_counter < TOLERANCE_MAX: # grant a little bit of time for the car doing its state transition
      ret.cruiseState.available = True
      self.tolerance_counter    = min(self.tolerance_counter + 1, TOLERANCE_MAX)

    # MADS disable via virtual lkas button press
    # Some newer gen MEB cars do not have a main cruise button and a native cancel button is present.
    # WARNING: Cruise state can not be fully toggled off for these cars!
    self.prev_lkas_button = self.lkas_button
    
    # get cancel button press
    user_disable = any(b.type == ButtonType.cancel and b.pressed for b in ret.buttonEvents)
    
    # get states
    steering_enabled = hca_status == "ACTIVE" # assume mads is actively steering
    cruise_standby   = not ret.cruiseState.enabled
    
    # set button to disable MADS if user cancels while cruise not enabled
    self.lkas_button = steering_enabled and user_disable and cruise_standby
    
    # translate into lkas button event for MADS disable
    if self.prev_lkas_button != self.lkas_button:
      # generate event
      ev = structs.CarState.ButtonEvent()
      ev.type = ButtonType.lkas
      ev.pressed = self.lkas_button
      ret.buttonEvents = list(ret.buttonEvents) + [ev]