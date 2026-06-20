from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.rivian.carstate import CarState
from opendbc.car.rivian.radar_interface import RadarInterface
from opendbc.car.rivian.values import RivianFlags, RivianSafetyFlags
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def build_secondary_lateral_controller(self, CP_SP, dt):
    # cooperative torque alongside the primary angle path (handoff / driver override)
    from opendbc.car.rivian.ext_controller import build_torque_controller
    return build_torque_controller(self.CP, CP_SP, self, dt)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "rivian"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.rivian)]

    # GEN2 (2025+) doesn't have SCCM_WheelTouch on the bus
    if 0x321 not in fingerprint[0]:
      ret.flags |= RivianFlags.GEN2.value

    # no angle upgrade installed
    if 0x1310 not in fingerprint[1]:
      ret.dashcamOnly = True

    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True
    ret.steerLimitTimer = 0.4

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    # TODO: pending finding/handling missing set speed
    ret.alphaLongitudinalAvailable = False
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= RivianSafetyFlags.LONG_CONTROL.value

    ret.longitudinalActuatorDelay = 0.2
    ret.stopAccel = -0.2
    ret.vEgoStopping = 0.25
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.2]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    if 0x131a in fingerprint[1]:
      ret.flags |= RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE.value
      stock_cp.radarUnavailable = False
      stock_cp.enableBsm = True
      stock_cp.alphaLongitudinalAvailable = True

    if alpha_long and stock_cp.alphaLongitudinalAvailable:
      stock_cp.openpilotLongitudinalControl = True
      stock_cp.safetyConfigs[0].safetyParam |= RivianSafetyFlags.LONG_CONTROL.value

    return ret
