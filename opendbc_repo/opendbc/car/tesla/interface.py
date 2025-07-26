from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaFlags, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if candidate in LEGACY_CARS:
      TESLA_LEGACY_SAFETY_MODEL = 99
      PARAM_EXTERNAL_PANDA = 2
      PARAM_HW1 = 4
      PARAM_HW2 = 8
      PARAM_HW3 = 16

      if not any(0x201 in f for f in fingerprint.values()):
        ret.flags |= TeslaFlags.NO_SDM1.value

      if candidate in (CAR.TESLA_MODEL_S_HW1,):
        ret.safetyConfigs = [
          get_safety_config(TESLA_LEGACY_SAFETY_MODEL, PARAM_HW1),
        ]
      elif candidate in (CAR.TESLA_MODEL_S_HW2,):
        ret.safetyConfigs = [
          get_safety_config(TESLA_LEGACY_SAFETY_MODEL, PARAM_HW2),
          get_safety_config(TESLA_LEGACY_SAFETY_MODEL, PARAM_HW2 | PARAM_EXTERNAL_PANDA),
        ]
      elif candidate in (CAR.TESLA_MODEL_S_HW3,):
        ret.safetyConfigs = [
          get_safety_config(TESLA_LEGACY_SAFETY_MODEL, PARAM_HW3),
          get_safety_config(TESLA_LEGACY_SAFETY_MODEL, PARAM_HW3 | PARAM_EXTERNAL_PANDA),
        ]
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

      if candidate == CAR.TESLA_MODEL_Y_JUNIPER:
        ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.JUNIPER.value

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_S_HW3)

    ret.alphaLongitudinalAvailable = True
    if alpha_long or candidate in LEGACY_CARS:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret
