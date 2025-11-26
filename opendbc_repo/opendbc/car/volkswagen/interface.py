from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.values import CanBus, CAR, NetworkLocation, TransmissionType, VolkswagenFlags, VolkswagenSafetyFlags
from opendbc.car.volkswagen.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volkswagen"
    ret.radarUnavailable = True

    if ret.flags & VolkswagenFlags.PQ:
      # Set global PQ35/PQ46/NMS parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenPq)]
      ret.enableBsm = 0x3BA in fingerprint[0]  # SWA_1

      if 0x440 in fingerprint[0] or docs:  # Getriebe_1
        ret.transmissionType = TransmissionType.automatic
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x1A0, 0xC2)):  # Bremse_1, Lenkwinkel_1
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      # The PQ port is in dashcam-only mode due to a fixed six-minute maximum timer on HCA steering. An unsupported
      # EPS flash update to work around this timer, and enable steering down to zero, is available from:
      #   https://github.com/pd0wm/pq-flasher
      # It is documented in a four-part blog series:
      #   https://blog.willemmelching.nl/carhacking/2022/01/02/vw-part1/
      # Panda ALLOW_DEBUG firmware required.
      #ret.dashcamOnly = True

    elif ret.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      # Set global MEB parameters
      if ret.flags & VolkswagenFlags.MEB:
        safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMeb)]
      elif ret.flags & VolkswagenFlags.MQB_EVO:
        safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagenMqbEvo)]
        
      if ret.flags & VolkswagenFlags.MEB_GEN2:
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.ALT_CRC_VARIANT_1.value

      if ret.flags & VolkswagenFlags.MQB_EVO:
        safety_configs[0].safetyParam |= VolkswagenSafetyFlags.NO_GAS_OFFSET.value
      
      ret.enableBsm = 0x24C in fingerprint[0]  # MEB_Side_Assist_01
      ret.transmissionType = TransmissionType.direct
      #ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerControlType = structs.CarParams.SteerControlType.curvatureDEPRECATED
      ret.steerAtStandstill = True

      if any(msg in fingerprint[1] for msg in (0x520, 0x86, 0xFD, 0x13D)):  # Airbag_02, LWI_01, ESP_21, QFK_01
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if ret.networkLocation == NetworkLocation.gateway:
        ret.radarUnavailable = False
        
      if 0x30B in fingerprint[0]:  # Kombi_01
        ret.flags |= VolkswagenFlags.KOMBI_PRESENT.value

      if 0x25D in fingerprint[0]:  # KLR_01
        ret.flags |= VolkswagenFlags.STOCK_KLR_PRESENT.value

      if all(msg in fingerprint[1] for msg in (0x462, 0x463, 0x464)):  # PSD_04, PSD_05, PSD_06
        ret.flags |= VolkswagenFlags.STOCK_PSD_PRESENT.value

      if 0x464 in fingerprint[0]:  # PSD_06 on bus 0, used additionally for mph detection as long as no native stable speed limit unit flag is found
        ret.flags |= VolkswagenFlags.STOCK_PSD_06_PRESENT.value

      if 0x6B2 in fingerprint[0]: # Diagnose_01 for local time from car
        ret.flags |= VolkswagenFlags.STOCK_DIAGNOSE_01_PRESENT.value

      if 0x3DC in fingerprint[0]:  # Gatway_73
       ret.flags |= VolkswagenFlags.ALT_GEAR.value

    else:
      # Set global MQB parameters
      safety_configs = [get_safety_config(structs.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0] or docs:  # Getriebe_11
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # Motor_EV_01
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in (0x40, 0x86, 0xB2, 0xFD)):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

      if 0x126 in fingerprint[2]:  # HCA_01
        ret.flags |= VolkswagenFlags.STOCK_HCA_PRESENT.value
      if 0x6B8 in fingerprint[0]:  # Kombi_03
        ret.flags |= VolkswagenFlags.KOMBI_PRESENT.value

    # Global lateral tuning defaults, can be overridden per-vehicle
    ret.steerLimitTimer = 0.4

    if ret.flags & VolkswagenFlags.PQ:
      ret.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    elif ret.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      ret.steerActuatorDelay = 0.3
    else:
      ret.steerActuatorDelay = 0.1
      ret.lateralTuning.pid.kpBP = [0.]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpV = [0.6]
      ret.lateralTuning.pid.kiV = [0.2]

    # Global longitudinal tuning defaults, can be overridden per-vehicle

    if ret.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      ret.longitudinalActuatorDelay = 0.5
      ret.radarDelay = 0.8
      #ret.longitudinalTuning.kpBP = [0., 5.]
      ret.longitudinalTuning.kiBP = [0., 30.]
      #ret.longitudinalTuning.kpV = [0.2, 0.] # (with usage of starting state otherwise starting jerk)
      ret.longitudinalTuning.kiV = [0.4, 0.]

    ret.alphaLongitudinalAvailable = ret.networkLocation == NetworkLocation.gateway or docs
    if alpha_long:
      # Proof-of-concept, prep for E2E only. No radar points available. Panda ALLOW_DEBUG firmware required.
      ret.openpilotLongitudinalControl = True
      safety_configs[0].safetyParam |= VolkswagenSafetyFlags.LONG_CONTROL.value
      if ret.transmissionType == TransmissionType.manual:
        ret.minEnableSpeed = 4.5

    ret.pcmCruise = not ret.openpilotLongitudinalControl
    ret.autoResumeSng = ret.minEnableSpeed == -1

    if ret.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      ret.startingState = True # OP long starting state is used: for very slow start the car can go into error (EPB car shutting down bug)
      ret.startAccel = 0.8
      ret.vEgoStarting = 0.5 # minimum ~0.5 m/s acc starting state is neccessary to not fault the car
      ret.vEgoStopping = 0.1
      ret.stopAccel = -0.55 # different stopping accels seen, good working value
    else:
      ret.vEgoStarting = 0.1
      ret.vEgoStopping = 0.5
      ret.stopAccel = -0.55

    CAN = CanBus(fingerprint=fingerprint)
    if CAN.pt >= 4:
      safety_configs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = safety_configs

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:

    ret.intelligentCruiseButtonManagementAvailable = stock_cp.pcmCruise
                       
    return ret
