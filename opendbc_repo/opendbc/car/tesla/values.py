from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party'})


class CAR(Platforms):
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      TeslaCarDocsHW3("Tesla Model 3 (with HW3) 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 (with HW4) 2024-25"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated'},
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y (with HW3) 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y (with HW4) 2024-25"),
    ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated'},
  )
  TESLA_MODEL_X = TeslaPlatformConfig(
    [TeslaCarDocsHW4("Tesla Model X (with HW4) 2024")],
    CarSpecs(mass=2495., wheelbase=2.960, steerRatio=12.0),
  )
  TESLA_MODEL_X_HW1 = TeslaPlatformConfig(
    [CarDocs("Tesla Model X (with HW1) 2014-16", "All", car_parts=CarParts.common([CarHarness.tesla_model_x_hw1]))],
    CarSpecs(mass=2447., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_can',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_X_HW2 = TeslaPlatformConfig(
    [CarDocs("Tesla Model X (with HW2) 2016-19", "All", car_parts=CarParts.common([CarHarness.tesla_model_sx_hw2]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_powertrain',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S = TeslaPlatformConfig(
    [TeslaCarDocsHW4("Tesla Model S (with HW4) 2024")],
    CarSpecs(mass=2166., wheelbase=2.960, steerRatio=12.0),
  )
  TESLA_MODEL_S_HW1 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW1) 2014-16", "All", car_parts=CarParts.common([CarHarness.tesla_model_s_hw1]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_can',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S_HW2 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW2) 2017-19", "All", car_parts=CarParts.common([CarHarness.tesla_model_sx_hw2]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_can',
      Bus.pt: 'tesla_powertrain',
      Bus.radar: 'tesla_radar_bosch_generated',
    },
  )
  TESLA_MODEL_S_HW3 = TeslaPlatformConfig(
    [CarDocs("Tesla Model S (with HW3) 2020-23", "All", car_parts=CarParts.common([CarHarness.tesla_model_sx_hw3]))],
    CarSpecs(mass=2100., wheelbase=2.960, steerRatio=15.0),
    {
      Bus.chassis: 'tesla_can',
      Bus.party: 'tesla_raven_party',
      Bus.pt: 'tesla_powertrain',
      Bus.radar: 'tesla_radar_continental_generated',
    },
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      rx_offset=0x08,
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      rx_offset=0x08,
      bus=0,
    ),
  ]
)

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
    b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
    b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
  ]
}


class CANBUS:
  party = 0
  vehicle = 1
  radar = 1
  autopilot_party = 2

  # only needed on raven
  powertrain = 4
  chassis = 5
  autopilot_powertrain = 6


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration



class CruiseButtons:
  """SpdCtrlLvr_Stat values (tesla_can.dbc STW_ACTN_RQ)."""
  IDLE = 0
  CANCEL = 1
  MAIN = 2
  RES_ACCEL_2ND = 4
  DECEL_2ND = 8
  RES_ACCEL = 16
  DECEL_SET = 32

  @classmethod
  def is_accel(cls, btn: int) -> bool:
    return btn in (cls.RES_ACCEL, cls.RES_ACCEL_2ND)

  @classmethod
  def is_decel(cls, btn: int) -> bool:
    return btn in (cls.DECEL_SET, cls.DECEL_2ND)


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360,  # deg
    ([0., 5., 15.], [11.5, 10.5, 3.6]),
    ([0., 5., 15.], [11.5, 11.5, 6.2]),
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),
    MAX_LATERAL_JERK=3.2 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),
    MAX_ANGLE_RATE=5,
  )

  CURVE_ASSIST_ANGLE_BP = [0.0, 4.0, 10.0, 18.0]
  CURVE_ASSIST_GAIN_V = [1.00, 1.005, 1.015, 1.025]
  CURVE_ASSIST_EXTRA_DEG_V = [0.0, 0.03, 0.18, 0.40]
  CURVE_ASSIST_SPEED_BP = [0.0, 8.0, 15.0, 25.0, 35.0]
  CURVE_ASSIST_SPEED_GAIN_V = [0.0, 0.25, 0.45, 0.60, 0.70]
  CURVE_ASSIST_MAX_DELTA_BP = [0.0, 10.0, 20.0, 30.0]
  CURVE_ASSIST_MAX_DELTA_V = [0.3, 0.6, 1.0, 1.8]

  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.48
  JERK_LIMIT_MAX = 4.9
  JERK_LIMIT_MIN = -4.9
  JERK_RAMP_RATE = JERK_LIMIT_MAX * 0.002


class TeslaLegacyParams(IntFlag):
  NO_SDM1 = 1


class TeslaSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  FLAG_EXTERNAL_PANDA = 4
  FLAG_HW1 = 8
  FLAG_HW2 = 16
  FLAG_HW3 = 32
  OP_STALK_ENABLE = 64


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  MISSING_DAS_SETTINGS = 4


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1

LEGACY_CARS = (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_S_HW3, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_X_HW2)

