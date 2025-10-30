""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.TESLA_MODEL_3: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
      b'TeM3_E014p10_0.0.0 (16),EL014.17.00',
      b'TeM3_ES014p11_0.0.0 (25),ES014.19.0',
      b'TeMYG4_DCS_Update_0.0.0 (13),E4014.28.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),E4014.26.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),E4015.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4015.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),E4L015.03.2',
      b'TeMYG4_Main_0.0.0 (59),E4H014.29.0',
      b'TeMYG4_Main_0.0.0 (65),E4H015.01.0',
      b'TeMYG4_Main_0.0.0 (67),E4H015.02.1',
      b'TeMYG4_SingleECU_0.0.0 (33),E4S014.27',
    ],
  },
  CAR.TESLA_MODEL_Y: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_E014p10_0.0.0 (16),Y002.18.00',
      b'TeM3_E014p10_0.0.0 (16),YP002.18.00',
      b'TeM3_ES014p11_0.0.0 (16),YS002.17',
      b'TeM3_ES014p11_0.0.0 (25),YS002.19.0',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (13),Y4P002.27.1',
      b'TeMYG4_DCS_Update_0.0.0 (9),Y4P002.25.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (2),Y4P003.02.0',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4003.03.2',
      b'TeMYG4_Legacy3Y_0.0.0 (5),Y4P003.03.2',
      b'TeMYG4_SingleECU_0.0.0 (28),Y4S002.23.0',
      b'TeMYG4_SingleECU_0.0.0 (33),Y4S002.26',
    ],
  },
  CAR.TESLA_MODEL_Y_JUNIPER: {
    (Ecu.eps, 0x730, None): [
      b'TeMYG4_Main_0.0.0 (67),Y4OC003.04.3',
      b'\x01\x0113\x04\x00\x01\x00\x00\x00\t\x00\x00\x00\x01\x00\x00\xff\xff'
    ],
  },
  CAR.TESLA_MODEL_X: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_SP_XP002p2_0.0.0 (23),XPR003.6.0',
      b'TeM3_SP_XP002p2_0.0.0 (23),XP003.6.0',
    ],
  },
  CAR.TESLA_MODEL_X_HW1: {
    (Ecu.eps, 0x730, None): [
      b'1057658-00-00B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.TESLA_MODEL_S: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_SP_XP002p2_0.0.0 (23),SPP003.6.0',
      b'TeM3_SP_XP002p2_0.0.0 (23),SPR003.6.0',
      b'TeM3_SP_XP002p2_0.0.0 (36),SPP003.10.0',
    ],
  },
  CAR.TESLA_MODEL_S_HW1: {
    (Ecu.eps, 0x730, None): [
      b'1016704-00-HAA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'\x10\x00A',
    ],
  },
  CAR.TESLA_MODEL_S_HW2: {
    (Ecu.eps, 0x730, None): [
      b'\x10#\x01',
    ],
  },
  CAR.TESLA_MODEL_S_HW3: {
    (Ecu.eps, 0x730, None): [
      b'SX_0.0.0 (99),S013.7',
      b'SX_0.0.0 (99),SR013.7',
    ],
  },
}
