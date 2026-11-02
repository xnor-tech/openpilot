import unittest
from unittest.mock import Mock, patch

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import get_car
from opendbc.car.rivian.fingerprints import FW_VERSIONS
from opendbc.car.rivian.ext_controller import get_safety_CP
from opendbc.car.rivian.interface import CarInterface
from opendbc.car.rivian.values import CAR, FW_QUERY_CONFIG, R1S_WHEELBASE, R1T_WHEELBASE, WMI, ModelLine, ModelYear, set_model_line_specs
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor


class TestRivian(unittest.TestCase):
  def test_custom_fuzzy_fingerprinting(self):
    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for line in ModelLine:
            for year in ModelYear:
              for bad in (True, False):
                vin = ["0"] * 17
                vin[:3] = wmi
                vin[3] = line.value
                vin[9] = year.value
                if bad:
                  vin[3] = "Z"
                vin = "".join(vin)

                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, vin, FW_VERSIONS)
                should_match = year in platform.config.years and not bad
                assert (matches == {platform}) == should_match, "Bad match"


class TestModelLineSpecs(unittest.TestCase):
  @staticmethod
  def _params(vin):
    CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
    CP.carVin = vin
    return CP

  def test_r1s_geometry_preserves_axle_split_and_is_idempotent(self):
    CP = self._params('7PDS0000000000000')
    front_fraction = CP.centerToFront / CP.wheelbase
    set_model_line_specs(CP)
    self.assertAlmostEqual(CP.wheelbase, R1S_WHEELBASE, places=6)
    self.assertAlmostEqual(CP.centerToFront / CP.wheelbase, front_fraction)
    geometry = CP.wheelbase, CP.centerToFront
    set_model_line_specs(CP)
    self.assertEqual((CP.wheelbase, CP.centerToFront), geometry)

  def test_r1t_and_unknown_vins_keep_r1t_geometry(self):
    for vin in ('7FCT0000000000000', '0' * 17, '', '7PD', '7PDZ0000000000000'):
      with self.subTest(vin=vin):
        CP = self._params(vin)
        geometry = CP.wheelbase, CP.centerToFront
        set_model_line_specs(CP)
        self.assertAlmostEqual(CP.wheelbase, R1T_WHEELBASE, places=6)
        self.assertEqual((CP.wheelbase, CP.centerToFront), geometry)

  def test_geometry_is_selected_before_controller_construction(self):
    for vin, wheelbase in (('7PDS0000000000000', R1S_WHEELBASE), ('7FCT0000000000000', R1T_WHEELBASE), ('0' * 17, R1T_WHEELBASE)):
      with self.subTest(vin=vin):
        identified = (CAR.RIVIAN_R1, gen_empty_fingerprint(), vin, [], structs.CarParams.FingerprintSource.fw, True)
        with patch('opendbc.car.car_helpers.fingerprint', return_value=identified):
          CI = get_car(Mock(), Mock(), Mock(), False, False)
        CP = CI.CP
        self.assertAlmostEqual(CP.wheelbase, wheelbase, places=6)
        controller_vm = VehicleModel(CI.CP)
        self.assertEqual(controller_vm.l, CP.wheelbase)
        self.assertEqual(controller_vm.aF, CP.centerToFront)
        with structs.CarParams.from_bytes(CP.to_bytes()) as shared_cp:
          VM = VehicleModel(shared_cp)
          for speed in (0., 5., 30.):
            self.assertEqual(VM.get_steer_from_curvature(0.01, speed, 0.03),
                             controller_vm.get_steer_from_curvature(0.01, speed, 0.03))

  def test_safety_model_matches_existing_panda_geometry(self):
    # The nominal R1T geometry must not implicitly change the independent model in rivian.h.
    VM = VehicleModel(get_safety_CP())
    self.assertAlmostEqual(VM.l, 3.08, places=6)
    self.assertAlmostEqual(VM.sR, 15.2, places=6)
    self.assertAlmostEqual(calc_slip_factor(VM), -0.0005445721739802007)
