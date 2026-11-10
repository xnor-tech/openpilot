import tempfile
import unittest

from opendbc.car import structs
from opendbc.car.rivian.interface import CarInterface
from opendbc.car.rivian.values import CAR, R1S_WHEELBASE, R1T_WHEELBASE
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import set_car_specific_params


class TestRivianSharedGeometry(unittest.TestCase):
  def test_shared_car_params_contain_correct_geometry(self):
    for vin, wheelbase in (('7PDS0000000000000', R1S_WHEELBASE), ('7FCT0000000000000', R1T_WHEELBASE), ('0' * 17, R1T_WHEELBASE)):
      with self.subTest(vin=vin), tempfile.TemporaryDirectory() as directory:
        params = Params(directory)
        CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
        # Exercise card's hook even when the interface was supplied with an existing CarParams.
        CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.RIVIAN_R1)
        CP.carVin = vin
        set_car_specific_params(CP, CP_SP, params)
        self.assertAlmostEqual(CP.wheelbase, wheelbase, places=6)
        self.assertAlmostEqual(CP.centerToFront, wheelbase / 2., places=6)
        params.put('CarParams', CP.to_bytes(), block=True)
        with structs.CarParams.from_bytes(params.get('CarParams')) as shared_cp:
          self.assertEqual(shared_cp.wheelbase, CP.wheelbase)
          self.assertEqual(shared_cp.centerToFront, CP.centerToFront)

  def test_other_brands_keep_their_geometry_and_ratio(self):
    with tempfile.TemporaryDirectory() as directory:
      params = Params(directory)
      CP = structs.CarParams(brand='tesla', carVin='7PDS0000000000000', wheelbase=2.89, centerToFront=1.445, steerRatio=14.)
      CP_SP = structs.CarParamsSP()
      original = CP.wheelbase, CP.centerToFront, CP.steerRatio
      set_car_specific_params(CP, CP_SP, params)
      self.assertEqual((CP.wheelbase, CP.centerToFront, CP.steerRatio), original)


if __name__ == '__main__':
  unittest.main()
