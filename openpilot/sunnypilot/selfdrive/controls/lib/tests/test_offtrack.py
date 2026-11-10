import math
import unittest

from opendbc.car.car_helpers import interfaces
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.sunnypilot.selfdrive.controls.lib.offtrack import offtrack_curvature


class TestOfftrackCurvature(unittest.TestCase):
  def test_matches_rx_wb_geometry_for_every_platform(self):
    for platform, interface in interfaces.items():
      wheelbase = interface.get_non_essential_params(platform).wheelbase
      for curvature in (-0.2, -0.1, -0.01, 0.01, 0.1, 0.2):
        for gain in (0., 0.5, 1.):
          with self.subTest(platform=platform, curvature=curvature, gain=gain):
            radius = 1. / abs(curvature)
            expected_radius = radius + gain * (radius - math.sqrt(radius**2 - wheelbase**2))
            self.assertAlmostEqual(offtrack_curvature(curvature, wheelbase, gain), math.copysign(1. / expected_radius, curvature))

  def test_longer_wheelbase_and_higher_gain_widen_turns_more(self):
    curvature = 0.1
    short = offtrack_curvature(curvature, 2.5, 0.5)
    long = offtrack_curvature(curvature, 3.5, 0.5)
    full = offtrack_curvature(curvature, 3.5, 1.)
    self.assertTrue(0. < full < long < short < curvature)
    self.assertEqual(offtrack_curvature(-curvature, 3.5, 1.), -full)

  def test_disabled_straight_and_invalid_geometry(self):
    for curvature, wheelbase, gain in ((0.1, 3., 0.), (0.1, 3., -1.), (0., 3., 1.),
                                       (0.1, 0., 1.), (0.1, -3., 1.), (0.5, 3., 1.), (0.5, 2., 1.),
                                       (0.1, math.nan, 1.), (0.1, math.inf, 1.), (0.1, 3., math.nan), (0.1, 3., math.inf)):
      with self.subTest(curvature=curvature, wheelbase=wheelbase, gain=gain):
        self.assertEqual(offtrack_curvature(curvature, wheelbase, gain), curvature)

  def test_gain_cannot_exceed_full_compensation(self):
    self.assertEqual(offtrack_curvature(0.1, 3., 10.), offtrack_curvature(0.1, 3., 1.))

  def test_gentle_curves_remain_finite(self):
    for curvature in (1e-5, 1e-20, 1e-200, 5e-324):
      with self.subTest(curvature=curvature):
        compensated = offtrack_curvature(curvature, 3., 1.)
        self.assertTrue(math.isfinite(compensated))
        self.assertTrue(0. < compensated <= curvature)

  def test_compensation_preserves_curvature_limits(self):
    for speed in (0., 5., 15., 30.):
      for roll in (-0.05, 0., 0.05):
        for target in (-0.5, -0.2, 0.2, 0.5):
          with self.subTest(speed=speed, roll=roll, target=target):
            compensated = offtrack_curvature(target, 3., 1.)
            actual, limited = clip_curvature(speed, 0., compensated, roll)
            expected, expected_limited = clip_curvature(speed, 0., target, roll)
            self.assertAlmostEqual(actual, expected)
            self.assertEqual(limited, expected_limited)


if __name__ == "__main__":
  unittest.main()
