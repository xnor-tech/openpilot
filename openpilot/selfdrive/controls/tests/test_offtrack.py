import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from openpilot.cereal import log
from openpilot.common.params import Params
from opendbc.car.structs import car
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.sunnypilot.selfdrive.controls.lib.offtrack import offtrack_curvature


class SubMasterStub(dict):
  frame = 1
  valid = {'lateralManeuverPlan': False}

  def all_checks(self, services):
    return False


class TestOfftrackControls(unittest.TestCase):
  @staticmethod
  def make_controls(brand, control_type, wheelbase, active=True, maneuver=False, gain=0.5):
    controls = Controls.__new__(Controls)
    controls.CP = SimpleNamespace(brand=brand, wheelbase=wheelbase, minSteerSpeed=0., steerAtStandstill=True,
                                 openpilotLongitudinalControl=False, steerControlType=getattr(car.CarParams.SteerControlType, control_type),
                                 lateralTuning=SimpleNamespace(which=lambda: 'torque' if brand in ('toyota', 'rivian') else 'pid'))
    controls.CP_SP = SimpleNamespace(pcmCruiseSpeed=True)
    controls.CI = Mock()
    controls.CI.get_pid_accel_limits.return_value = (-3.5, 2.)
    controls.VM = Mock()
    controls.VM.calc_curvature.return_value = -0.02
    controls.LoC = Mock(long_control_state='off')
    controls.LoC.update.return_value = 0.
    controls.LaC = Mock()
    controls.LaC_angle = Mock() if brand == 'rivian' else None
    controls.angle_steering = False
    if controls.LaC_angle is not None:
      controls.LaC_angle.update.return_value = (0., 0.012, log.ControlsState.LateralAngleState.new_message())
    controls.LaC.update.return_value = (0., 0.012, log.ControlsState.LateralPIDState.new_message())
    controls.get_lat_active = Mock(return_value=active)
    controls.offtrack_gain = gain
    controls.desired_curvature = 0.
    controls.steer_limited_by_safety = False
    controls.calibrated_pose = None
    controls.lat_delay = 0.2
    controls.sm = SubMasterStub({
      'carState': car.CarState.new_message(vEgo=5.),
      'vehicleParameters': log.VehicleParameters.new_message(stiffnessFactor=1., steerRatio=15.),
      'selfdriveState': log.SelfdriveState.new_message(enabled=active),
      'onroadEvents': [],
      'modelV2': log.ModelDataV2.new_message(action={'desiredCurvature': 0.1}),
      'longitudinalPlan': log.LongitudinalPlan.new_message(),
      'lateralManeuverPlan': log.LateralManeuverPlan.new_message(desiredCurvature=0.08),
      'lateralDelay': SimpleNamespace(lateralDelay=0.2),
      'lateralTorqueParameters': SimpleNamespace(useParams=False),
      'carOutput': SimpleNamespace(actuatorsOutput=car.CarControl.Actuators.new_message()),
    })
    controls.sm.valid = {'lateralManeuverPlan': maneuver}
    return controls

  def test_all_controller_types_receive_compensated_curvature(self):
    for brand, control_type, wheelbase in (('honda', 'torque', 2.7), ('toyota', 'torque', 2.69),
                                            ('nissan', 'angle', 2.7), ('hyundai', 'curvature', 3.), ('rivian', 'torque', 3.08)):
      for maneuver in (False, True):
        with self.subTest(brand=brand, control_type=control_type, maneuver=maneuver):
          controls = self.make_controls(brand, control_type, wheelbase, maneuver=maneuver)
          for _ in range(100):
            CC, _ = controls.state_control()
          target = 0.08 if maneuver else 0.1
          expected = offtrack_curvature(target, wheelbase, 0.5)
          self.assertAlmostEqual(controls.desired_curvature, expected, places=6)
          self.assertAlmostEqual(controls.LaC.update.call_args.args[5], expected, places=6)
          self.assertAlmostEqual(CC.actuators.curvature, 0.012 if control_type == 'curvature' else expected, places=6)
          self.assertTrue(CC.latActive)

  def test_inactive_steering_tracks_measured_curvature(self):
    for maneuver in (False, True):
      controls = self.make_controls('nissan', 'angle', 2.7, active=False, maneuver=maneuver)
      for _ in range(100):
        CC, _ = controls.state_control()
      self.assertFalse(CC.latActive)
      self.assertAlmostEqual(controls.desired_curvature, controls.curvature)
      controls.LaC.reset.assert_called()

  def test_disabled_compensation_and_existing_limits(self):
    controls = self.make_controls('honda', 'torque', 2.7, gain=0.)
    for _ in range(100):
      previous = controls.desired_curvature
      CC, _ = controls.state_control()
      expected, _ = clip_curvature(5., previous, 0.1, 0.)
      self.assertAlmostEqual(CC.actuators.curvature, expected)
    self.assertAlmostEqual(controls.desired_curvature, 0.1, places=6)

  def test_parameter_defaults_and_refresh(self):
    controls = Controls.__new__(Controls)
    with tempfile.TemporaryDirectory() as directory:
      controls.params = Params(directory)
      controls.update_params_1hz()
      self.assertEqual(controls.offtrack_gain, 0.)
      controls.params.put_bool('OfftrackCompensationEnabled', True, block=True)
      controls.update_params_1hz()
      self.assertEqual(controls.offtrack_gain, 0.5)
      controls.params.put('OfftrackCompensationGain', 0.8, block=True)
      controls.update_params_1hz()
      self.assertEqual(controls.offtrack_gain, 0.8)
      controls.params.put_bool('OfftrackCompensationEnabled', False, block=True)
      controls.update_params_1hz()
      self.assertEqual(controls.offtrack_gain, 0.)


if __name__ == '__main__':
  unittest.main()
