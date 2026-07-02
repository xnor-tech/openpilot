import math
from collections import deque
import numpy as np

from opendbc.car.lateral import (
  apply_driver_steer_torque_limits, common_fault_avoidance,
  apply_steer_angle_limits_vm, get_max_angle_delta_vm,
)
from opendbc.car.rivian.values import CarControllerParams as CCP
from opendbc.car.vehicle_model import VehicleModel

# EPAS angle envelope (EPAS_High_Angle_Cmd_Err)
EPAS_FW_MAX_ANGLE_BP = [0.0, 2.78, 5.56, 8.33, 12.50, 16.67, 22.22, 27.78]  # m/s
EPAS_FW_MAX_ANGLE_V  = [500, 500,  250,  150,  85,    56,    40,    25   ]  # deg

# EPAS windowed rate limit (EPAS_High_Actual_Angle_Rate_Err)
EPAS_FW_RATE_BP = [5.56, 8.33, 12.50, 16.67]  # m/s
EPAS_FW_RATE_V  = [4.50, 1.50, 0.60,  0.18 ]  # deg/frame

EPAS_FW_ANGLE_MARGIN = 0.98
EPAS_FW_RATE_MARGIN  = 0.94

# final per-frame cap inside panda's jerk limit
PANDA_STEP_MARGIN = 0.9

MIN_TORQUE_FRAMES = 50
HANDOFF_EXIT_DEG = 15.0      # hand back to angle when the wheel is within this of the commanded angle
UNWIND_HANDOFF_RATE = 40.0  # max wheel speed in deg/s to hand back to angle
EAC_RECOVER_FRAMES = 15     # angle frames with the EPAS EAC not active before falling back to torque (~0.15s, normal activation is under 0.05s)

# blip the TOI request bit at high angle so the EPAS does not latch ToiFlt
TOI_MAX_ANGLE_DEG = 90
TOI_MAX_ANGLE_FRAMES = 89        # frames held high before a blip (~0.9s)
TOI_BLIP_FRAMES = 2              # frames to release ACM_lkaActToi


class _RateBudget:
  # sliding-window budget for the EPAS rate limit; history is CAN-quantized to 0.1 deg
  WINDOW_USER_FRAMES = 16
  WINDOW_TIME_S = 0.16

  def __init__(self):
    self.history = deque([0.0] * self.WINDOW_USER_FRAMES, maxlen=self.WINDOW_USER_FRAMES)

  def push(self, sent_angle: float) -> None:
    self.history.append(round(sent_angle * 10) / 10)

  def bounds(self, threshold_dps: float, margin: float):
    cmd_oldest = self.history[0]
    budget = threshold_dps * self.WINDOW_TIME_S * margin
    return cmd_oldest - budget, cmd_oldest + budget


def get_safety_CP():
  from opendbc.car.rivian.interface import CarInterface
  return CarInterface.get_non_essential_params("RIVIAN_R1")


class ExternalController:
  def __init__(self, CP):
    self.CP = CP
    self.steer_ratio = CP.steerRatio
    self.wheelbase = CP.wheelbase
    self.VM = VehicleModel(get_safety_CP())

    # hands-on
    self.wheel_touch_cnt = 0
    self.torsion_cnt = 0
    self.torsion_sign = 0
    self.hands_on = False

    # cooperative torque mode
    self.torque_active = False
    self.torque_active_frames = 0
    self.lat_active_last = False
    self.eac_dead_frames = 0

    # angle command
    self.apply_angle_last = 0.0
    self.angle_active = False
    self.rate_budget = _RateBudget()
    # liveParameters, pushed in from card each frame
    self.roll = 0.0
    self.angle_offset_deg = 0.0

    # cooperative torque
    self.apply_torque_last = 0
    # decoupled from torque_active so a blip does not flip angle or feature mode
    self.toi_angle_limit_counter = 0
    self.toi_act_cmd = False     # sent into ACM_lkaActToi, low for 2 frames during a blip

  def update(self, CS, lat_active: bool, actuators):
    self._update_hands_on(CS)
    desired_angle = math.degrees(self.VM.get_steer_from_curvature(-float(actuators.curvature), CS.out.vEgo, self.roll)) + self.angle_offset_deg
    self._update_torque_active(CS, lat_active, desired_angle)
    self._update_angle(CS, lat_active, desired_angle)
    self._update_torque(CS, actuators)

  def _update_wheel_touched(self, wheel_touched, wheel_touched_min_count):
    self.wheel_touch_cnt += 1 if wheel_touched else -1
    self.wheel_touch_cnt = int(np.clip(self.wheel_touch_cnt, 0, wheel_touched_min_count * 2 + 1))
    return self.wheel_touch_cnt > wheel_touched_min_count

  def _update_torsion(self, torque, torque_threshold, torsion_min_count):
    abs_torque = abs(torque)
    pressed = abs_torque > torque_threshold
    sign = int(np.sign(torque))
    # reset on sign flip, opposing torque applications shouldn't accumulate
    if pressed and self.torsion_sign and sign != self.torsion_sign:
      self.torsion_cnt = 0
    else:
      self.torsion_cnt += max(1, math.ceil(abs_torque / torque_threshold)) if pressed else -1
      self.torsion_cnt = int(np.clip(self.torsion_cnt, 0, torsion_min_count * 2 + 1))
    if pressed:
      self.torsion_sign = sign
    return self.torsion_cnt > torsion_min_count

  def _update_hands_on(self, CS):
    # hands-on if any of: capacitive sensor, EPAS-side level, or torsion bar
    calibration = CS.sccm_wheel_touch["SETME_X52"]
    wheel_touch = self._update_wheel_touched(CS.sccm_wheel_touch["SCCM_WheelTouch_CapacitiveValue"] > calibration * 0.9, 25)
    torsion = self._update_torsion(CS.out.steeringTorque, 4.0, 9)
    self.hands_on = wheel_touch or torsion or CS.hands_on_level > 1

  def _update_torque_active(self, CS, lat_active: bool, desired_angle: float):
    self.torque_active_frames = self.torque_active_frames + 1 if self.torque_active else 0

    # EPAS available and no published EacErrorCode
    epas_ready = CS.eac_status == 1 and CS.eac_error_code == 0
    # is the EPAS actually steering on angle
    eac_active = CS.eac_status == 2
    # how far the wheel is from the angle openpilot wants
    gap = abs(desired_angle - CS.out.steeringAngleDeg)

    if not lat_active:
      self.torque_active = False
    # enter torque the moment the driver touches the wheel, which is when the EPAS drops angle control
    elif self.hands_on and CS.out.steeringPressed:
      self.torque_active = True
    # EPAS lost angle and won't recover, torque re-arms it
    elif self.eac_dead_frames >= EAC_RECOVER_FRAMES:
      self.torque_active = True
    # fresh engage while EPAS is not ready yet
    elif not self.lat_active_last and not epas_ready:
      self.torque_active = True
    # hand back to angle once hands off and the wheel is settled near the commanded angle
    elif self.torque_active and self.torque_active_frames >= MIN_TORQUE_FRAMES and not self.hands_on and epas_ready:
      fw_max = float(np.interp(CS.out.vEgoRaw, EPAS_FW_MAX_ANGLE_BP, EPAS_FW_MAX_ANGLE_V)) * EPAS_FW_ANGLE_MARGIN
      in_envelope = abs(CS.out.steeringAngleDeg) < fw_max
      # only once the wheel motion fits the EPAS rate budget
      thr_dps = float(np.interp(CS.out.vEgoRaw, EPAS_FW_RATE_BP, EPAS_FW_RATE_V)) * 100.0
      lo, hi = self.rate_budget.bounds(thr_dps, EPAS_FW_RATE_MARGIN)
      rate_settled = lo <= CS.out.steeringAngleDeg <= hi and abs(CS.out.steeringRateDeg) < UNWIND_HANDOFF_RATE
      if in_envelope and rate_settled and gap < HANDOFF_EXIT_DEG:
        self.torque_active = False

    # count consecutive frames we are trying to steer on angle but the EPAS EAC is not active
    if lat_active and not self.torque_active and not eac_active:
      self.eac_dead_frames += 1
    else:
      self.eac_dead_frames = 0

    self.lat_active_last = lat_active

  def _update_angle(self, CS, lat_active: bool, desired_angle: float):
    self.angle_active = lat_active and not self.torque_active

    apply_angle = desired_angle

    # use future v_ego so the jerk limit ramps the angle down before the lat-accel envelope shrinks
    v_lookahead = max(CS.out.vEgoRaw + max(CS.out.aEgo, 0.0), 1.0)
    apply_angle = apply_steer_angle_limits_vm(apply_angle, self.apply_angle_last, v_lookahead,
                                              CS.out.steeringAngleDeg, self.angle_active, CCP, self.VM)

    if self.angle_active:
      # EPAS absolute envelope
      fw_max = float(np.interp(CS.out.vEgoRaw, EPAS_FW_MAX_ANGLE_BP, EPAS_FW_MAX_ANGLE_V)) * EPAS_FW_ANGLE_MARGIN
      apply_angle = float(np.clip(apply_angle, -fw_max, fw_max))

      # EPAS windowed rate budget
      thr_dps = float(np.interp(CS.out.vEgoRaw, EPAS_FW_RATE_BP, EPAS_FW_RATE_V)) * 100.0
      lo, hi = self.rate_budget.bounds(thr_dps, EPAS_FW_RATE_MARGIN)
      apply_angle = float(np.clip(apply_angle, lo, hi))

      # panda's per-frame jerk limit
      step = get_max_angle_delta_vm(max(CS.out.vEgoRaw, 1.0), self.VM, CCP) * PANDA_STEP_MARGIN
      apply_angle = float(np.clip(apply_angle, self.apply_angle_last - step, self.apply_angle_last + step))

    self.apply_angle_last = apply_angle
    self.rate_budget.push(apply_angle)

  def _update_torque(self, CS, actuators):
    if not self.torque_active:
      self.apply_torque_last = 0
      self.toi_act_cmd = False
      self.toi_angle_limit_counter = 0
      return

    v_ego = CS.out.vEgoRaw
    steer_max = round(float(np.interp(v_ego, CCP.STEER_MAX_LOOKUP[0], CCP.STEER_MAX_LOOKUP[1])))
    new_torque = int(round(float(actuators.torque) * steer_max))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                    CS.out.steeringTorque, CCP, steer_max)

    # blip the TOI request when held at high angle, torque drops to 0 so the rate limiter ramps back from 0
    self.toi_angle_limit_counter, toi_act = common_fault_avoidance(
      abs(CS.out.steeringAngleDeg) >= TOI_MAX_ANGLE_DEG, self.torque_active,
      self.toi_angle_limit_counter, TOI_MAX_ANGLE_FRAMES, TOI_BLIP_FRAMES)
    if not toi_act:
      apply_torque = 0

    self.toi_act_cmd = toi_act
    self.apply_torque_last = apply_torque
