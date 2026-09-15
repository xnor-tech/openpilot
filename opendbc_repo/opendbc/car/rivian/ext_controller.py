import math
from collections import deque
import numpy as np

from opendbc.car.lateral import (
  apply_driver_steer_torque_limits, common_fault_avoidance,
  apply_steer_angle_limits_vm, get_max_angle_delta_vm, get_max_angle_vm,
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
HANDOFF_EXIT_DEG = 15.0       # hand back to angle when the wheel is within this of the commanded angle
UNWIND_HANDOFF_RATE = 40.0    # max wheel speed in deg/s to hand back to angle
HANDOFF_MAX_ANGLE_DEG = 25.0  # no handoff to angle mid-turn

# light-torsion presence, bridges capacitive dropouts while hands slide on the wheel
PRESENCE_TORQUE_THRESHOLD = 1.5
PRESENCE_MIN_FRAMES = 30      # frames above threshold to latch (0.3s)
PRESENCE_HOLD_FRAMES = 100    # latch hold (1.0s)
HANDS_OFF_EXIT_FRAMES = 75    # hands-off time before handing back to angle (0.75s)
EAC_RECOVER_FRAMES = 15       # angle frames with the EPAS EAC not active before falling back to torque

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
  def __init__(self):
    self.VM_safety = VehicleModel(get_safety_CP())

    # hands-on
    self.wheel_touch_cnt = 0
    self.torsion_cnt = 0
    self.hands_on = False
    # torsion observer states
    self.wheel_angle = None  # rad
    self.wheel_rate = 0.0    # rad/s
    self.column_angle_last = 0.0
    self.presence_cnt = 0
    self.presence_hold = 0
    self.hands_off_frames = 0

    # cooperative torque mode
    self.torque_active = False
    self.torque_active_frames = 0
    self.lat_active_last = False
    self.eac_dead_frames = 0

    # angle command
    self.apply_angle_last = 0.0
    self.angle_active = False
    self.rate_budget = _RateBudget()

    # cooperative torque
    self.apply_torque_last = 0
    # decoupled from torque_active so a blip does not flip angle or feature mode
    self.toi_angle_limit_counter = 0
    self.toi_act_cmd = False     # sent into ACM_lkaActToi, low for 2 frames during a blip

  def update(self, CS, lat_active: bool, actuators):
    self._update_hands_on(CS)
    # computed by the angle controller in controlsd with live vehicle parameters
    desired_angle = float(actuators.steeringAngleDeg)
    self._update_torque_active(CS, lat_active, desired_angle)
    self._update_angle(CS, lat_active, desired_angle)
    self._update_torque(CS, actuators)

  def _update_wheel_touched(self, wheel_touched, wheel_touched_min_count):
    self.wheel_touch_cnt += 1 if wheel_touched else -1
    self.wheel_touch_cnt = int(np.clip(self.wheel_touch_cnt, 0, wheel_touched_min_count * 2 + 1))
    return self.wheel_touch_cnt > wheel_touched_min_count

  def _update_torsion(self, torque, torque_threshold, torsion_min_count):
    abs_torque = abs(torque)
    self.torsion_cnt += max(1, math.ceil(abs_torque / torque_threshold)) if abs_torque > torque_threshold else -1
    self.torsion_cnt = int(np.clip(self.torsion_cnt, 0, torsion_min_count * 2 + 1))
    return self.torsion_cnt > torsion_min_count

  def _update_torsion_presence(self, torque):
    # sustained light torque, latched
    self.presence_cnt = self.presence_cnt + 1 if abs(torque) > PRESENCE_TORQUE_THRESHOLD else 0
    if self.presence_cnt >= PRESENCE_MIN_FRAMES:
      self.presence_hold = PRESENCE_HOLD_FRAMES
    elif self.presence_hold > 0:
      self.presence_hold -= 1
    return self.presence_hold > 0

  def _update_driver_torque(self, CS):
    # EPAS angle control itself loads the torsion bar, model the wheel's reaction and subtract it
    inertia = 0.0205    # kg*m^2
    stiffness = 51.0    # Nm/rad
    damping = 0.156     # Nm/(rad/s)
    friction = 0.141    # Nm, tanh-blended over 0.0066 rad/s
    substeps = 10

    column_angle = math.radians(CS.out.steeringAngleDeg)
    if self.wheel_angle is None:
      self.wheel_angle = self.column_angle_last = column_angle
    h = 0.01 / substeps
    for i in range(substeps):
      c = self.column_angle_last + (column_angle - self.column_angle_last) * (i + 1) / substeps
      bar_torque = stiffness * (self.wheel_angle - c)
      accel = (-bar_torque - damping * self.wheel_rate - friction * math.tanh(self.wheel_rate / 0.0066)) / inertia
      self.wheel_rate += accel * h
      self.wheel_angle += self.wheel_rate * h
    self.column_angle_last = column_angle
    return CS.out.steeringTorque - stiffness * (self.wheel_angle - column_angle)

  def _update_hands_on(self, CS):
    # hands-on if any of: capacitive sensor, EPAS-side level, or torsion bar
    driver_torque = self._update_driver_torque(CS)
    calibration = CS.sccm_wheel_touch["SCCM_WheelTouch_Calibration"]
    wheel_touch = self._update_wheel_touched(CS.sccm_wheel_touch["SCCM_WheelTouch_CapacitiveValue"] > calibration * 0.9, 25)
    torsion = self._update_torsion(driver_torque, 3.0, 9)
    presence = self._update_torsion_presence(driver_torque)
    self.hands_on = wheel_touch or torsion or CS.hands_on_level > 1
    self.hands_off_frames = 0 if self.hands_on or presence else self.hands_off_frames + 1

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
    elif self.torque_active and self.torque_active_frames >= MIN_TORQUE_FRAMES and self.hands_off_frames >= HANDS_OFF_EXIT_FRAMES and epas_ready:
      fw_max = float(np.interp(CS.out.vEgoRaw, EPAS_FW_MAX_ANGLE_BP, EPAS_FW_MAX_ANGLE_V)) * EPAS_FW_ANGLE_MARGIN
      # no handoff to an angle the ISO lat-accel envelope won't let us command, panda would block it
      iso_max = get_max_angle_vm(max(CS.out.vEgoRaw, 1.0), self.VM_safety, CCP)
      in_envelope = abs(CS.out.steeringAngleDeg) < min(fw_max, HANDOFF_MAX_ANGLE_DEG, iso_max)
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
                                              CS.out.steeringAngleDeg, self.angle_active, CCP, self.VM_safety)

    if self.angle_active:
      # EPAS absolute envelope
      fw_max = float(np.interp(CS.out.vEgoRaw, EPAS_FW_MAX_ANGLE_BP, EPAS_FW_MAX_ANGLE_V)) * EPAS_FW_ANGLE_MARGIN
      apply_angle = float(np.clip(apply_angle, -fw_max, fw_max))

      # EPAS windowed rate budget
      thr_dps = float(np.interp(CS.out.vEgoRaw, EPAS_FW_RATE_BP, EPAS_FW_RATE_V)) * 100.0
      lo, hi = self.rate_budget.bounds(thr_dps, EPAS_FW_RATE_MARGIN)
      apply_angle = float(np.clip(apply_angle, lo, hi))

      # panda's per-frame jerk limit
      step = get_max_angle_delta_vm(max(CS.out.vEgoRaw, 1.0), self.VM_safety, CCP) * PANDA_STEP_MARGIN
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
