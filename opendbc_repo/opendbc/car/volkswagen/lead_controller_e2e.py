import cereal.messaging as messaging
import numpy as np

STOP_SPEED_THRESHOLD = 0.5  # m/s
SPEED_DROP_THRESHOLD = 1.5

class LeadControllerE2E():
  def __init__(self):
    assert False
    self.sm = messaging.SubMaster(['modelV2'])
    self.reset()

  def reset(self):
    self.distance = np.inf
    self.has_lead = False
    self.braking_distance = np.inf
    self.braking_detected = False

  def is_lead_present(self):
    return self.has_lead or self.braking_detected

  def get_distance(self):
    return min(self.distance, self.braking_distance)

  def update(self):
    self.sm.update(0)
    if not self.sm.updated['modelV2']:
      return

    model = self.sm['modelV2'].velocity

    if len(model.t) < 2:
      self.reset()
      return

    # not faking a lead in stopped state
    v_current = np.linalg.norm([model.x[0], model.y[0], model.z[0]])
    if v_current < 0.1:
      self.reset()
      return

    v_all = np.stack([model.x, model.y, model.z], axis=1)
    v = np.linalg.norm(v_all, axis=1)
    t = np.array(model.t)
    dt = np.diff(t)
    v_mid = 0.5 * (v[:-1] + v[1:])

    # Stop detection
    below_thresh = np.where(v < STOP_SPEED_THRESHOLD)[0]
    if len(below_thresh) > 0:
      stop_idx = below_thresh[0]
      valid_len = min(stop_idx, len(dt))
      self.distance = max(1, np.sum(v_mid[:valid_len] * dt[:valid_len]))
      self.has_lead = True
    else:
      self.distance = np.inf
      self.has_lead = False

    # Target braking detection (e.g. curves)
    self.braking_detected = False
    self.braking_distance = np.inf

    dv = np.diff(v)
    for i in range(len(dv)):
      if dv[i] <= -SPEED_DROP_THRESHOLD:
        valid_len = min(i, len(dt))
        braking_dist = np.sum(v_mid[:valid_len] * dt[:valid_len])
        self.braking_distance = max(1, braking_dist)
        self.braking_detected = True
        break
