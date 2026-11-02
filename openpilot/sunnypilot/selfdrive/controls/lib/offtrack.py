import math


def offtrack_curvature(curvature: float, wheelbase: float, gain: float) -> float:
  """Widen a turn by gain * (r - sqrt(r**2 - wheelbase**2)), as in rx-wb."""
  if not all(math.isfinite(v) for v in (curvature, wheelbase, gain)) or wheelbase <= 0. or gain <= 0.:
    return curvature

  wheelbase_over_radius = wheelbase * abs(curvature)
  if not 0. < wheelbase_over_radius < 1.:
    return curvature

  # Rationalize 1 - sqrt(1 - (L/r)**2) to avoid cancellation on gentle curves.
  relative_cut_in = wheelbase_over_radius**2 / (1. + math.sqrt(1. - wheelbase_over_radius**2))
  return curvature / (1. + min(gain, 1.) * relative_cut_in)
