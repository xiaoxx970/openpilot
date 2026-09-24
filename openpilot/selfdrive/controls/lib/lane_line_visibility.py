from openpilot.common.realtime import DT_CTRL

# The driving model keeps predicting lane lines where there are none (e.g. a parking lot) with a
# probability near zero, while real but worn lines on city roads often sit well below 0.5.
# Replayed over city, motorway and parking lot drives: with these values a parked car shows no
# lines, motorway lines stay visible ~99% of the time, and a side changes state < 1x per minute.
LANE_LINE_ON_PROB = 0.15
LANE_LINE_OFF_PROB = 0.05
LANE_LINE_HOLD_TIME = 2.0  # s a new state must persist before the HUD follows it


class LaneLineVisibility:
  """Debounced 'is this lane line really there' for the car's HUD, one instance per side."""

  def __init__(self, dt: float = DT_CTRL):
    self.dt = dt
    self.visible = False
    self.pending_time = 0.

  def update(self, prob: float) -> bool:
    want = prob > (LANE_LINE_OFF_PROB if self.visible else LANE_LINE_ON_PROB)
    if want == self.visible:
      self.pending_time = 0.
    else:
      self.pending_time += self.dt
      if self.pending_time >= LANE_LINE_HOLD_TIME:
        self.visible = want
        self.pending_time = 0.
    return self.visible
