from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL

Personality = log.LongitudinalPersonality

# A Golf 8's rain sensor amount is smoothed: 0 -> 100 % in about 25 s as rain starts, pinned at 100 in a
# downpour, and a lone 10 % frame now and then when dry (2026-10-24 route 219). The wiper request it
# sends moved between 0, 1 and 2 meanwhile.
RAIN_AMOUNT_WET = 30.        # %
RAIN_ON_TIME = 5.            # s wet before it counts as rain
RAIN_OFF_TIME = 120.         # s dry before it no longer does: the road stays wet after the rain stops
HEAVY_RAIN_HOLD_TIME = 60.   # s without a heavy sign before heavy rain drops back to rain
WIPER_STAGE_1, WIPER_STAGE_2 = 2, 3

DRY, RAIN, HEAVY_RAIN = 0, 1, 2

# the rain following distance sits one step past the farthest personality, only reachable while it rains
RAIN_POSITION = Personality.relaxed + 1


class RainDetector:
  """Rain level from the car's rain sensor and wiper stalk, fed once per carStateIC."""
  def __init__(self, dt: float = DT_CTRL):
    self.dt = dt
    self.wet_time = 0.
    self.dry_time = 0.
    self.heavy_age = HEAVY_RAIN_HOLD_TIME
    self.raining = False

  @property
  def level(self) -> int:
    if not self.raining:
      return DRY
    return HEAVY_RAIN if self.heavy_age < HEAVY_RAIN_HOLD_TIME else RAIN

  def update(self, rain_amount: float, wiper_request: int, wiper_switch: int) -> int:
    # a wiper stalk set to a fixed stage is the driver saying it rains, whatever the sensor reads
    wet = rain_amount >= RAIN_AMOUNT_WET or wiper_switch in (WIPER_STAGE_1, WIPER_STAGE_2)
    heavy = wet and (wiper_switch == WIPER_STAGE_2 or wiper_request >= 2)

    self.wet_time = self.wet_time + self.dt if wet else 0.
    self.dry_time = 0. if wet else self.dry_time + self.dt
    self.heavy_age = 0. if heavy else self.heavy_age + self.dt

    if not self.raining and self.wet_time >= RAIN_ON_TIME:
      self.raining = True
      self.heavy_age = 0. if heavy else HEAVY_RAIN_HOLD_TIME
    elif self.raining and self.dry_time >= RAIN_OFF_TIME:
      self.raining = False
    return self.level


class RainFollow:
  """Selects the rain following distance while it rains, on top of the stored personality.

  Rain switches to it and says so; the rain stopping switches back to the personality from before. The
  distance buttons step through the personalities and, while it rains, the rain distance past relaxed.
  Stepping away from it keeps it off for the rest of that rain. Having stepped around and come back to it,
  the end of the rain goes to relaxed. It is never stored: the personality param keeps the last
  personality, so the next drive does not start on a rain distance.
  """
  def __init__(self):
    self.active = False
    self.touched = False  # distance buttons used during this rain
    self.raining = False

  def top(self, enabled: bool) -> int:
    return RAIN_POSITION if enabled and self.raining else Personality.relaxed

  def position(self, personality: int) -> int:
    return RAIN_POSITION if self.active else personality

  def select(self, position: int, personality: int) -> int:
    """A distance button picked position; returns the personality to store."""
    self.touched = True
    self.active = position == RAIN_POSITION
    return personality if self.active else position

  def update(self, raining: bool, enabled: bool, personality: int) -> tuple[int, bool]:
    """Returns the personality to store and whether to show the following distance."""
    changed = False
    if not enabled:
      changed = self.active
      self.active = False
      self.touched = False
    elif raining and not self.raining:
      self.active = True
      self.touched = False
      changed = True
    elif not raining and self.raining:
      if self.active:
        if self.touched:
          personality = Personality.relaxed
        self.active = False
        changed = True
      self.touched = False
    self.raining = raining
    return personality, changed
