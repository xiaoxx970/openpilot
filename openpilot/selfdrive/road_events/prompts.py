"""When a road event becomes a cluster prompt. Pure python, so the offline replay runs the same code."""
import math

EVENT_NONE, EVENT_CROSSING, EVENT_ROUNDABOUT, EVENT_JAM, EVENT_DOWNHILL, EVENT_UPHILL = 0, 9, 10, 12, 14, 15

HOLD = 4.0           # s a prompt stays on the cluster


class SlopePrompt:
  """Uphill / downhill ahead, from flat road only (on a slope there is nothing to announce).

  Fires once the road under the car is flat and a slope is predicted ahead for CONFIRM s, stays HOLD s, and fires again
  only after the car is back on flat road with no slope ahead for REARM s, so one slope gives one prompt.
  """
  FLAT = 0.015         # |grade| counted as flat
  THRESHOLD = 0.03     # predicted grade that makes a slope: 3 % gave 0.3 false prompts per 100 km, 2.5 % gave 0.75
  CONFIRM = 1.0        # s
  REARM = 2.0          # s

  def __init__(self):
    self.armed = True
    self.since = None        # (sign, t) the current candidate started
    self.flat_since = None
    self.until = -math.inf
    self.event = EVENT_NONE
    self.distance = 0.

  def update(self, t: float, grade_now: float, sign: int, distance: float) -> tuple[int, float]:
    flat = abs(grade_now) < self.FLAT
    candidate = flat and sign != 0
    if not candidate:
      self.since = None
    elif self.since is None or self.since[0] != sign:
      self.since = (sign, t)

    if self.armed and self.since is not None and t - self.since[1] >= self.CONFIRM - 1e-6:
      self.armed = False
      self.until = t + HOLD
      self.event = EVENT_UPHILL if sign > 0 else EVENT_DOWNHILL
    if t < self.until:
      if candidate and (sign > 0) == (self.event == EVENT_UPHILL):
        self.distance = distance
      return self.event, self.distance

    if not self.armed:
      if flat and sign == 0:
        self.flat_since = t if self.flat_since is None else self.flat_since
        if t - self.flat_since >= self.REARM - 1e-6:
          self.armed = True
      else:
        self.flat_since = None
    return EVENT_NONE, 0.


class JamPrompt:
  """A traffic jam on the road ahead (Baidu's live traffic along the route it predicts, so in the car's direction).

  Congested or worse (slow too on motorways and trunk roads) starting within NEAR m (NEAR_FAST m on fast roads) and at least
  MIN_LENGTH long, adjoining stretches joined. No exception before traffic lights (the owner's call): a queue Baidu marks
  congested is worth the prompt there too. Once per jam: again only after REARM s with none ahead.
  """
  NEAR = 500.          # m
  NEAR_FAST = 1500.    # m
  MIN_LENGTH = 100.    # m
  REARM = 30.          # s

  def __init__(self):
    self.armed = True
    self.until = -math.inf
    self.distance = self.length = 0.
    self.clear_since: float | None = None

  @classmethod
  def nearest(cls, jams: list, fast: bool) -> tuple[float, float] | None:
    """(distance, length) of the first jam that counts; jams: (distance ahead, length, status) in route order."""
    min_status, near = (2, cls.NEAR_FAST) if fast else (3, cls.NEAR)
    runs: list[list[float]] = []
    for d, length, status in jams:
      if status < min_status:
        continue
      if runs and abs(runs[-1][0] + runs[-1][1] - d) < 1.:
        runs[-1][1] += length
      else:
        runs.append([d, length])
    return next(((d, length) for d, length in runs if 0. <= d <= near and length >= cls.MIN_LENGTH), None)

  def update(self, t: float, jams: list, fast: bool) -> tuple[int, float, float]:
    found = self.nearest(jams, fast)
    if found is not None and self.armed:
      self.armed = False
      self.until = t + HOLD
    if found is not None:
      self.clear_since = None
    elif self.clear_since is None:
      self.clear_since = t
    if t < self.until:
      if found is not None:
        self.distance, self.length = found
      return EVENT_JAM, self.distance, self.length
    if not self.armed and self.clear_since is not None and t - self.clear_since >= self.REARM - 1e-6:
      self.armed = True
    return EVENT_NONE, 0., 0.


def ground_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """m, equirectangular: fine for the few hundred meters compared here."""
  k = math.radians(1) * 6371000.
  return math.hypot((lon2 - lon1) * k * math.cos(math.radians(lat1)), (lat2 - lat1) * k)


class MapPrompt:
  """A map feature ahead (roundabout, crossing): one prompt per feature, when it first shows up between NEAR and FAR m on
  the road ahead. Features seen again within DEDUP m of one already announced in the last FORGET s are the same one
  (the second carriageway of a divided road, the next node of the same junction)."""
  DEDUP = 60.          # m
  FORGET = 120.        # s

  def __init__(self, event: int, near: float, far: float):
    self.event, self.near, self.far = event, near, far
    self.fired: list[tuple[float, float, float]] = []   # (t, lat, lon)
    self.until = -math.inf
    self.distance = 0.

  def update(self, t: float, feature) -> tuple[int, float]:
    """feature: None or (distance along the road, lat, lon) of the nearest one ahead."""
    self.fired = [f for f in self.fired if t - f[0] < self.FORGET]
    if feature is not None:
      d, lat, lon = feature
      seen = any(ground_distance(lat, lon, flat, flon) < self.DEDUP for _, flat, flon in self.fired)
      if not seen and self.near <= d <= self.far:
        self.fired.append((t, lat, lon))
        self.until = t + HOLD
      if t < self.until:
        self.distance = d
    if t < self.until:
      return self.event, self.distance
    return EVENT_NONE, 0.
