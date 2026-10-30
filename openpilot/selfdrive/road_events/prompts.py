"""When a road event becomes a cluster prompt. Pure python, so the offline replay runs the same code."""
import math

EVENT_NONE, EVENT_CROSSING, EVENT_ROUNDABOUT, EVENT_JAM, EVENT_DOWNHILL, EVENT_UPHILL = 0, 9, 10, 12, 14, 15
EVENT_SPEED_LIMIT_AHEAD = 4   # the cluster's "speed limit ahead", with the limit in ACC_Tempolimit

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

  Congested or worse (slow too on motorways and trunk roads) and at least MIN_LENGTH long, adjoining stretches joined,
  announced LOOK_AHEAD s before its start at the car's speed (LOOK_AHEAD_FAST on fast roads), NEAR m at the least and FAR
  (FAR_FAST) at the most: a fixed 1.5 km on fast roads came 75 s to 4 min before the jam on a 10-30 drive. No exception
  before traffic lights (the owner's call): a queue Baidu marks congested is worth the prompt there too. Once per jam: again
  only after REARM s with none within FAR (FAR_FAST), so slowing down short of a jam does not announce it twice.
  """
  LOOK_AHEAD = 15.       # s
  LOOK_AHEAD_FAST = 25.  # s (the owner's pick after 20 s, 10-30)
  NEAR = 150.            # m
  FAR = 500.             # m
  FAR_FAST = 1500.       # m
  MIN_LENGTH = 100.      # m
  REARM = 30.            # s

  def __init__(self):
    self.armed = True
    self.until = -math.inf
    self.distance = self.length = 0.
    self.clear_since: float | None = None

  @classmethod
  def far(cls, fast: bool) -> float:
    return cls.FAR_FAST if fast else cls.FAR

  @classmethod
  def window(cls, v: float, fast: bool) -> float:
    """m ahead a jam start is announced at speed v."""
    return min(cls.far(fast), max(cls.NEAR, (cls.LOOK_AHEAD_FAST if fast else cls.LOOK_AHEAD) * v))

  @classmethod
  def runs(cls, jams: list, fast: bool) -> list[tuple[float, float]]:
    """(distance, length) of the jams that count, in route order; jams: (distance ahead, length, status) in route order.
    A negative distance: the car is in that jam."""
    min_status = 2 if fast else 3
    runs: list[list[float]] = []
    for d, length, status in jams:
      if status < min_status:
        continue
      if runs and abs(runs[-1][0] + runs[-1][1] - d) < 1.:
        runs[-1][1] += length
      else:
        runs.append([d, length])
    return [(d, length) for d, length in runs if length >= cls.MIN_LENGTH]

  @classmethod
  def nearest(cls, jams: list, fast: bool, within: float) -> tuple[float, float] | None:
    """(distance, length) of the first jam that counts starting ahead within that many m."""
    return next(((d, length) for d, length in cls.runs(jams, fast) if 0. <= d <= within), None)

  def update(self, t: float, jams: list, fast: bool, v: float, gone: bool = False) -> tuple[int, float, float]:
    """gone: the jam data is gone (the car left the planned road, or it went stale): a prompt being shown ends."""
    if gone:
      self.until = min(self.until, t)
    ahead = self.nearest(jams, fast, self.far(fast))
    if ahead is not None and ahead[0] <= self.window(v, fast) and self.armed:
      self.armed = False
      self.until = t + HOLD
    if ahead is not None:
      self.clear_since = None
    elif self.clear_since is None:
      self.clear_since = t
    if t < self.until:
      if ahead is not None:
        self.distance, self.length = ahead
      return EVENT_JAM, self.distance, self.length
    if not self.armed and self.clear_since is not None and t - self.clear_since >= self.REARM - 1e-6:
      self.armed = True
    return EVENT_NONE, 0., 0.


class JamAhead:
  """Asks for the relaxed following distance while a traffic jam lies ahead; selfdrived applies it.

  Comes on with the jams JamPrompt announces (the same ones, within its window at the car's speed, from MIN_SPEED), but
  stays for the whole approach, not just the prompt. Once on, a jam starting within KEEP times the largest window since then
  keeps it, so a jam start that moves a little between Baidu answers, or the car slowing down, does not switch the distance
  back and forth. Off as soon as the car is in a jam (from there the driver's own distance: queues are driven and stopped
  in as before), when Baidu reports none ahead and when its answer is stale. Without Baidu data for the car's position (no
  GPS in a tunnel) it holds for up to NO_DATA_HOLD s.
  """
  KEEP = 1.5
  NO_DATA_HOLD = 30.   # s

  def __init__(self):
    self.active = False
    self.distance = 0.
    self.keep = 0.       # m: a jam starting this far ahead keeps it on
    self.data_t = -math.inf

  def update(self, t: float, jams: list | None, fast: bool, v: float, moving: bool) -> tuple[bool, float]:
    """jams: the Baidu snapshot's, [] with Baidu off or stale, None without a snapshot for the car's position.
    Returns whether to ask for the relaxed distance and the distance to the start of that jam."""
    if jams is None:
      if t - self.data_t >= self.NO_DATA_HOLD:
        self.active, self.distance = False, 0.
      return self.active, self.distance
    self.data_t = t
    window = JamPrompt.window(v, fast)
    near = max(self.keep, self.KEEP * window) if self.active else window
    # in route order: a jam the car is in comes first, with a negative distance
    first = next((d for d, _ in JamPrompt.runs(jams, fast) if d <= near), None)
    if first is None or first < 0. or not (self.active or moving):
      self.active, self.distance = False, 0.
    else:
      self.keep = max(self.keep, self.KEEP * window) if self.active else self.KEEP * window
      self.active, self.distance = True, first
    return self.active, self.distance


class CameraPrompt:
  """A speed camera with a limit ahead (Baidu, see baidu.py): "speed limit ahead" with its limit, LOOK_AHEAD s before it
  (NEAR m at the least, FAR at the most), for CAMERA_HOLD s, once per camera. Baidu places the camera again in every
  answer, a few meters off at times: one within SAME m of where the announced one is by the car's own odometer, with the
  same limit, is that camera.
  """
  LOOK_AHEAD = 12.     # s
  NEAR = 300.          # m
  FAR = 800.           # m
  CAMERA_HOLD = 5.     # s
  SAME = 50.           # m

  def __init__(self):
    self.odo = 0.
    self.t: float | None = None
    self.fired: tuple[float, float] | None = None   # (odometer at the camera, limit) of the last announced camera
    self.until = -math.inf
    self.distance = self.limit = 0.

  @classmethod
  def window(cls, v: float) -> float:
    return min(cls.FAR, max(cls.NEAR, cls.LOOK_AHEAD * v))

  def update(self, t: float, v: float, distance: float, limit: float, moving: bool, reported: bool = True) -> tuple[int, float, float]:
    """distance, limit: the next camera's (baidu.Snapshot.camera_distance / camera_limit), limit 0 for none; reported: they
    are Baidu's for the car's position (not a gap without GPS). A prompt being shown ends once no camera is reported (the
    car left that road, or the data went stale). Returns (event, distance, limit)."""
    self.odo += v * (t - self.t) if self.t is not None else 0.
    self.t = t
    if reported and limit <= 0.:
      self.until = min(self.until, t)
    if limit > 0.:
      at = self.odo + distance
      if self.fired is not None and abs(at - self.fired[0]) < self.SAME and abs(limit - self.fired[1]) < 0.1:
        self.fired, self.distance = (at, limit), distance   # the announced one, placed again
      elif moving and 0. <= distance <= self.window(v):
        self.fired, self.distance, self.limit = (at, limit), distance, limit
        self.until = t + self.CAMERA_HOLD
    if t < self.until:
      return EVENT_SPEED_LIMIT_AHEAD, max(0., self.distance), self.limit
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
