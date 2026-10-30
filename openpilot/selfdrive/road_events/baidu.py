"""Baidu Maps online data for roadeventd: speed cameras and traffic lights on the road the car is predicted to take
(passenger-car cruise API, 小客车巡航API) and the traffic on the road ahead (driving route planning, 驾车路线规划). Baidu sells
road speed limits (road info API, 道路信息API) to companies only (2026-10-30): those come from the car's sign recognition and
the offline map.

Opt-in (param VwBaiduOnline) and inert without a key file: /data/baidu_ak with the lines AK=... and SK=... of a Baidu server
AK set to SN check. Not a Param, those can be backed up off the car. The keys stay in this module: requests are SN signed and
no URL is ever logged.

Network I/O runs in one worker thread with short timeouts. The 2 Hz loop only queues requests and picks up answers, so a slow
or absent network never holds it up; answers picked up late are dropped, and data older than its STALE from its request too.

Quotas (the owner's AK, 2026-10-29): about 100 cruise calls a day (Beijing date), 5000 route plannings. The calls of the day
are counted in COUNT_FILE, across drives and reboots, and stop at DAILY_BUDGET.
- Cruise, for the speed cameras: a detection when the car leaves the predicted route (more than OFF_ROUTE_DIST off its
  polyline for OFF_ROUTE_CYCLES updates) or nears its end (ROUTE_END in town, ROUTE_END_FAST on motorways and trunk roads),
  at most every CRUISE_MIN_GAP; otherwise one every CRUISE_PERIOD, none of those once fewer than LOW_BUDGET are left today;
  parked, one in CRUISE_PARKED at most; never more than BUDGET_PER_HOUR.
- Route planning, for the traffic jams: every ROUTE_PERIOD while moving, from the car to a point ROUTE_AHEAD along the road it
  is expected to take (the cruise route while it follows it, else the offline map's road ahead, else straight on), and at
  once (ROUTE_MIN_GAP) when the car leaves that route or nears its end. An answer that leaves the expected road within its
  first ROUTE_CHECK m (a detour, another carriageway) is not used.
- While navigating (navd's Baidu route, NavRouteIC), the traffic comes with that route instead: no route planning of its own
  while the car follows a navigation route asked less than NAV_STALE ago, and navd asks again every navd.BAIDU_REFRESH.
- Off a route its data is dropped at once, so nothing from the road the car left reaches a prompt or the speed limit.
Errors and rate limits back off up to 5 min; a spent quota waits until midnight Beijing time, logged once.

At a tunnel portal a detection bound the car to the surface road beside and then above the tunnel, traffic lights and all.
Positions cannot tell stacked roads apart, Baidu's link ids can: a route that does not start on a link of the route the car
is following is not taken until the car leaves that one or it ages out.

Pure python + the standard library, no openpilot imports: tests and the offline replay drive it with recorded answers.
"""
import collections
import datetime
import hashlib
import json
import math
import os
import queue
import threading
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field

KEY_FILE = "/data/baidu_ak"
COUNT_FILE = "/data/baidu_calls"   # calls made today per API, with the Beijing date: kept across drives and reboots
HOST = "https://api.map.baidu.com"
ROUTE_PATH = "/direction/v2/driving"
SN_SAFE = "/:=&?#+!$,;'@()*[]"   # characters Baidu's SN rule leaves unquoted

KPH_TO_MS = 1 / 3.6
DAILY_BUDGET = {"cruise": 90, "route": 1500}   # calls a Beijing day: the quotas are about 100 and 5000
LOW_BUDGET = 20            # cruise calls left today below which there are no routine detections
BUDGET_PER_HOUR = {"cruise": 30, "route": 120}
CRUISE_PERIOD = 300.       # s between routine cruise detections
CRUISE_MIN_GAP = 60.       # s between two detections, also when the car leaves the route or nears its end
CRUISE_PARKED = 300.       # s: below PARKED_SPEED at most one detection in this time
PARKED_SPEED = 5. * KPH_TO_MS
STALE = CRUISE_PARKED + 60.  # s a cruise answer is used for, from when it was asked; longer than CRUISE_PERIOD
ROUTE_PERIOD = 60.         # s between route plannings while moving (at PARKED_SPEED or faster)
ROUTE_MIN_GAP = 20.        # s, also when the car leaves the planned route or nears its end
ROUTE_STALE = 150.         # s a route planning is used for
NAV_STALE = 180.           # s the traffic of a navigation route is used for (navd asks again every 120 s while moving)
ROUTE_AHEAD = (2000., 5000.)   # m planned ahead: at least, and at most (at speed: the jam range plus a ROUTE_PERIOD's drive)
ROUTE_MIN = 800.           # m: a shorter expected road (cruise route nearly used up, the map ending) falls to the next source
ROUTE_CHECK = 300.         # m of a planned route that must stay on the expected road (ROUTE_CHECK_STRAIGHT when that is
ROUTE_CHECK_STRAIGHT = 100.  # only the course straight on)
JAM_RANGE = (500., 1500.)  # m ahead a jam is announced in town and on fast roads, as prompts.JamPrompt
ANSWER_MAX_AGE = 10.       # s: an answer picked up later (the loop is not fed without GPS, e.g. in a tunnel) is dropped
CLOCK_SET = 1735689600.    # 2025-01-01: before the clock is set there is no day to count calls against
OFF_ROUTE_DIST = 40.       # m off a route's polyline (GPS, lane and polyline errors stay below ~20 m)
OFF_ROUTE_CYCLES = 10      # updates in a row: 5 s at 2 Hz
OFF_ROUTE_MIN_SPEED = 3.   # m/s: slower (car parks, a queue at the side of the road) the route is not checked
ROUTE_END = 300.           # m before the end of the predicted route a new one is asked for
ROUTE_END_FAST = 1000.     # m, on motorways, trunk roads and their ramps (FAST_ROAD_CLASSES)
FAST_ROAD_CLASSES = frozenset((1, 2, 3, 4))   # OSM classes, as events.FAST_ROADS
CAMERA_OFF_PATH = 30.      # m: speed cameras further off the route's polyline belong to another road
BACKOFF = (5., 300.)       # s, first and longest wait after errors
TIMEOUT = 4.               # s per request
HISTORY = 25               # points the cruise API takes, > 2 m apart; it refuses a detection without at least 2
EXT_INFO = 0x01 | 0x02 | 0x04 | 0x10   # keep the session, traffic lights, cameras, lane guidance (ext_info=23)
CAMERA_RANGE = 1000.       # m
CAMERA_PASSED = 50.        # m after a speed camera it is still reported (its limit holds until then)
CAMERA_SPEED = 1           # camera_list type with a speed limit, the only one with a limit (types 2/3/4/8/10/17 have none)
QUOTA = frozenset((301, 302))                # quota spent (401 / 402, too many requests at once, back off like errors)
BEIJING = datetime.timezone(datetime.timedelta(hours=8))


def unix_now() -> float:
  return time.time()  # noqa: TID251  Baidu takes UNIX seconds for the points' times


def until_beijing_midnight(now: float) -> float:
  """s from unix time now to a minute past the next midnight in Beijing (UTC+8), when Baidu's daily quota starts again."""
  return 86400. - (now + 8 * 3600.) % 86400. + 60.


def beijing_day(now: float) -> str:
  return datetime.datetime.fromtimestamp(now, BEIJING).date().isoformat()


class DailyCount:
  """Calls made today (Beijing date) per API by every drive, kept in a file. Written from a thread of its own: a /data write
  that stalls must not hold up the 2 Hz loop. Without a file (tests, replays) the count lives as long as the object."""
  def __init__(self, path: str | None):
    self.path = path
    self.day, self.counts = "", collections.Counter()
    self.lock, self.write_lock = threading.Lock(), threading.Lock()
    if path is not None:
      try:
        with open(path) as f:
          saved = json.load(f)
        self.day, self.counts = str(saved["day"]), collections.Counter({k: int(v) for k, v in saved["calls"].items()})
      except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass

  def today(self, kind: str, now: float) -> int:
    return self.counts[kind] if self.day == beijing_day(now) else 0

  def left(self, kind: str, now: float) -> int:
    return DAILY_BUDGET[kind] - self.today(kind, now)

  def add(self, kind: str, now: float) -> None:
    day = beijing_day(now)
    with self.lock:
      if day != self.day:
        self.day, self.counts = day, collections.Counter()
      self.counts[kind] += 1
    if self.path is not None:
      threading.Thread(target=self._write, name="baidu-count", daemon=True).start()

  def _write(self) -> None:
    with self.write_lock:   # one at a time, each with the latest count
      with self.lock:
        text = json.dumps({"day": self.day, "calls": dict(self.counts)})
      assert self.path is not None
      try:
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
          f.write(text)
        os.replace(tmp, self.path)
      except OSError:
        pass


def load_keys(path: str = KEY_FILE) -> tuple[str, str] | None:
  try:
    keys = {}
    with open(path) as f:
      for line in f:
        k, _, v = line.strip().partition("=")
        keys[k] = v
    return (keys["AK"], keys["SK"]) if keys.get("AK") and keys.get("SK") else None
  except (OSError, KeyError):
    return None


def signed_get(path: str, params: list[tuple[str, str]], ak: str, sk: str) -> str:
  """Path and query with ak and sn: sn = md5(quote_plus(quote(path?query, SN_SAFE) + sk))."""
  q = urllib.parse.quote(path + "?" + "&".join(f"{k}={v}" for k, v in params + [("ak", ak)]), safe=SN_SAFE)
  return q + "&sn=" + hashlib.md5(urllib.parse.quote_plus(q + sk).encode()).hexdigest()


# ---- GCJ-02 (Baidu's answers) -> WGS-84 (the car), the usual published transform, ~1 m --------------------------------------
_A, _EE = 6378245.0, 0.00669342162296594323


def _gcj_delta(lat: float, lon: float) -> tuple[float, float]:
  x, y = lon - 105., lat - 35.
  dlat = -100. + 2. * x + 3. * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
  dlat += (20. * math.sin(6. * x * math.pi) + 20. * math.sin(2. * x * math.pi)) * 2. / 3.
  dlat += (20. * math.sin(y * math.pi) + 40. * math.sin(y / 3. * math.pi)) * 2. / 3.
  dlat += (160. * math.sin(y / 12. * math.pi) + 320. * math.sin(y * math.pi / 30.)) * 2. / 3.
  dlon = 300. + x + 2. * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
  dlon += (20. * math.sin(6. * x * math.pi) + 20. * math.sin(2. * x * math.pi)) * 2. / 3.
  dlon += (20. * math.sin(x * math.pi) + 40. * math.sin(x / 3. * math.pi)) * 2. / 3.
  dlon += (150. * math.sin(x / 12. * math.pi) + 300. * math.sin(x / 30. * math.pi)) * 2. / 3.
  rad = lat / 180. * math.pi
  magic = 1 - _EE * math.sin(rad) ** 2
  dlat = (dlat * 180.) / ((_A * (1 - _EE)) / (magic * math.sqrt(magic)) * math.pi)
  dlon = (dlon * 180.) / (_A / math.sqrt(magic) * math.cos(rad) * math.pi)
  return dlat, dlon


def gcj02_to_wgs84(lat: float, lon: float) -> tuple[float, float]:
  wlat, wlon = lat, lon
  for _ in range(4):
    dlat, dlon = _gcj_delta(wlat, wlon)
    wlat, wlon = wlat - (wlat + dlat - lat), wlon - (wlon + dlon - lon)
  return wlat, wlon


def lnglat(s: str) -> tuple[float, float]:
  """Baidu's "lng,lat" (gcj02) -> WGS-84 (lat, lon)."""
  lng, lat = (float(v) for v in s.split(","))
  return gcj02_to_wgs84(lat, lng)


def ground(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
  """(east, north) m from point 1 to point 2."""
  k = math.radians(1) * 6371000.
  return (lon2 - lon1) * k * math.cos(math.radians(lat1)), (lat2 - lat1) * k


Path = list[tuple[float, float, float]]   # (distance along, lat, lon)


def project(path: Path, lat: float, lon: float, lo: float = -math.inf, hi: float = math.inf) -> tuple[float, float] | None:
  """(distance along the path, distance off it) of the point's nearest point on the path between lo and hi m along it."""
  best = None
  for (s0, la0, lo0), (s1, la1, lo1) in zip(path, path[1:], strict=False):
    if s1 < lo or s0 > hi:
      continue
    ex, ny = ground(la0, lo0, la1, lo1)
    px, py = ground(la0, lo0, lat, lon)
    u = min(1., max(0., (px * ex + py * ny) / (ex * ex + ny * ny))) if ex or ny else 0.
    off = math.hypot(px - u * ex, py - u * ny)
    if best is None or off < best[1]:
      best = (s0 + u * (s1 - s0), off)
  return best


def as_path(points: list[tuple[float, float]]) -> Path:
  """(lat, lon) points -> (distance along, lat, lon)."""
  path: Path = []
  for lat, lon in points:
    path.append((path[-1][0] + math.hypot(*ground(path[-1][1], path[-1][2], lat, lon)) if path else 0., lat, lon))
  return path


def ahead_of(lat: float, lon: float, course: float, distance: float) -> tuple[float, float]:
  """The point distance m from (lat, lon) along course (rad from north, clockwise)."""
  k = math.radians(1) * 6371000.
  return lat + distance * math.cos(course) / k, lon + distance * math.sin(course) / (k * math.cos(math.radians(lat)))


def leaves(path: Path, expected: list[tuple[float, float]], check: float, step: float = 20.) -> bool:
  """Does the route (distances from the car) get more than OFF_ROUTE_DIST off the expected road within its first check m?"""
  road = as_path(expected)
  for (s0, la0, lo0), (s1, la1, lo1) in zip(path, path[1:], strict=False):
    if s1 < 0.:
      continue
    if s0 > check:
      break
    for s in [max(0., s0) + k * step for k in range(int((min(s1, check) - max(0., s0)) / step) + 1)] + [min(s1, check)]:
      u = (s - s0) / (s1 - s0) if s1 > s0 else 0.
      on = project(road, la0 + u * (la1 - la0), lo0 + u * (lo1 - lo0))
      if on is None or on[1] > OFF_ROUTE_DIST:
        return True
  return False


def route_params(lat: float, lon: float, course: float, v: float, destination: tuple[float, float]) -> list[tuple[str, str]]:
  """A driving route planning from the car (its course in rad, speed m/s) to destination (lat, lon), all WGS-84."""
  return [("origin", f"{lat:.6f},{lon:.6f}"), ("destination", f"{destination[0]:.6f},{destination[1]:.6f}"), ("coord_type", "wgs84"),
          ("ret_coordtype", "gcj02"), ("gps_direction", str(int(round(math.degrees(course))) % 360)), ("speed", f"{v:.2f}")]


# ---- answers -------------------------------------------------------------------------------------------------------------
@dataclass
class CruiseRoute:
  """The predicted route of one cruise detection. Distances are along it from where the car was when it was asked."""
  odo: float                                  # car odometer then, m
  t: float                                    # when it was asked
  length: float
  path: Path                                  # the route's polyline
  lamps: list[tuple[float, float, float]]     # (distance, lat, lon) of traffic lights
  jams: list[tuple[float, float, int]]        # (start, length, status 2 / 3 / 4); not used, route planning's are fresher
  cameras: list[tuple[float, float]]          # (distance, speed limit m/s) of speed cameras
  links: tuple[str, ...] = ()                 # Baidu's link ids along it, the first the one the car is on


def parse_path(steps: list) -> Path:
  path: Path = []
  for step in steps:
    for p in (step.get("path") or "").split(";"):
      if not p:
        continue
      lat, lon = lnglat(p)
      if not path:
        path.append((0., lat, lon))
      elif (lat, lon) != path[-1][1:]:
        path.append((path[-1][0] + math.hypot(*ground(path[-1][1], path[-1][2], lat, lon)), lat, lon))
  return path


def parse_cruise(resp: dict, odo: float, t: float, lat: float, lon: float) -> CruiseRoute | None:
  """The most likely route of a detection asked at (lat, lon). Baidu starts the route, and all its distances, at the start
  of the link the car is on, 40-170 m back; they are moved to start at the car. None when the route misses the car."""
  routes = ((resp.get("result") or {}).get("routes") or []) if resp.get("status") == 0 else []
  if not routes:
    return None
  r = routes[0]
  steps = r.get("steps") or []
  path = parse_path(steps)
  start = project(path, lat, lon)
  if start is None or start[1] > OFF_ROUTE_DIST:
    return None
  s0 = start[0]
  lamps = sorted((float(lp["distance"]) - s0, *lnglat(lp["pos"])) for lp in r.get("lamps") or [] if lp.get("pos") and "distance" in lp)
  cameras = []
  for c in r.get("camera_list") or []:
    if c.get("type") != CAMERA_SPEED or not c.get("speed_limit") or not c.get("position"):
      continue
    on = project(path, *lnglat(c["position"]))
    if on is not None and on[1] < CAMERA_OFF_PATH:
      cameras.append((on[0] - s0, float(c["speed_limit"]) * KPH_TO_MS))
  path = [(d - s0, la, lo) for d, la, lo in path]
  links = tuple(x for x in str(r.get("linkids") or "").split(",") if x)
  return CruiseRoute(odo, t, float(r.get("distance") or 0.) - s0, path, lamps, traffic_runs(steps, s0), sorted(cameras), links)


def traffic_runs(steps: list, s0: float) -> list[tuple[float, float, int]]:
  """(start, length, status) of the slow or worse runs of the steps' traffic, from s0 along their polyline (their geo_cnt
  path segments add up to it), adjoining runs of a status joined."""
  jams: list[tuple[float, float, int]] = []
  s = 0.
  for step in steps:
    for c in step.get("traffic_condition") or []:
      d, status = float(c.get("distance") or 0.), int(c.get("status") or 0)
      if status >= 2 and d > 0:
        if jams and jams[-1][2] == status and abs(jams[-1][0] + jams[-1][1] - (s - s0)) < 1.:
          jams[-1] = (jams[-1][0], jams[-1][1] + d, status)
        else:
          jams.append((s - s0, d, status))
      s += d
  return jams


@dataclass
class PlannedRoute:
  """A route planning answer along the road ahead, for its traffic. Distances are along it from where the car was asked."""
  odo: float
  t: float
  length: float
  path: Path
  jams: list[tuple[float, float, int]]        # (start, length, status 2 slow / 3 congested / 4 severe)
  nav: bool = False                           # navd's navigation route, not one of ours


def nav_route(t: float, points: list[tuple[float, float]], jams: list[tuple[float, float, int]], odo: float, lat: float,
              lon: float) -> PlannedRoute | None:
  """navd's route (asked at t, WGS-84 (lat, lon) points, jams by distance from its first point) moved to start at the car,
  None when the car is not on it."""
  path = as_path(points)
  start = project(path, lat, lon) if len(path) >= 2 else None
  if start is None or start[1] > OFF_ROUTE_DIST:
    return None
  s0 = start[0]
  return PlannedRoute(odo, t, path[-1][0] - s0, [(d - s0, la, lo) for d, la, lo in path],
                      [(a - s0, n, st) for a, n, st in jams if a + n > s0], nav=True)


def parse_route(resp: dict, odo: float, t: float, lat: float, lon: float) -> PlannedRoute | None:
  """The first route of a driving route planning asked at (lat, lon), moved to start at the car. None without one, or when it
  misses the car (the origin bound to another road)."""
  routes = ((resp.get("result") or {}).get("routes") or []) if resp.get("status") == 0 else []
  if not routes:
    return None
  steps = routes[0].get("steps") or []
  path = parse_path(steps)
  start = project(path, lat, lon)
  if start is None or start[1] > OFF_ROUTE_DIST:
    return None
  s0 = start[0]
  return PlannedRoute(odo, t, path[-1][0] - s0, [(d - s0, la, lo) for d, la, lo in path], traffic_runs(steps, s0))


@dataclass
class Snapshot:
  online: bool = False                                          # a cruise answer younger than STALE: lamps and camera
  lamps: list = field(default_factory=list)                     # (distance ahead, lat, lon), nearest first
  jams_online: bool = False                                     # a route planning younger than ROUTE_STALE: jams
  jams: list = field(default_factory=list)                      # (distance ahead, length, status)
  camera_limit: float = 0.                                      # m/s, next speed camera ahead, or passed less than CAMERA_PASSED ago
  camera_distance: float = 0.                                   # m to it, negative once passed


# ---- network ---------------------------------------------------------------------------------------------------------------
def http_get(path_and_query: str) -> dict:
  opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
  with opener.open(HOST + path_and_query, timeout=TIMEOUT) as r:
    return json.loads(r.read())


class Worker:
  """One thread, one request at a time; answers come back through a queue."""
  def __init__(self, fetch=http_get):
    self.fetch = fetch
    self.jobs: queue.Queue = queue.Queue(maxsize=4)
    self.done: queue.Queue = queue.Queue()
    self.busy = set()
    threading.Thread(target=self._run, name="baidu", daemon=True).start()

  def submit(self, kind: str, path_and_query: str, meta) -> bool:
    if kind in self.busy:
      return False
    try:
      self.jobs.put_nowait((kind, path_and_query, meta))
    except queue.Full:
      return False
    self.busy.add(kind)
    return True

  def _run(self):
    while True:
      kind, pq, meta = self.jobs.get()
      try:
        resp, err = self.fetch(pq), None
      except Exception as e:   # network, timeout, bad JSON: only the type is kept, never a message with the URL
        resp, err = None, type(e).__name__
      self.done.put((kind, meta, resp, err))

  def results(self):
    while True:
      try:
        kind, meta, resp, err = self.done.get_nowait()
      except queue.Empty:
        return
      self.busy.discard(kind)
      yield kind, meta, resp, err


# ---- scheduling ------------------------------------------------------------------------------------------------------------
class BaiduOnline:
  def __init__(self, keys: tuple[str, str], worker=None, clock=time.monotonic, log=None, count_file: str | None = None):
    self.ak, self.sk = keys
    self.worker = worker if worker is not None else Worker()
    self.clock = clock
    self.log = log if log is not None else (lambda msg: None)
    self.logged: set[str] = set()               # one-time log lines already given
    self.daily = DailyCount(count_file)
    self.history: collections.deque = collections.deque(maxlen=HISTORY)   # (lat, lon, v, course deg, t)
    # cruise: the speed cameras on the route Baidu predicts
    self.route: CruiseRoute | None = None
    self.off_route = 0
    self.last_detect = -math.inf
    self.want_detect = True
    # route planning: the traffic on the road ahead
    self.planned: PlannedRoute | None = None
    self.off_planned = 0
    self.last_plan = -math.inf
    self.want_plan = True
    self.discarded = 0                          # plannings that left the expected road
    self.nav_asked = None                       # request time of the last navigation route looked at
    self.call_times: dict[str, collections.deque] = {k: collections.deque() for k in BUDGET_PER_HOUR}
    self.road_key = None
    self.backoff: dict[str, float] = dict.fromkeys(BUDGET_PER_HOUR, 0.)
    self.retry_at: dict[str, float] = dict.fromkeys(BUDGET_PER_HOUR, -math.inf)
    self.calls: collections.Counter = collections.Counter()

  # -- inputs
  def update(self, odo: float, lat: float, lon: float, course: float, v: float, road: tuple | None, road_ahead=None,
             nav=None) -> Snapshot:
    """odo: m driven since start; course: rad from north, clockwise; road: (name, OSM class) the car is matched to, or None.
    road_ahead(lat, lon, course, length): the offline map's road ahead of the car as [(lat, lon), ...], None off the map;
    route planning asks along it when there is no cruise route to follow.
    nav: navd's navigation route as (asked at, in this clock; [(lat, lon), ...]; [(start, length, status), ...]), or None."""
    t = self.clock()
    self._collect(t, odo)
    if nav is not None and nav[0] != self.nav_asked:
      self.nav_asked = nav[0]
      planned = nav_route(nav[0], nav[1], nav[2], odo, lat, lon) if t - nav[0] < NAV_STALE else None
      if planned is not None:
        self.planned, self.off_planned, self.want_plan = planned, 0, False
    if not self.history or math.hypot(*ground(self.history[-1][0], self.history[-1][1], lat, lon)) > 2.:
      self.history.append((lat, lon, v, math.degrees(course) % 360., t))

    self.road_key = road
    self._follow_route(odo, lat, lon, v)
    self._follow_planned(odo, lat, lon, v)

    self._schedule(t, odo, lat, lon, course, v, road_ahead)
    return self._snapshot(t, odo)

  def _fast(self) -> bool:
    return self.road_key is not None and self.road_key[1] in FAST_ROAD_CLASSES

  def _route_end(self) -> float:
    """m before the end of the predicted route from where a new one is asked for."""
    return ROUTE_END_FAST if self._fast() else ROUTE_END

  def _follow_route(self, odo: float, lat: float, lon: float, v: float):
    """Is the car still on the predicted route? Off it, the route's lights and cameras are dropped at once."""
    r = self.route
    if r is None:
      return
    if odo - r.odo > r.length - self._route_end():
      self.want_detect = True
    self.off_route = self._off(r.path, odo - r.odo, lat, lon, v, self.off_route)
    if self.off_route >= OFF_ROUTE_CYCLES:
      self.route, self.off_route, self.want_detect = None, 0, True

  def _navigating(self, t: float, odo: float) -> bool:
    """Is the car on a navigation route fresh enough for its traffic, with more of it ahead than a jam prompt looks?"""
    p = self.planned
    return p is not None and p.nav and t - p.t < NAV_STALE and p.length - (odo - p.odo) > JAM_RANGE[self._fast()]

  def _follow_planned(self, odo: float, lat: float, lon: float, v: float):
    """Is the car still on the planned route? Off it, its jams are dropped at once and the road ahead is planned again."""
    p = self.planned
    if p is None:
      return
    if odo - p.odo > p.length - JAM_RANGE[self._fast()]:
      self.want_plan = True
    self.off_planned = self._off(p.path, odo - p.odo, lat, lon, v, self.off_planned)
    if self.off_planned >= OFF_ROUTE_CYCLES:
      self.planned, self.off_planned, self.want_plan = None, 0, True

  @staticmethod
  def _off(path: Path, done: float, lat: float, lon: float, v: float, count: int) -> int:
    """Updates in a row the car has been more than OFF_ROUTE_DIST off the path; slow, it is not checked."""
    if v < OFF_ROUTE_MIN_SPEED:
      return count
    on = project(path, lat, lon, done - 150., done + 150.)
    return count + 1 if on is None or on[1] > OFF_ROUTE_DIST else 0

  def _log_once(self, key: str, msg: str):
    if key not in self.logged:
      self.logged.add(key)
      self.log(msg)

  def _can_call(self, kind: str, t: float, now: float) -> bool:
    times = self.call_times[kind]
    while times and t - times[0] > 3600.:
      times.popleft()
    if kind in DAILY_BUDGET and self.daily.left(kind, now) <= 0:
      self._log_once(f"budget {kind} {self.daily.day}",
                     f"roadeventd: Baidu {kind} budget of {DAILY_BUDGET[kind]} calls used today, no more until midnight Beijing time")
      return False
    return len(times) < BUDGET_PER_HOUR[kind] and t >= self.retry_at[kind]

  def _submit(self, kind: str, t: float, now: float, path: str, params: list, meta) -> bool:
    if not self.worker.submit(kind, signed_get(path, params, self.ak, self.sk), meta):
      return False
    self.call_times[kind].append(t)
    self.calls[kind] += 1
    if kind in DAILY_BUDGET:
      self.daily.add(kind, now)
    return True

  def _schedule(self, t, odo, lat, lon, course, v, road_ahead):
    now = unix_now()
    if now < CLOCK_SET or not self.history:   # no day to count against, and the points' times would be wrong
      return

    # cruise: the speed cameras ahead, from a small daily quota
    routine = t - self.last_detect >= CRUISE_PERIOD and self.daily.left("cruise", now) >= LOW_BUDGET
    asked = self.want_detect and t - self.last_detect >= CRUISE_MIN_GAP
    parked = v < PARKED_SPEED and t - self.last_detect < CRUISE_PARKED
    if (routine or asked) and not parked and len(self.history) >= 2 and self._can_call("cruise", t, now):
      params = [("origin", f"{lat:.6f},{lon:.6f}"), ("gps_direction", str(int(round(math.degrees(course))) % 360)), ("speed", f"{v:.2f}"),
                ("radius", "10"), ("point_list", self._point_list()), ("coord_type", "wgs84"), ("ret_coordtype", "gcj02"),
                ("ext_info", str(EXT_INFO))]
      if self._submit("cruise", t, now, "/api_cruise/v1/detection", params, (t, odo, lat, lon)):
        self.last_detect, self.want_detect = t, False

    # route planning: the traffic on the road ahead, while moving
    due = t - self.last_plan >= ROUTE_PERIOD or (self.want_plan and t - self.last_plan >= ROUTE_MIN_GAP)
    if due and v >= PARKED_SPEED and not self._navigating(t, odo) and self._can_call("route", t, now):
      expected, check = self._road_ahead(t, odo, lat, lon, course, v, road_ahead)
      params = route_params(lat, lon, course, v, expected[-1])
      if self._submit("route", t, now, ROUTE_PATH, params, (t, odo, lat, lon, expected, check)):
        self.last_plan, self.want_plan = t, False

  def _road_ahead(self, t, odo, lat, lon, course, v, road_ahead) -> tuple[list[tuple[float, float]], float]:
    """The road the car is expected to take, as far as a route planning goes: the cruise route while the car follows it,
    else the offline map's road ahead, else straight on along the course; and how much of a planned route must stay on it."""
    want = min(ROUTE_AHEAD[1], max(ROUTE_AHEAD[0], JAM_RANGE[self._fast()] + v * ROUTE_PERIOD))
    r = self.route
    if r is not None and t - r.t < STALE and self.off_route == 0:
      done = odo - r.odo
      pts = [(lat, lon)] + [(la, lo) for d, la, lo in r.path if done < d <= done + want]
      if as_path(pts)[-1][0] >= ROUTE_MIN:
        return pts, ROUTE_CHECK
    if road_ahead is not None:
      pts = road_ahead(lat, lon, course, want)
      if pts and as_path(pts)[-1][0] >= ROUTE_MIN:
        return pts, ROUTE_CHECK
    return [(lat, lon), ahead_of(lat, lon, course, want)], ROUTE_CHECK_STRAIGHT

  def _point_list(self) -> str:
    """Far -> near, the last one the current point: lng,lat,0,0,0,speed,direction,radius,gps,outdoor,satellites,time."""
    now, mono = int(unix_now()), self.clock()
    return ";".join(f"{h[1]:.6f},{h[0]:.6f},0,0,0,{min(56, int(round(h[2])))},{int(round(h[3])) % 360},10,1,2,0,{now - int(round(mono - h[4]))}"
                    for h in self.history)

  # -- answers
  def _agrees(self, new: CruiseRoute) -> bool:
    """Does a new route start on the route the car is following? Stacked roads are too close to tell apart by position."""
    old = self.route
    assert old is not None
    return not new.links or not old.links or new.links[0] in old.links

  def _collect(self, t: float, odo: float):
    for kind, meta, resp, err in self.worker.results():
      if t - meta[0] > ANSWER_MAX_AGE:
        continue
      status = resp.get("status") if isinstance(resp, dict) else None
      if err is None and status is None:
        err = "BadAnswer"
      if err is not None or status != 0:
        if status in QUOTA:   # spent for today: nothing more until it starts again
          self.retry_at[kind] = t + until_beijing_midnight(unix_now())
          self._log_once(f"quota {kind}", f"roadeventd: Baidu {kind} quota spent (status {status}), no more requests until midnight Beijing time")
        else:   # network, timeouts, Baidu's own errors: again soon, then less and less often
          self.backoff[kind] = min(BACKOFF[1], max(BACKOFF[0], self.backoff[kind] * 2))
          self.retry_at[kind] = t + self.backoff[kind]
          self.want_detect = self.want_detect or kind == "cruise"
          self.want_plan = self.want_plan or kind == "route"
        continue
      self.backoff[kind] = 0.
      self.logged.discard(f"quota {kind}")
      if kind == "cruise":
        route = parse_cruise(resp, meta[1], meta[0], meta[2], meta[3])
        if route is None or not route.path:
          continue
        r = self.route
        if r is not None and t - r.t < STALE and self.off_route == 0 and odo - r.odo < r.length - self._route_end() and not self._agrees(route):
          continue   # bound to another road: keep the followed route until the car leaves it or it ages out
        self.route, self.off_route = route, 0
      elif kind == "route":
        if self._navigating(t, odo):
          continue   # asked before the navigation route came: that one has the traffic now
        planned = parse_route(resp, meta[1], meta[0], meta[2], meta[3])
        if planned is None or leaves(planned.path, meta[4], meta[5]):
          self.discarded += 1   # a detour or another road: not the traffic ahead of the car
          continue
        self.planned, self.off_planned = planned, 0

  def _snapshot(self, t, odo) -> Snapshot:
    s = Snapshot()
    r = self.route
    if r is not None and t - r.t < STALE:
      s.online = True
      done = odo - r.odo
      s.lamps = [(d - done, la, lo) for d, la, lo in r.lamps if d - done > -10.]
      ahead = next(((d - done, limit) for d, limit in r.cameras if -CAMERA_PASSED < d - done < CAMERA_RANGE), None)
      if ahead is not None:
        s.camera_distance, s.camera_limit = ahead
    p = self.planned
    if p is not None and t - p.t < (NAV_STALE if p.nav else ROUTE_STALE):
      s.jams_online = True
      done = odo - p.odo
      s.jams = [(start - done, length, st) for start, length, st in p.jams if start + length - done > 0.]
    return s
