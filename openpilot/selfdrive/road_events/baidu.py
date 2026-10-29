"""Baidu Maps online data for roadeventd: traffic jams, traffic lights and speed cameras on the road the car is predicted to
take (passenger-car cruise API, 小客车巡航API), and the speed limit of the road the car is on (road info API, 道路信息API).

Opt-in (param VwBaiduOnline) and inert without a key file: /data/baidu_ak with the lines AK=... and SK=... of a Baidu server
AK set to SN check. Not a Param, those can be backed up off the car. The keys stay in this module: requests are SN signed and
no URL is ever logged.

Network I/O runs in one worker thread with short timeouts. The 2 Hz loop only queues requests and picks up answers, so a slow
or absent network never holds it up; answers picked up late are dropped, and data older than STALE from its request too.

Calls, on demand: the cruise API's daily quota is small (2026-10-29: spent by 19:27). A detection when the car leaves the
predicted route (more than OFF_ROUTE_DIST off its polyline for OFF_ROUTE_CYCLES updates) or nears its end (ROUTE_END in
town, ROUTE_END_FAST on motorways and trunk roads), at most every CRUISE_MIN_GAP, otherwise one every CRUISE_PERIOD to keep
the traffic fresh; parked, one in CRUISE_PARKED at most; never more than BUDGET_PER_HOUR. A road info query when the road
changes and every ROADINFO_PERIOD. Errors and rate limits back off up to 5 min; a refused road info permission waits 30 min,
so road info switches on by itself once the AK gets it; a spent quota waits until midnight Beijing time and is logged once.

At a tunnel portal a detection bound the car to the surface road beside and then above the tunnel, traffic lights and all.
Positions cannot tell stacked roads apart, Baidu's link ids can: a route that does not start on a link of the route the car
is following is not taken until the car leaves that one or it ages out.

Pure python + the standard library, no openpilot imports: tests and the offline replay drive it with recorded answers.
"""
import collections
import hashlib
import json
import math
import queue
import threading
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field

KEY_FILE = "/data/baidu_ak"
HOST = "https://api.map.baidu.com"
SN_SAFE = "/:=&?#+!$,;'@()*[]"   # characters Baidu's SN rule leaves unquoted

KPH_TO_MS = 1 / 3.6
CRUISE_PERIOD = 120.       # s between routine cruise detections (tune to the AK's daily quota)
CRUISE_MIN_GAP = 20.       # s between two detections, also when the car leaves the route or nears its end
CRUISE_PARKED = 300.       # s: below PARKED_SPEED at most one detection in this time
PARKED_SPEED = 5. * KPH_TO_MS
ROADINFO_PERIOD = 60.      # s between road info queries on the same road
ROADINFO_MIN_GAP = 10.     # s
DENIED_RETRY = 1800.       # s before asking again after a refused permission
STALE = CRUISE_PARKED + 60.  # s an answer is used for, from when it was asked; longer than CRUISE_PARKED for a car in a queue
ANSWER_MAX_AGE = 10.       # s: an answer picked up later (the loop is not fed without GPS, e.g. in a tunnel) is dropped
BUDGET_PER_HOUR = {"cruise": 45, "roadinfo": 120}   # calls; under 100 cruise detections in a two hour drive
OFF_ROUTE_DIST = 40.       # m off the predicted route's polyline (GPS, lane and polyline errors stay below ~20 m)
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
DENIED = frozenset((240, 241, 4, 210, 211))  # service not enabled / no permission / IP or SN check failed
QUOTA = frozenset((301, 302))                # quota spent (401 / 402, too many requests at once, back off like errors)
NO_ROAD = 1001                               # road info: the points did not bind to a road


def unix_now() -> float:
  return time.time()  # noqa: TID251  Baidu takes UNIX seconds for the points' times


def until_beijing_midnight(now: float) -> float:
  """s from unix time now to a minute past the next midnight in Beijing (UTC+8), when Baidu's daily quota starts again."""
  return 86400. - (now + 8 * 3600.) % 86400. + 60.


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


# ---- answers -------------------------------------------------------------------------------------------------------------
@dataclass
class CruiseRoute:
  """The predicted route of one cruise detection. Distances are along it from where the car was when it was asked."""
  odo: float                                  # car odometer then, m
  t: float                                    # when it was asked
  length: float
  path: Path                                  # the route's polyline
  lamps: list[tuple[float, float, float]]     # (distance, lat, lon) of traffic lights
  jams: list[tuple[float, float, int]]        # (start, length, status 2 slow / 3 congested / 4 severe)
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
  # traffic: runs along each step, in order (their geo_cnt path segments add up to the polyline)
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
  cameras = []
  for c in r.get("camera_list") or []:
    if c.get("type") != CAMERA_SPEED or not c.get("speed_limit") or not c.get("position"):
      continue
    on = project(path, *lnglat(c["position"]))
    if on is not None and on[1] < CAMERA_OFF_PATH:
      cameras.append((on[0] - s0, float(c["speed_limit"]) * KPH_TO_MS))
  path = [(d - s0, la, lo) for d, la, lo in path]
  links = tuple(x for x in str(r.get("linkids") or "").split(",") if x)
  return CruiseRoute(odo, t, float(r.get("distance") or 0.) - s0, path, lamps, jams, sorted(cameras), links)


def parse_roadinfo(resp: dict) -> float | None:
  """Speed limit m/s of the road at the latest point, 0 when Baidu binds the road but has no limit for it; None without one."""
  if resp.get("status") != 0:
    return None
  info = resp.get("road_base_info") or {}
  try:
    return max(0., float(info.get("speed_limit") or 0.)) * KPH_TO_MS
  except (TypeError, ValueError):
    return None


@dataclass
class Snapshot:
  online: bool = False                                          # a cruise answer younger than STALE
  lamps: list = field(default_factory=list)                     # (distance ahead, lat, lon), nearest first
  jams: list = field(default_factory=list)                      # (distance ahead, length, status)
  camera_limit: float = 0.                                      # m/s, next speed camera ahead, or passed less than CAMERA_PASSED ago
  camera_distance: float = 0.                                   # m to it, negative once passed
  speed_limit: float = 0.                                       # m/s, road the car is on; 0 = none
  speed_limit_valid: bool = False                               # a fresh road info limit for this road


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
  def __init__(self, keys: tuple[str, str], worker=None, clock=time.monotonic, log=None):
    self.ak, self.sk = keys
    self.worker = worker if worker is not None else Worker()
    self.clock = clock
    self.log = log if log is not None else (lambda msg: None)
    self.quota_logged = False
    self.history: collections.deque = collections.deque(maxlen=HISTORY)   # (lat, lon, v, course deg, t)
    self.route: CruiseRoute | None = None
    self.off_route = 0
    self.last_detect = -math.inf
    self.call_times: dict[str, collections.deque] = {k: collections.deque() for k in BUDGET_PER_HOUR}
    self.want_detect = True
    self.road_key = None
    self.roadinfo: tuple[float, object, float] | None = None   # (t, road key, limit m/s)
    self.last_roadinfo = -math.inf
    self.roadinfo_denied_until = -math.inf
    self.backoff: dict[str, float] = {"cruise": 0., "roadinfo": 0.}
    self.retry_at: dict[str, float] = {"cruise": -math.inf, "roadinfo": -math.inf}
    self.calls: collections.Counter = collections.Counter()

  # -- inputs
  def update(self, odo: float, lat: float, lon: float, course: float, v: float, road: tuple | None) -> Snapshot:
    """odo: m driven since start; course: rad from north, clockwise; road: (name, OSM class) the car is matched to, or None."""
    t = self.clock()
    self._collect(t, odo)
    if not self.history or math.hypot(*ground(self.history[-1][0], self.history[-1][1], lat, lon)) > 2.:
      self.history.append((lat, lon, v, math.degrees(course) % 360., t))

    self.road_key = road
    self._follow_route(odo, lat, lon, v)

    self._schedule(t, odo, lat, lon, course, v)
    return self._snapshot(t, odo)

  def _follow_route(self, odo: float, lat: float, lon: float, v: float):
    """Is the car still on the predicted route? Off it, the route's lights, jams and cameras are dropped at once."""
    r = self.route
    if r is None:
      return
    if odo - r.odo > r.length - self._route_end():
      self.want_detect = True
    if v < OFF_ROUTE_MIN_SPEED:
      return
    done = odo - r.odo
    on = project(r.path, lat, lon, done - 150., done + 150.)
    self.off_route = self.off_route + 1 if on is None or on[1] > OFF_ROUTE_DIST else 0
    if self.off_route >= OFF_ROUTE_CYCLES:
      self.route, self.off_route, self.want_detect = None, 0, True

  def _route_end(self) -> float:
    """m before the end of the predicted route from where a new one is asked for."""
    return ROUTE_END_FAST if self.road_key is not None and self.road_key[1] in FAST_ROAD_CLASSES else ROUTE_END

  def _can_call(self, kind: str, t: float) -> bool:
    times = self.call_times[kind]
    while times and t - times[0] > 3600.:
      times.popleft()
    return len(times) < BUDGET_PER_HOUR[kind] and t >= self.retry_at[kind]

  def _submit(self, kind: str, t: float, path: str, params: list, meta) -> bool:
    if not self.worker.submit(kind, signed_get(path, params, self.ak, self.sk), meta):
      return False
    self.call_times[kind].append(t)
    self.calls[kind] += 1
    return True

  def _schedule(self, t, odo, lat, lon, course, v):
    due = t - self.last_detect >= CRUISE_PERIOD or (self.want_detect and t - self.last_detect >= CRUISE_MIN_GAP)
    parked = v < PARKED_SPEED and t - self.last_detect < CRUISE_PARKED
    if due and not parked and len(self.history) >= 2 and self._can_call("cruise", t):
      params = [("origin", f"{lat:.6f},{lon:.6f}"), ("gps_direction", str(int(round(math.degrees(course))) % 360)), ("speed", f"{v:.2f}"),
                ("radius", "10"), ("point_list", self._point_list()), ("coord_type", "wgs84"), ("ret_coordtype", "gcj02"),
                ("ext_info", str(EXT_INFO))]
      if self._submit("cruise", t, "/api_cruise/v1/detection", params, (t, odo, lat, lon)):
        self.last_detect, self.want_detect = t, False

    road_changed = self.roadinfo is not None and self.roadinfo[1] != self.road_key
    due = t - self.last_roadinfo >= ROADINFO_PERIOD or (road_changed and t - self.last_roadinfo >= ROADINFO_MIN_GAP)
    if due and t >= self.roadinfo_denied_until and self.history and self._can_call("roadinfo", t):
      now = unix_now()
      pts = [{"latitude": round(h[0], 6), "longitude": round(h[1], 6), "coord_type_input": "wgs84", "loc_time": int(now - (t - h[4])),
              "speed": round(h[2] * 3.6, 1), "direction": int(h[3])} for h in list(self.history)[-10:]]
      params = [("point_list", json.dumps(pts, separators=(",", ":"))), ("coord_type_output", "gcj02"), ("options", "vehicle_type:car")]
      if self._submit("roadinfo", t, "/api_roadinfo/v1/track", params, (t, self.road_key)):
        self.last_roadinfo = t

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
      if kind == "roadinfo" and status == NO_ROAD:
        self.roadinfo = None   # off the road network: no Baidu limit, the offline map may still have one
        continue
      if err is not None or status != 0:
        if kind == "roadinfo" and status in DENIED:
          self.roadinfo_denied_until = t + DENIED_RETRY   # not enabled for this AK (yet)
        elif status in QUOTA:   # spent for today: nothing more until it starts again
          self.retry_at[kind] = t + until_beijing_midnight(unix_now())
          if not self.quota_logged:
            self.quota_logged = True
            self.log(f"roadeventd: Baidu {kind} quota spent (status {status}), no more requests until midnight Beijing time")
        else:   # network, timeouts, Baidu's own errors: again soon, then less and less often
          self.backoff[kind] = min(BACKOFF[1], max(BACKOFF[0], self.backoff[kind] * 2))
          self.retry_at[kind] = t + self.backoff[kind]
          self.want_detect = self.want_detect or kind == "cruise"
        continue
      self.backoff[kind] = 0.
      self.quota_logged = False
      if kind == "cruise":
        route = parse_cruise(resp, meta[1], meta[0], meta[2], meta[3])
        if route is None or not route.path:
          continue
        r = self.route
        if r is not None and t - r.t < STALE and self.off_route == 0 and odo - r.odo < r.length - self._route_end() and not self._agrees(route):
          continue   # bound to another road: keep the followed route until the car leaves it or it ages out
        self.route, self.off_route = route, 0
      elif kind == "roadinfo":
        limit = parse_roadinfo(resp)
        if limit is not None:
          self.roadinfo = (meta[0], meta[1], limit)

  def _snapshot(self, t, odo) -> Snapshot:
    s = Snapshot()
    r = self.route
    if r is not None and t - r.t < STALE:
      s.online = True
      done = odo - r.odo
      s.lamps = [(d - done, la, lo) for d, la, lo in r.lamps if d - done > -10.]
      s.jams = [(start - done, length, st) for start, length, st in r.jams if start + length - done > 0.]
      ahead = next(((d - done, limit) for d, limit in r.cameras if -CAMERA_PASSED < d - done < CAMERA_RANGE), None)
      if ahead is not None:
        s.camera_distance, s.camera_limit = ahead
    if self.roadinfo is not None and t - self.roadinfo[0] < STALE and self.roadinfo[1] == self.road_key and self.roadinfo[2] > 0.:
      s.speed_limit, s.speed_limit_valid = self.roadinfo[2], True
    return s
