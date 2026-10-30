import json
import math
import os
import tempfile
import time
import unittest
import urllib.parse
from unittest import mock

import numpy as np

from openpilot.selfdrive.road_events import baidu, osm_map
from openpilot.selfdrive.road_events.events import SOURCE_BAIDU, SOURCE_MAP, RoadEvents
from openpilot.selfdrive.road_events.prompts import EVENT_CROSSING, EVENT_JAM, EVENT_NONE, EVENT_ROUNDABOUT, EVENT_SPEED_LIMIT_AHEAD, HOLD, \
  CameraPrompt, JamAhead, JamPrompt
from openpilot.selfdrive.road_events.tests.test_road_events import EAST, KX, LAT0, LON0, T_IDXS, build_tile, latlon

MAIN = ("Main Rd", 5)
FLAT = (T_IDXS * 15., np.zeros(33), np.zeros(33))

# a road along y = 0 east of (LAT0, LON0), as Baidu's cruise API describes it
WORLD_LAMPS = (220., 600., 740., 1300.)
WORLD_CAMERAS = ((15., 0., 1, 60), (300., 0., 4, 0), (400., 60., 1, 40), (500., 0., 1, 60), (1200., 0., 1, 50))   # x, y, type, km/h
WORLD_TRAFFIC = ((-1000., 220., 2), (220., 1000., 1), (1000., 1250., 3), (1250., 5000., 1))                       # from x, to x, status
LINK_BACK = 40.   # m: Baidu starts its route, and its distances, at the start of the link the car is on


def gcj(x: float, y: float) -> str:
  """Baidu's "lng,lat" (GCJ-02) of a point x m east, y m north of (LAT0, LON0)."""
  lat, lon = latlon(x, y)
  dlat, dlon = baidu._gcj_delta(lat, lon)
  return f"{lon + dlon:.6f},{lat + dlat:.6f}"


def east_route(x0: float = 0., length: float = 800., y: float = 0.) -> dict:
  """A detection answer for a car at (x0, y), shaped like a recorded one: the route east from LINK_BACK m behind the car to
  length m ahead of it in two steps sharing their joint point, with the lights, speed cameras and traffic runs of WORLD_*,
  and a second, less likely candidate turning north."""
  start = x0 - LINK_BACK
  xs = np.arange(start, x0 + length + 1e-6, 40.)
  half = len(xs) // 2
  steps = []
  for part in (xs[:half + 1], xs[half:]):
    a, b = float(part[0]), float(part[-1])
    traffic = [{"status": st, "geo_cnt": max(1, int((min(hi, b) - max(lo, a)) // 40)), "distance": round(min(hi, b) - max(lo, a), 2)}
               for lo, hi, st in WORLD_TRAFFIC if min(hi, b) > max(lo, a)]
    steps.append({"link_num": len(part) - 1, "path": ";".join(gcj(x, y) for x in part), "distance": round(b - a), "duration": 60,
                  "traffic_condition": traffic, "guidance": {}})
  lamps = [{"linkid": 1530353204, "link_start": gcj(x - 50., y), "link_end": gcj(x, y), "out_linkid": 1569422956, "pos": gcj(x, y),
            "distance": round(x - start), "key": "15303532040_15694229560_1"} for x in WORLD_LAMPS if x0 < x < x0 + length]
  cameras = [{"type": kind, "speed_limit": limit, "priority": 0, "position": gcj(x, y + cy), "linkid": 1576030826, "offset": 15}
             for x, cy, kind, limit in WORLD_CAMERAS if start <= x < x0 + length]
  north = {"distance": 300, "steps": [{"path": f"{gcj(x0, y)};{gcj(x0, y + 300.)}", "distance": 300,
                                       "traffic_condition": [{"status": 4, "geo_cnt": 1, "distance": 300.}]}]}
  links = ",".join(dict.fromkeys(f"{int(y)}{int((x + 1000.) // 200.):04d}" for x in xs))   # a link every 200 m
  route = {"route_md5": "c4aaf54f", "distance": round(x0 + length - start), "duration": 1800, "linkids": links,
           "camera_list": cameras, "lamps": lamps, "steps": steps}
  return {"status": 0, "navi_err": 0, "message": "成功", "result": {"total": 2, "session_id": "{}", "routes": [route, north]}}


def line_route(points: list[tuple[float, float]], links: str, lamps: tuple[float, ...] = ()) -> dict:
  """A one-step detection answer along the polyline points (x, y) and links, with lights at the given distances along it."""
  s = [0.]
  for (x1, y1), (x2, y2) in zip(points, points[1:], strict=False):
    s.append(s[-1] + math.hypot(x2 - x1, y2 - y1))

  def at(d: float) -> tuple[float, float]:
    k = max(i for i in range(len(s) - 1) if s[i] <= d)
    u = (d - s[k]) / (s[k + 1] - s[k])
    return points[k][0] + u * (points[k + 1][0] - points[k][0]), points[k][1] + u * (points[k + 1][1] - points[k][1])
  route = {"distance": round(s[-1]), "linkids": links, "camera_list": [], "lamps": [{"pos": gcj(*at(d)), "distance": round(d)} for d in lamps],
           "steps": [{"path": ";".join(gcj(x, y) for x, y in points), "distance": round(s[-1]), "traffic_condition": []}]}
  return {"status": 0, "message": "成功", "result": {"routes": [route]}}


def origin(path_and_query: str) -> tuple[float, float]:
  """(x, y) m of a detection's origin."""
  query = urllib.parse.parse_qs(urllib.parse.urlsplit(path_and_query).query)
  lat, lon = (float(v) for v in query["origin"][0].split(","))
  return (lon - LON0) * KX, (lat - LAT0) * osm_map.M_PER_DEG


class Clock:
  def __init__(self):
    self.t = 0.

  def __call__(self) -> float:
    return self.t


class FakeWorker:
  """Worker without a thread or network: answers come from `auto` (kind -> function of the request) or from answer()."""
  def __init__(self, auto=None):
    self.auto = auto or {}
    self.sent: list[tuple[str, str, object]] = []
    self.answers: list[tuple] = []
    self.busy: set[str] = set()

  def submit(self, kind, path_and_query, meta) -> bool:
    if kind in self.busy:
      return False
    self.busy.add(kind)
    self.sent.append((kind, path_and_query, meta))
    if kind in self.auto:
      self.answer(kind, *self.auto[kind](path_and_query))
    return True

  def answer(self, kind, resp=None, err=None):
    meta = next(m for k, _, m in reversed(self.sent) if k == kind)
    self.answers.append((kind, meta, resp, err))

  def results(self):
    while self.answers:
      kind, meta, resp, err = self.answers.pop(0)
      self.busy.discard(kind)
      yield kind, meta, resp, err


def follow_route(pq, length: float = 2000.):
  x, y = origin(pq)
  return east_route(x, length, y), None


def follow_long_route(pq):
  return follow_route(pq, 4000.)


def destination(path_and_query: str) -> tuple[float, float]:
  """(x, y) m of a route planning's destination."""
  query = urllib.parse.parse_qs(urllib.parse.urlsplit(path_and_query).query)
  lat, lon = (float(v) for v in query["destination"][0].split(","))
  return (lon - LON0) * KX, (lat - LAT0) * osm_map.M_PER_DEG


def planned(points: list[tuple[float, float]], traffic: list[tuple[float, int]]) -> dict:
  """A driving route planning answer along the polyline points (x, y), traffic (length, status) runs from its start."""
  length = sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(points, points[1:], strict=False))
  step = {"road_name": "Main Rd", "distance": round(length), "path": ";".join(gcj(x, y) for x, y in points),
          "traffic_condition": [{"status": st, "geo_cnt": 1, "distance": round(d, 2)} for d, st in traffic]}
  return {"status": 0, "message": "成功", "result": {"routes": [{"distance": round(length), "duration": 300, "steps": [step]}]}}


def plan_straight(pq):
  """Route planning along the straight line from the origin to the destination, with WORLD_TRAFFIC where it runs east on
  y = 0, free flow elsewhere."""
  (x0, y0), (x1, y1) = origin(pq), destination(pq)
  n = max(1, int(math.hypot(x1 - x0, y1 - y0) // 40.))
  points = [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]
  if abs(y0) < 1. and abs(y1) < 1. and x1 > x0:
    traffic = [(min(hi, x1) - max(lo, x0), st) for lo, hi, st in WORLD_TRAFFIC if min(hi, x1) > max(lo, x0)]
  else:
    traffic = [(math.hypot(x1 - x0, y1 - y0), 1)]
  return planned(points, traffic), None


class TestRequests(unittest.TestCase):
  def test_sn_matches_baidus_example(self):
    pq = baidu.signed_get("/geocoder/v2/", [("address", "百度大厦"), ("output", "json")], "yourak", "yoursk")
    self.assertTrue(pq.endswith("&sn=7de5a22212ffaa9e326444c75a58f9a0"))
    self.assertNotIn("yoursk", pq)

  def test_key_file(self):
    with tempfile.TemporaryDirectory() as tmp:
      path = os.path.join(tmp, "baidu_ak")
      self.assertIsNone(baidu.load_keys(path))
      with open(path, "w") as f:
        f.write("AK=abc\n")
      self.assertIsNone(baidu.load_keys(path))
      with open(path, "w") as f:
        f.write("AK=abc\nSK=def\n")
      self.assertEqual(baidu.load_keys(path), ("abc", "def"))


class TestAnswers(unittest.TestCase):
  def test_cruise_detection(self):
    r = baidu.parse_cruise(east_route(), 100., 5., *latlon(0., 0.))
    assert r is not None
    self.assertEqual((r.odo, r.t), (100., 5.))
    # distances from the car, not from the start of its link
    self.assertAlmostEqual(r.length, 800., delta=0.5)
    self.assertAlmostEqual(r.path[0][0], -LINK_BACK, delta=0.5)
    self.assertAlmostEqual(r.path[-1][0], 800., delta=1.)
    self.assertEqual([round(d) for d, _, _ in r.lamps], [220, 600, 740])
    lat, lon = latlon(220., 0.)
    self.assertLess(math.hypot(*baidu.ground(lat, lon, *r.lamps[0][1:])), 0.5)   # GCJ-02 back to WGS-84
    # speed cameras with a limit on the route: not the one without a limit, not the one on the road 60 m north
    self.assertEqual([(round(d), round(v * 3.6)) for d, v in r.cameras], [(15, 60), (500, 60)])
    # status 1 is free flow; the candidate turning north is not used
    self.assertEqual([(round(a), round(b), st) for a, b, st in r.jams], [(-40, 260, 2)])

  def test_failed_detections(self):
    here = latlon(0., 0.)
    self.assertIsNone(baidu.parse_cruise({"status": 2, "message": "参数错误, 必须传入轨迹点序列"}, 0., 0., *here))
    self.assertIsNone(baidu.parse_cruise({"status": 0, "result": {"routes": []}}, 0., 0., *here))
    self.assertIsNone(baidu.parse_cruise(east_route(), 0., 0., *latlon(0., 100.)))   # bound to a road 100 m away


class TestBaiduOnline(unittest.TestCase):
  def setUp(self):
    self.start()

  def start(self, auto=None):
    self.clock = Clock()
    self.worker = FakeWorker(auto)
    self.online = baidu.BaiduOnline(("ak-test", "sk-test"), self.worker, self.clock)
    self.odo, self.prev = 0., None

  def step(self, x, y=0., course=EAST, v=15., road=MAIN, road_ahead=None, nav=None) -> baidu.Snapshot:
    """One 2 Hz cycle of roadeventd with the car at (x, y)."""
    if self.prev is not None:
      self.odo += math.hypot(x - self.prev[0], y - self.prev[1])
    self.prev = (x, y)
    snap = self.online.update(self.odo, *latlon(x, y), course, v, road, road_ahead, nav)
    self.clock.t += 0.5
    return snap

  def sent(self, kind="cruise") -> list[str]:
    return [pq for k, pq, _ in self.worker.sent if k == kind]

  def begin(self, length: float):
    """The first two cycles, 7.5 m apart: the cruise API needs two points. Its first detection gets the route east."""
    self.step(0.)
    self.assertEqual(self.sent(), [])
    self.step(7.5)
    self.assertEqual(len(self.sent()), 1)
    self.worker.answer("cruise", east_route(7.5, length))

  def detection_times(self, cycles: int, at, kind="cruise") -> list[float]:
    times = []
    for i in range(cycles):
      n = len(self.sent(kind))
      self.step(*at(i))
      if len(self.sent(kind)) > n:
        times.append(self.clock.t - 0.5)
    return times

  # -- cruise: speed cameras
  def test_a_routine_detection_every_cruise_period_along_the_route(self):
    self.start({"cruise": follow_long_route})
    times = self.detection_times(1240, lambda i: (2.5 * i, 0., EAST, 5.))   # 620 s at 5 m/s, the route never near its end
    self.assertEqual(times, [0.5, 0.5 + baidu.CRUISE_PERIOD, 0.5 + 2 * baidu.CRUISE_PERIOD])
    self.assertTrue(self.step(2.5 * 1240, v=5.).online)
    self.assertEqual(self.sent("roadinfo"), [])        # Baidu's road info API is not used: companies only
    for pq in self.sent() + self.sent("route"):
      self.assertTrue("&sn=" in pq)
      self.assertNotIn("sk-test", pq)

  def test_leaving_the_route_drops_it_and_asks_again(self):
    self.begin(2000.)
    for i in range(2, 122):
      self.assertTrue(self.step(7.5 * i).online)       # to 907.5 m east, 60.5 s
    self.assertEqual(len(self.sent()), 1)
    for i in range(1, 31):                             # then north, off the route
      snap = self.step(907.5, 7.5 * i, course=0.)
      if len(self.sent()) == 2:
        break
    self.assertEqual(i, 15)                            # 40 m off it (from 45 m, i = 6) for 5 s
    self.assertFalse(snap.online)                      # its cameras are not for this road

  def test_a_short_jump_off_the_route_keeps_it(self):
    # a GPS jump or a parallel lane: off the route for less than 5 s
    self.begin(2000.)
    for i in range(2, 21):
      self.step(7.5 * i)
    for i in range(21, 21 + baidu.OFF_ROUTE_CYCLES - 1):
      self.assertTrue(self.step(7.5 * i, 60.).online)
    for i in range(30, 60):
      self.assertTrue(self.step(7.5 * i).online)
    self.assertEqual(len(self.sent()), 1)

  def test_a_new_road_name_alone_does_not_ask_again(self):
    # a tunnel portal: a new detection there bound the car to the surface road beside the tunnel
    self.begin(2000.)
    for i in range(2, 21):
      self.step(7.5 * i)
    self.step(157.5, road=("大校场隧道", 3))
    self.step(165., road=("", 4))
    self.assertEqual(len(self.sent()), 1)

  def test_the_other_branch_of_a_split_is_not_taken(self):
    # a tunnel portal: the car keeps to the tunnel (y = 0), a new detection binds it to the surface road beside it, 12 m to
    # the left and then right above the tunnel, with a traffic light
    self.step(0.)
    self.step(7.5)
    self.worker.answer("cruise", line_route([(x, 0.) for x in np.arange(-32.5, 6000., 40.)], "101,102,103"))
    x = 7.5
    while len(self.sent()) < 2:                        # to the routine detection, far from the route's end
      x += 7.5
      self.step(x)
    surface = [(x - 40., 12.), (x + 100., 12.), (x + 200., 3.), (x + 400., 3.)]
    self.worker.answer("cruise", line_route(surface, "201,202,203", lamps=(100.,)))
    snap = self.step(x + 7.5)
    self.assertEqual(snap.lamps, [])                   # not the surface road's light
    self.assertTrue(snap.online)                       # the tunnel's route stays
    for k in range(2, 12):
      self.assertEqual(self.step(x + 7.5 * k).lamps, [])
    self.assertEqual(len(self.sent()), 2)              # no extra detections for it

  def test_a_late_answer_is_dropped(self):
    # asked at a tunnel portal, picked up at the exit: roadeventd does not feed the client without GPS
    self.step(0.)
    self.step(7.5)
    self.worker.answer("cruise", east_route(7.5, 800.))
    self.clock.t += 20.
    self.assertFalse(self.step(307.5).online)

  def test_near_the_end_of_the_route_asks_again(self):
    # 300 m before its end in town, 1000 m on motorways: 30 m a cycle, both before the routine detection
    for road, end in ((MAIN, baidu.ROUTE_END), (("G2503", 1), baidu.ROUTE_END_FAST)):
      self.start()
      self.step(0., road=road)
      self.step(15., road=road)
      self.worker.answer("cruise", east_route(15., 3000.))
      for i in range(2, 240):
        self.step(15. * i, v=30., road=road)
        if len(self.sent()) == 2:
          break
      self.assertAlmostEqual(15. * i, 15. + 3000. - end, delta=15.)
      self.assertLess(self.clock.t, baidu.CRUISE_PERIOD)

  def test_not_again_within_the_min_gap(self):
    self.begin(100.)                                   # its end comes up at once
    times = self.detection_times(140, lambda i: (7.5 + 7.5 * i,))
    self.assertEqual(times, [0.5 + baidu.CRUISE_MIN_GAP])

  def test_errors_back_off(self):
    # again soon (no closer than CRUISE_MIN_GAP), then less and less often
    self.start({"cruise": lambda pq: (None, "URLError")})
    times = self.detection_times(1600, lambda i: (3. * i, 0., EAST, 6.))
    self.assertEqual([b - a for a, b in zip(times, times[1:], strict=False)][:6], [60., 60., 60., 60., 80.5, 160.5])

  def test_rate_limit_backs_off_like_errors(self):
    self.start({"cruise": lambda pq: ({"status": 401, "message": "当前并发量已经超过约定并发配额，限制访问"}, None)})
    times = self.detection_times(160, lambda i: (3. * i, 0., EAST, 6.))
    self.assertEqual(times[:2], [0.5, 0.5 + baidu.CRUISE_MIN_GAP])

  def test_spent_quota_waits_until_midnight_in_beijing_and_logs_once(self):
    spent = {"status": 302, "message": "天配额超限，限制访问"}
    answer = {"cruise": spent}
    logged: list[str] = []
    self.clock = Clock()
    self.worker = FakeWorker({k: (lambda pq, k=k: (answer[k], None)) for k in answer})
    self.online = baidu.BaiduOnline(("ak-test", "sk-test"), self.worker, self.clock, logged.append)
    self.odo, self.prev = 0., None
    evening = 1793273220.   # 2026-10-29 19:27 in Beijing, when the owner's AK ran out; the clock's 0
    midnight = (4 * 60 + 33) * 60 + 60.   # to 00:01
    with mock.patch.object(baidu, "unix_now", lambda: evening + self.clock.t):
      times = self.detection_times(600, lambda i: (3. * i, 0., EAST, 6.))
      self.assertEqual(times, [0.5])
      self.assertAlmostEqual(self.online.retry_at["cruise"], midnight, delta=0.01)
      self.assertEqual(len(logged), 1)                 # once
      self.assertTrue(all("sk-test" not in line and "ak-test" not in line for line in logged))

      self.clock.t = midnight                          # answered again the next day
      answer["cruise"] = east_route(1800., 2000.)
      self.assertEqual(len(self.detection_times(4, lambda i: (1800. + 3. * i, 0., EAST, 6.))), 1)
      self.assertTrue(self.step(1812.).online)
      self.assertEqual(len(logged), 1)
      answer["cruise"] = spent                         # and spent again later that day: logged again
      self.clock.t += baidu.CRUISE_PERIOD
      self.detection_times(4, lambda i: (1900. + 3. * i, 0., EAST, 6.))
      self.assertEqual(len(logged), 2)

  def test_parked_at_most_one_detection_in_cruise_parked(self):
    self.start({"cruise": follow_long_route})
    self.step(0.)
    self.step(7.5)
    plans = len(self.sent("route"))
    times = self.detection_times(1400, lambda i: (10., 0., EAST, 1.))   # 700 s at a standstill
    self.assertEqual(times, [0.5 + baidu.CRUISE_PARKED, 0.5 + 2 * baidu.CRUISE_PARKED])
    self.assertTrue(self.step(10., v=0.).online)       # its data stays until the next one
    self.assertEqual(len(self.sent("route")), plans)   # no route planning standing still

  def test_hourly_budget(self):
    # two parallel roads 100 m apart, 6 s on each: every jump leaves the route, a detection every minute
    self.start({"cruise": follow_route})
    times = self.detection_times(int(4000 / 0.5), lambda i: (0., 100. * ((i // 12) % 2), EAST, 5.))
    most = max(sum(1 for u in times if t <= u < t + 3600.) for t in times)
    self.assertEqual(most, baidu.BUDGET_PER_HOUR["cruise"])
    self.assertGreater(len(times), most)                 # and on again in the next hour

  def test_snapshot_along_the_route(self):
    self.start({"route": plan_straight})
    self.begin(800.)
    for i in range(2, 14):
      snap = self.step(7.5 * i)                          # 97.5 m east
    self.assertAlmostEqual(snap.lamps[0][0], 220. - 97.5, delta=1.)   # Baidu gives whole meters
    self.assertEqual(len(snap.lamps), 3)
    self.assertAlmostEqual(snap.camera_distance, 500. - 97.5, delta=0.5)
    self.assertAlmostEqual(snap.camera_limit, 60 / 3.6)
    # the jams come from route planning
    self.assertTrue(snap.jams_online)
    (start, length, status), (start2, length2, status2) = snap.jams
    self.assertEqual((status, status2), (2, 3))
    self.assertLess(start, 0.)                           # the car is in it: not ahead
    self.assertAlmostEqual(start + length, 220. - 97.5, delta=1.)
    self.assertAlmostEqual(start2, 1000. - 97.5, delta=1.)
    while self.clock.t < baidu.STALE + 1.:               # parked, its next detection and plannings are never answered
      self.worker.auto.clear()
      snap = self.step(97.5, v=0.)
    self.assertFalse(snap.online or snap.jams_online)
    self.assertEqual(len(self.sent()), 2)

  def test_a_speed_camera_is_reported_until_it_is_50_m_behind(self):
    self.begin(800.)                                     # a 60 km/h camera at x = 15
    self.assertAlmostEqual(self.step(15.).camera_distance, 0., delta=0.5)
    snap = self.step(15. + baidu.CAMERA_PASSED - 2.)
    self.assertAlmostEqual(snap.camera_distance, 2. - baidu.CAMERA_PASSED, delta=0.5)
    self.assertAlmostEqual(snap.camera_limit, 60 / 3.6)
    snap = self.step(15. + baidu.CAMERA_PASSED + 2.)
    self.assertAlmostEqual(snap.camera_distance, 500. - 15. - baidu.CAMERA_PASSED - 2., delta=0.5)   # the next one

  # -- daily budget
  def test_daily_budget_kept_across_drives_until_the_next_day(self):
    with tempfile.TemporaryDirectory() as tmp:
      path = os.path.join(tmp, "baidu_calls")
      day = 1793241600.   # 2026-10-29 10:40 in Beijing
      with mock.patch.object(baidu, "unix_now", lambda: day + self.clock.t):
        count = baidu.DailyCount(path)
        for _ in range(3):
          count.add("cruise", day)
        count.add("route", day)
        for _ in range(100):                           # its writes run in threads of their own
          if os.path.exists(path) and json.load(open(path))["calls"] == {"cruise": 3, "route": 1}:
            break
          time.sleep(0.01)
        again = baidu.DailyCount(path)                 # the next drive
        self.assertEqual((again.today("cruise", day), again.today("route", day)), (3, 1))
        self.assertEqual(again.left("cruise", day), baidu.DAILY_BUDGET["cruise"] - 3)
        midnight = day + baidu.until_beijing_midnight(day)
        self.assertEqual(again.today("cruise", midnight), 0)
        self.assertEqual(again.left("cruise", midnight), baidu.DAILY_BUDGET["cruise"])
      with open(path, "w") as f:
        f.write("not json")
      self.assertEqual(baidu.DailyCount(path).today("cruise", day), 0)

  def test_no_routine_detections_when_the_budget_runs_low(self):
    self.start({"cruise": follow_long_route})
    self.online.daily.day = baidu.beijing_day(baidu.unix_now())
    self.online.daily.counts["cruise"] = baidu.DAILY_BUDGET["cruise"] - baidu.LOW_BUDGET - 1
    times = self.detection_times(700, lambda i: (2.5 * i, 0., EAST, 5.))
    self.assertEqual(times, [0.5, 0.5 + baidu.CRUISE_PERIOD])   # the first one, and a routine one with 20 left
    times = self.detection_times(700, lambda i: (1750. + 2.5 * i, 0., EAST, 5.))   # 19 left: only when the route asks
    self.assertEqual(times, [])
    for i in range(1, 30):                             # off it
      self.step(3497.5, 7.5 * i, course=0.)
    self.assertEqual(len(self.sent()), 3)

  def test_budget_used_up_stops_and_logs_once(self):
    logged: list[str] = []
    self.online = baidu.BaiduOnline(("ak-test", "sk-test"), self.worker, self.clock, logged.append)
    self.online.daily.day = baidu.beijing_day(baidu.unix_now())
    self.online.daily.counts["cruise"] = baidu.DAILY_BUDGET["cruise"]
    for i in range(400):
      self.step(7.5 * i)
    self.assertEqual(self.sent(), [])
    self.assertEqual(len([line for line in logged if "cruise" in line]), 1)

  def test_no_calls_before_the_clock_is_set(self):
    with mock.patch.object(baidu, "unix_now", lambda: 1e6):
      for i in range(40):
        self.step(7.5 * i)
    self.assertEqual(self.worker.sent, [])

  # -- route planning: the traffic ahead
  def test_route_planning_every_minute_while_moving(self):
    self.start({"route": plan_straight})
    times = self.detection_times(300, lambda i: (7.5 * i,), "route")   # 150 s at 15 m/s
    self.assertEqual(times, [0., baidu.ROUTE_PERIOD, 2 * baidu.ROUTE_PERIOD])
    x, y = destination(self.sent("route")[0])
    self.assertAlmostEqual(x, baidu.ROUTE_AHEAD[0], delta=1.)   # straight on, without a cruise route or the map
    self.assertAlmostEqual(y, 0., delta=1.)
    snap = self.step(7.5 * 300)
    self.assertTrue(snap.jams_online)
    self.assertEqual(self.online.discarded, 0)

  def test_route_planning_asks_along_the_road_ahead(self):
    # the cruise route while the car follows it, else the map's road ahead, else straight on; farther at speed on fast roads
    self.start({"route": plan_straight})
    self.begin(4000.)
    self.step(15.)
    self.clock.t += baidu.ROUTE_PERIOD
    self.step(22.5, v=30., road=("G2503", 1))
    x, y = destination(self.sent("route")[-1])
    self.assertAlmostEqual(x - 22.5, baidu.JAM_RANGE[1] + 30. * baidu.ROUTE_PERIOD, delta=40.)
    curve = [latlon(30. + 10. * k, 0.2 * k * k) for k in range(300)]   # the map's road bends north
    self.start({"route": plan_straight})
    self.step(0., road_ahead=lambda *a: curve)
    x, y = destination(self.sent("route")[-1])
    self.assertAlmostEqual(y, 0.2 * 299 ** 2, delta=1.)

  def test_a_planned_route_off_the_road_ahead_is_not_used(self):
    # a detour, the other carriageway, the road above: it leaves the expected road within its first 300 m (100 m when only
    # the course straight on is known)
    def detour(turn):
      def answer(pq):
        (x0, y0), (x1, y1) = origin(pq), destination(pq)
        return planned([(x0, y0), (x0 + turn, y0), (x0 + turn, y0 + 200.), (x1, y1 + 200.), (x1, y1)], [(5000., 4)]), None
      return answer
    for turn, along_the_road, used in ((40., False, False), (150., False, True), (150., True, False)):
      self.start({"route": detour(turn)})
      road = [latlon(10. * k, 0.) for k in range(300)]
      self.step(0., road_ahead=(lambda *a, road=road: road) if along_the_road else None)
      snap = self.step(7.5)
      self.assertEqual(snap.jams_online, used)
      self.assertEqual(self.online.discarded, 0 if used else 1)
      if not used:
        self.assertEqual(snap.jams, [])

  def test_no_route_planning_while_stopped(self):
    self.start({"route": plan_straight})
    for _ in range(400):
      self.step(0., v=1.)
    self.assertEqual(self.sent("route"), [])

  # -- navigation: navd's route brings the traffic
  def nav_east(self, x0: float, x1: float, jams, y: float = 0.):
    """navd's route east along y from x0 to x1 m, asked now, with jams (start, length, status) from its first point."""
    return self.clock.t, [latlon(x, y) for x in np.arange(x0, x1 + 1e-6, 40.)], jams

  def test_no_route_planning_while_navigating(self):
    self.start({"route": plan_straight})
    nav = self.nav_east(0., 8000., [(2000., 300., 3)])
    for i in range(300):                               # 150 s at 15 m/s
      snap = self.step(7.5 * i, nav=nav)
    self.assertEqual(self.sent("route"), [])
    self.assertTrue(snap.jams_online)
    self.assertEqual(len(snap.jams), 1)
    self.assertAlmostEqual(snap.jams[0][0], 2000. - 7.5 * 299, delta=2.)
    self.assertEqual(snap.jams[0][1:], (300., 3))

  def test_planning_again_once_the_navigation_traffic_is_old(self):
    self.start({"route": plan_straight})
    nav = self.nav_east(0., 8000., [])
    times = self.detection_times(400, lambda i: (7.5 * i, 0., EAST, 15., MAIN, None, nav), "route")   # 200 s, no new route
    self.assertEqual(times, [baidu.NAV_STALE])

  def test_a_new_navigation_route_replaces_a_planning(self):
    self.start({"route": plan_straight})
    self.step(0.)
    self.assertEqual(len(self.sent("route")), 1)
    snap = self.step(7.5, nav=self.nav_east(7.5, 8000., [(500., 100., 4)]))
    self.assertTrue(snap.online is False and snap.jams_online)
    self.assertAlmostEqual(snap.jams[0][0], 500., delta=2.)

  def test_a_navigation_route_away_from_the_car_is_not_used(self):
    self.start({"route": plan_straight})
    nav = self.nav_east(0., 8000., [(500., 100., 4)], y=200.)
    times = self.detection_times(250, lambda i: (7.5 * i, 0., EAST, 15., MAIN, None, nav), "route")
    self.assertEqual(times, [0., baidu.ROUTE_PERIOD, 2 * baidu.ROUTE_PERIOD])

  def test_leaving_the_navigation_route_plans_at_once(self):
    self.start({"route": plan_straight})
    nav = self.nav_east(0., 8000., [(1000., 100., 4)])
    for i in range(61):
      self.step(7.5 * i, nav=nav)
    self.assertEqual(self.sent("route"), [])
    for k in range(1, 20):                             # north, off the route
      snap = self.step(450., 7.5 * k, course=0., nav=nav)
      if self.sent("route"):
        break
    self.assertEqual(k, 15)                            # 45 m off from k = 6, for 10 cycles
    self.assertEqual(snap.jams, [])

  def test_leaving_the_road_drops_cameras_and_jams_at_once(self):
    # turned off at 30 s: 5 s after the car is 40 m off, the cameras and jams of the road it left are gone; route planning
    # asks at once for the new road (20 s gap), the cruise API after its 60 s gap
    self.start({"cruise": follow_long_route, "route": plan_straight})
    for i in range(61):                                # 30 s east at 15 m/s, to x = 450
      snap = self.step(7.5 * i)
    self.assertTrue(snap.online and snap.jams_online)
    self.assertGreater(snap.camera_limit, 0.)
    self.assertTrue(snap.jams)
    plans, detections = len(self.sent("route")), len(self.sent())
    for k in range(1, 40):                             # north
      snap = self.step(450., 7.5 * k, course=0.)
      if not snap.online:
        break
    self.assertEqual(k, 15)                            # 45 m off from k = 6, for 10 cycles
    self.assertEqual((snap.camera_limit, snap.jams), (0., []))
    self.assertFalse(snap.jams_online)
    self.assertEqual(len(self.sent("route")), plans + 1)   # at once
    self.assertEqual(len(self.sent()), detections)        # the cruise API only 60 s after its last detection
    snap = self.step(450., 7.5 * 16, course=0.)
    self.assertTrue(snap.jams_online)                  # the new road's traffic, a cycle later
    self.assertEqual(snap.jams, [])
    t_off, y = self.clock.t, 7.5 * 16
    while len(self.sent()) == detections:
      y += 7.5
      snap = self.step(450., y, course=0.)
    self.assertAlmostEqual(self.clock.t - 0.5, 0.5 + baidu.CRUISE_MIN_GAP, delta=0.01)
    self.assertLess(self.clock.t - t_off, baidu.CRUISE_MIN_GAP)

  def test_jam_range_as_the_prompt(self):
    self.assertEqual(baidu.JAM_RANGE, (JamPrompt.NEAR, JamPrompt.NEAR_FAST))


class TestJamPrompt(unittest.TestCase):
  def test_in_town_congestion_within_500_m(self):
    self.assertEqual(JamPrompt.nearest([(400., 150., 3)], fast=False), (400., 150.))
    self.assertIsNone(JamPrompt.nearest([(400., 150., 2)], fast=False))   # slow traffic only counts on fast roads
    self.assertIsNone(JamPrompt.nearest([(600., 150., 3)], fast=False))
    self.assertIsNone(JamPrompt.nearest([(400., 80., 4)], fast=False))
    self.assertIsNone(JamPrompt.nearest([(-50., 300., 4)], fast=False))   # already in it

  def test_fast_roads_slow_traffic_within_1500_m(self):
    self.assertEqual(JamPrompt.nearest([(1400., 200., 2)], fast=True), (1400., 200.))
    self.assertIsNone(JamPrompt.nearest([(1600., 200., 3)], fast=True))

  def test_adjoining_runs_join(self):
    self.assertEqual(JamPrompt.nearest([(300., 60., 3), (360., 60., 4)], fast=False), (300., 120.))
    self.assertIsNone(JamPrompt.nearest([(300., 60., 3), (380., 60., 4)], fast=False))

  def test_once_per_jam(self):
    p, jam = JamPrompt(), [(400., 200., 3)]
    out = [p.update(i * 0.5, jam, False)[0] for i in range(40)]
    n = int(HOLD / 0.5)
    self.assertEqual(out[:n], [EVENT_JAM] * n)
    self.assertEqual(set(out[n:]), {EVENT_NONE})
    self.assertEqual(p.update(20.5 + JamPrompt.REARM - 1., [], False)[0], EVENT_NONE)
    self.assertEqual(p.update(21. + JamPrompt.REARM - 1., jam, False)[0], EVENT_NONE)   # not clear for REARM s
    for i in range(int(JamPrompt.REARM / 0.5) + 1):
      p.update(50. + i * 0.5, [], False)
    self.assertEqual(p.update(81., jam, False)[0], EVENT_JAM)


  def test_a_prompt_ends_once_the_jam_data_is_gone(self):
    p, jam = JamPrompt(), [(400., 200., 3)]
    self.assertEqual(p.update(0., jam, False)[0], EVENT_JAM)
    self.assertEqual(p.update(0.5, [], False)[0], EVENT_JAM)              # in the hold without it, e.g. a GPS gap
    self.assertEqual(p.update(1., [], False, gone=True)[0], EVENT_NONE)   # the car left the planned road
    self.assertEqual(p.update(1.5, jam, False)[0], EVENT_NONE)            # once per jam


class TestJamAhead(unittest.TestCase):
  def test_on_for_the_jams_the_prompt_announces(self):
    self.assertEqual(JamAhead().update(0., [(400., 150., 3)], False, True), (True, 400.))
    self.assertEqual(JamAhead().update(0., [(1400., 200., 2)], True, True), (True, 1400.))
    for jams, fast in (([(400., 150., 2)], False), ([(600., 150., 3)], False), ([(400., 80., 4)], False), ([(1600., 200., 3)], True)):
      self.assertEqual(JamAhead().update(0., jams, fast, True), (False, 0.))

  def test_comes_on_only_while_moving(self):
    a, jam = JamAhead(), [(400., 150., 3)]
    self.assertFalse(a.update(0., jam, False, False)[0])
    self.assertTrue(a.update(0.5, jam, False, True)[0])
    self.assertTrue(a.update(1., [(380., 150., 3)], False, False)[0])   # slowing down on the approach keeps it

  def test_a_jam_start_moving_back_a_little_keeps_it(self):
    a = JamAhead()
    a.update(0., [(480., 150., 3)], False, True)
    self.assertEqual(a.update(30., [(700., 150., 3)], False, True), (True, 700.))   # within KEEP times 500 m
    self.assertEqual(a.update(60., [(800., 150., 3)], False, True), (False, 0.))
    self.assertFalse(a.update(60.5, [(700., 150., 3)], False, True)[0])            # on again only within 500 m

  def test_off_once_in_the_jam(self):
    a = JamAhead()
    a.update(0., [(100., 300., 3)], False, True)
    self.assertEqual(a.update(10., [(-5., 300., 3), (450., 200., 3)], False, True), (False, 0.))   # a next one too
    self.assertEqual(a.update(40., [(150., 200., 3)], False, True), (True, 150.))   # out of it, the next one ahead

  def test_off_without_jams_held_without_data(self):
    a, jam = JamAhead(), [(400., 150., 3)]
    a.update(0., jam, False, True)
    self.assertEqual(a.update(JamAhead.NO_DATA_HOLD - 1., None, False, True), (True, 400.))   # no GPS in a tunnel
    self.assertEqual(a.update(JamAhead.NO_DATA_HOLD + 0.5, None, False, True), (False, 0.))
    a.update(40., jam, False, True)
    self.assertEqual(a.update(40.5, [], False, True), (False, 0.))   # Baidu off, stale or the jam gone


class TestCameraPrompt(unittest.TestCase):
  @staticmethod
  def run_cameras(cameras, v=15., cycles=200, jitter=None):
    """(cycle, event, distance, limit) of every cycle with a prompt, driving at v past cameras [(odometer m, km/h)] that
    are reported from CAMERA_RANGE before them until CAMERA_PASSED after, as baidu.Snapshot does."""
    p, shown = CameraPrompt(), []
    for i in range(cycles):
      odo = v * 0.5 * i
      ahead = next(((at - odo, kph / 3.6) for at, kph in cameras if -baidu.CAMERA_PASSED < at - odo < baidu.CAMERA_RANGE), (0., 0.))
      d = ahead[0] + (jitter(i) if jitter else 0.)
      event, distance, limit = p.update(i * 0.5, v, d, ahead[1], True)
      if event != EVENT_NONE:
        shown.append((i, event, distance, limit))
    return shown

  def test_once_per_camera_12_s_ahead_for_5_s(self):
    for v, near in ((15., CameraPrompt.NEAR), (40., 12. * 40.), (80., CameraPrompt.FAR)):
      shown = self.run_cameras([(1000., 60)], v, cycles=int(2 * 1100. / v))
      self.assertEqual(len(shown), int(CameraPrompt.CAMERA_HOLD / 0.5))
      first = shown[0]
      self.assertEqual(first[1], EVENT_SPEED_LIMIT_AHEAD)
      self.assertAlmostEqual(first[3], 60 / 3.6)
      self.assertLessEqual(first[2], near)
      self.assertGreater(first[2], near - v * 0.5)
      self.assertEqual([i for i, *_ in shown], list(range(first[0], first[0] + len(shown))))
      self.assertAlmostEqual(shown[-1][2], first[2] - (len(shown) - 1) * v * 0.5, delta=0.01)   # counting down

  def test_placed_again_a_little_off_is_the_same_camera(self):
    shown = self.run_cameras([(1000., 60)], jitter=lambda i: 25. * ((i // 7) % 2))
    self.assertEqual(len(shown), int(CameraPrompt.CAMERA_HOLD / 0.5))

  def test_the_next_camera_and_a_new_limit(self):
    shown = self.run_cameras([(500., 60), (900., 80), (1300., 80)], cycles=220)
    starts = [(i, round(limit * 3.6)) for k, (i, _, _, limit) in enumerate(shown) if k == 0 or shown[k - 1][0] != i - 1]
    self.assertEqual([kph for _, kph in starts], [60, 80, 80])

  def test_a_prompt_ends_once_its_camera_is_no_longer_reported(self):
    # the car left the road (its cruise route dropped) or the data went stale; not in a gap without GPS
    for reported, shown in ((True, 2), (False, int(CameraPrompt.CAMERA_HOLD / 0.5))):
      p = CameraPrompt()
      events = [p.update(i * 0.5, 15., 290. - 7.5 * i, 60 / 3.6, True)[0] for i in range(2)]
      events += [p.update(i * 0.5, 15., 0., 0., True, reported=reported)[0] for i in range(2, 12)]
      self.assertEqual(sum(e == EVENT_SPEED_LIMIT_AHEAD for e in events), shown)

  def test_not_while_stopped(self):
    # first reported while the car stands in a queue 100 m before it: the prompt comes once the car moves again
    p = CameraPrompt()
    self.assertEqual({p.update(i * 0.5, 0., 100., 60 / 3.6, False)[0] for i in range(40)}, {EVENT_NONE})
    shown = [p.update(i * 0.5, 5., 100. - 2.5 * (i - 40), 60 / 3.6, True) for i in range(40, 70)]
    self.assertEqual(shown[0], (EVENT_SPEED_LIMIT_AHEAD, 100., 60 / 3.6))
    self.assertEqual(sum(e == EVENT_SPEED_LIMIT_AHEAD for e, _, _ in shown), int(CameraPrompt.CAMERA_HOLD / 0.5))


def drive(core: RoadEvents, xs, snapshot, y=1.):
  """Prompt starts (x, event, source) driving east at 15 m/s; snapshot(x) is Baidu's data there."""
  starts, last = [], EVENT_NONE
  for i, x in enumerate(xs):
    out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, y), True, FLAT, snapshot(x))
    if out["event"] != EVENT_NONE and out["event"] != last:
      starts.append((x, out["event"], out["source"]))
    last = out["event"]
  return starts


def lights(*xs):
  return lambda x: baidu.Snapshot(online=True, lamps=[(lx - x, *latlon(lx, 0.)) for lx in xs if lx - x > -10.])


class TestBaiduPrompts(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    build_tile(self.tmp.name)   # Main Rd east with Cross Rd at x = 150 and a roundabout at x = 280..320
    self.xs = [-400. + 7.5 * i for i in range(int(420 / 7.5))]

  def tearDown(self):
    self.tmp.cleanup()

  def test_crossings_stay_with_the_map(self):
    # Baidu's lights do not make crossing prompts, with or without lights on the road ahead
    for snapshot in (lights(-120., 150.), lights(), lambda x: baidu.Snapshot(online=False)):
      starts = drive(RoadEvents(self.tmp.name), self.xs, snapshot)
      self.assertEqual([(e, s) for _, e, s in starts], [(EVENT_CROSSING, SOURCE_MAP), (EVENT_ROUNDABOUT, SOURCE_MAP)])
      self.assertAlmostEqual(starts[0][0], -100., delta=8.)   # the map's Cross Rd at x = 150

  def test_a_jam_comes_first(self):
    core = RoadEvents(self.tmp.name)

    def jam(x):
      return baidu.Snapshot(online=True, jams_online=True, lamps=[(150. - x, *latlon(150., 0.))], jams=[(200. - x, 200., 3)])
    out = None
    for i, x in enumerate(self.xs[:50]):
      out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, FLAT, jam(x))
      if out["event"] != EVENT_NONE:
        break
    assert out is not None
    self.assertEqual((out["event"], out["source"], out["jamLength"]), (EVENT_JAM, SOURCE_BAIDU, 200.))
    self.assertAlmostEqual(x, -300., delta=8.)   # 500 m before it in town

  def test_a_speed_camera_comes_first(self):
    core = RoadEvents(self.tmp.name)
    out = None
    for i, x in enumerate(self.xs[:50]):
      snap = baidu.Snapshot(online=True, jams_online=True, jams=[(200. - x, 200., 3)], camera_limit=60 / 3.6, camera_distance=-100. - x)
      out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, FLAT, snap)
      if out["event"] != EVENT_NONE:
        break
    assert out is not None
    self.assertEqual((out["event"], out["source"]), (EVENT_SPEED_LIMIT_AHEAD, SOURCE_BAIDU))
    self.assertAlmostEqual(out["eventSpeedLimit"], 60 / 3.6)
    self.assertAlmostEqual(out["distance"], CameraPrompt.NEAR, delta=8.)
    self.assertAlmostEqual(x, -400., delta=8.)
    stale = baidu.Snapshot(online=False, camera_limit=60 / 3.6, camera_distance=200.)
    out = RoadEvents(self.tmp.name).update(0., 15., 0., 0., EAST, *latlon(0., 1.), True, FLAT, stale)
    self.assertNotEqual(out["event"], EVENT_SPEED_LIMIT_AHEAD)
    self.assertEqual(out["eventSpeedLimit"], 0.)

  def test_no_jam_prompt_in_the_queue(self):
    core = RoadEvents(self.tmp.name)
    snap = baidu.Snapshot(online=True, jams_online=True, jams=[(300., 200., 3)])
    out = [core.update(i * 0.5, 2., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, snap)["event"] for i in range(10)]
    self.assertEqual(set(out), {EVENT_NONE})
    self.assertEqual(core.update(5., 15., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, snap)["event"], EVENT_JAM)

  def test_relaxed_distance_for_the_approach(self):
    core = RoadEvents(self.tmp.name)
    on = []
    for i, x in enumerate(-400. + 7.5 * i for i in range(90)):   # to x = 267.5, a jam from x = 200
      snap = baidu.Snapshot(online=True, jams_online=True, jams=[(200. - x, 300., 3)])
      out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, FLAT, snap)
      if out["jamFollow"]:
        on.append(x)
        self.assertAlmostEqual(out["jamFollowDistance"], 200. - x, delta=0.1)
    self.assertAlmostEqual(on[0], -300., delta=8.)   # with the prompt, 500 m before it in town
    self.assertAlmostEqual(on[-1], 200., delta=8.)   # until the car reaches it
    self.assertEqual(len(on), 67)                    # without a gap

  def test_relaxed_distance_held_without_a_position_not_with_baidu_off(self):
    core = RoadEvents(self.tmp.name)
    jam = baidu.Snapshot(online=True, jams_online=True, jams=[(400., 300., 3)])
    self.assertTrue(core.update(0., 15., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, jam)["jamFollow"])
    self.assertTrue(core.update(10., 15., 0., 0., EAST, *latlon(-150., 1.), False, FLAT, None)["jamFollow"])
    self.assertFalse(core.update(10.5, 15., 0., 0., EAST, *latlon(-142., 1.), True, FLAT, baidu.Snapshot())["jamFollow"])

  def test_slow_traffic_counts_on_fast_roads(self):
    def slow(x):
      return baidu.Snapshot(online=True, jams_online=True, jams=[(1000., 300., 2)])

    starts = drive(RoadEvents(self.tmp.name), self.xs[:10], slow)
    self.assertEqual(starts, [])
    with tempfile.TemporaryDirectory() as tmp:
      build_tile(tmp, stacked=True)            # the car is on the trunk above Main Rd
      starts = drive(RoadEvents(tmp), self.xs[:10], slow, y=4.)
    self.assertEqual([e for _, e, _ in starts], [EVENT_JAM])


if __name__ == "__main__":
  unittest.main()
