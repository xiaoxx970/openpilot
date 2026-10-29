import math
import os
import tempfile
import unittest
import urllib.parse

import numpy as np

from openpilot.selfdrive.road_events import baidu, osm_map
from openpilot.selfdrive.road_events.events import SOURCE_BAIDU, SOURCE_MAP, RoadEvents
from openpilot.selfdrive.road_events.prompts import EVENT_CROSSING, EVENT_JAM, EVENT_NONE, EVENT_ROUNDABOUT, HOLD, JamAhead, JamPrompt
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


def follow_route(pq):
  x, y = origin(pq)
  return east_route(x, 2000., y), None


ROADINFO_DISABLED = {"status": 240, "message": "APP 服务被禁用"}


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

  def test_road_info(self):
    limit = baidu.parse_roadinfo({"status": 0, "road_base_info": {"road_name": "Main Rd", "road_grade": 3, "speed_limit": 60}})
    assert limit is not None
    self.assertAlmostEqual(limit, 60 / 3.6)
    self.assertEqual(baidu.parse_roadinfo({"status": 0, "road_base_info": {"speed_limit": 0}}), 0.)
    self.assertIsNone(baidu.parse_roadinfo(ROADINFO_DISABLED))


class TestBaiduOnline(unittest.TestCase):
  def setUp(self):
    self.start()

  def start(self, auto=None):
    self.clock = Clock()
    self.worker = FakeWorker(auto)
    self.online = baidu.BaiduOnline(("ak-test", "sk-test"), self.worker, self.clock)
    self.odo, self.prev = 0., None

  def step(self, x, y=0., course=EAST, v=15., road=MAIN) -> baidu.Snapshot:
    """One 2 Hz cycle of roadeventd with the car at (x, y)."""
    if self.prev is not None:
      self.odo += math.hypot(x - self.prev[0], y - self.prev[1])
    self.prev = (x, y)
    snap = self.online.update(self.odo, *latlon(x, y), course, v, road)
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

  def detection_times(self, cycles: int, at) -> list[float]:
    times = []
    for i in range(cycles):
      n = len(self.sent())
      self.step(*at(i))
      if len(self.sent()) > n:
        times.append(self.clock.t - 0.5)
    return times

  def test_a_detection_every_30_s_along_the_route(self):
    self.start({"cruise": follow_route, "roadinfo": lambda pq: (ROADINFO_DISABLED, None)})
    snaps = [self.step(7.5 * i) for i in range(140)]   # 70 s at 15 m/s
    self.assertEqual(len(self.sent()), 3)              # 0.5, 30.5 and 60.5 s
    self.assertTrue(all(s.online for s in snaps[2:]))
    self.assertEqual(len(self.sent("roadinfo")), 1)    # not enabled for the AK: asked again only after 30 min
    for pq in self.sent() + self.sent("roadinfo"):
      self.assertTrue("&sn=" in pq)
      self.assertNotIn("sk-test", pq)

  def test_leaving_the_route_drops_it_and_asks_again(self):
    self.begin(800.)
    for i in range(2, 41):
      self.assertTrue(self.step(7.5 * i).online)       # to 300 m east, 20 s
    self.assertEqual(len(self.sent()), 1)
    for i in range(1, 21):                             # then north, off the route
      snap = self.step(300., 7.5 * i, course=0.)
      if len(self.sent()) == 2:
        break
    self.assertLessEqual(7.5 * i, 60.)                 # 40 m off it, two cycles in a row
    self.assertFalse(snap.online)                      # its lights and jams are not for this road

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
    self.worker.answer("cruise", line_route([(x, 0.) for x in np.arange(-32.5, 2000., 40.)], "101,102,103"))
    x = 7.5
    while len(self.sent()) < 2:                        # to the 30 s detection
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
    self.begin(400.)
    for i in range(2, 60):
      self.step(7.5 * i)
      if len(self.sent()) == 2:
        break
    self.assertAlmostEqual(7.5 * i, 7.5 + 400. - baidu.ROUTE_END, delta=7.5)

  def test_errors_back_off(self):
    self.start({"cruise": lambda pq: (None, "URLError")})
    times = self.detection_times(400, lambda i: (3. * i, 0., EAST, 6.))
    self.assertEqual([b - a for a, b in zip(times, times[1:], strict=False)][:4], [5.5, 10.5, 20.5, 40.5])

  def test_rate_limit_backs_off_like_errors(self):
    self.start({"cruise": lambda pq: ({"status": 401, "message": "当前并发量已经超过约定并发配额，限制访问"}, None)})
    times = self.detection_times(40, lambda i: (3. * i, 0., EAST, 6.))
    self.assertEqual(times[:2], [0.5, 6.])

  def test_quota_waits_half_an_hour(self):
    self.start({"cruise": lambda pq: ({"status": 302, "message": "天配额超限，限制访问"}, None)})
    times = self.detection_times(600, lambda i: (3. * i, 0., EAST, 6.))
    self.assertEqual(times, [0.5])
    self.assertAlmostEqual(self.online.retry_at["cruise"], 1. + baidu.DENIED_RETRY)

  def test_hourly_budget(self):
    # two parallel roads 100 m apart, 6 s on each: every jump leaves the route, a detection every 6 s
    self.start({"cruise": follow_route})
    times = self.detection_times(int(4000 / 0.5), lambda i: (0., 100. * ((i // 12) % 2), EAST, 5.))
    most = max(sum(1 for u in times if t <= u < t + 3600.) for t in times)
    self.assertEqual(most, baidu.BUDGET_PER_HOUR["cruise"])
    self.assertGreater(len(times), most)                 # and on again in the next hour

  def test_snapshot_along_the_route(self):
    self.begin(800.)
    for i in range(2, 14):
      snap = self.step(7.5 * i)                          # 97.5 m east
    self.assertAlmostEqual(snap.lamps[0][0], 220. - 97.5, delta=1.)   # Baidu gives whole meters
    self.assertEqual(len(snap.lamps), 3)
    (start, length, status), = snap.jams
    self.assertEqual(status, 2)
    self.assertLess(start, 0.)                           # the car is in it: not ahead
    self.assertAlmostEqual(start + length, 220. - 97.5, delta=0.5)
    self.assertAlmostEqual(snap.camera_distance, 500. - 97.5, delta=0.5)
    self.assertAlmostEqual(snap.camera_limit, 60 / 3.6)
    while self.clock.t < 62.:                            # the 30 s detection is never answered
      snap = self.step(97.5, v=0.)
    self.assertFalse(snap.online)

  def test_speed_limit_for_the_road_it_was_asked_on(self):
    self.step(0.)
    self.worker.answer("roadinfo", {"status": 0, "road_base_info": {"road_name": "Main Rd", "speed_limit": 60}})
    snap = self.step(7.5)
    self.assertTrue(snap.speed_limit_valid)
    self.assertAlmostEqual(snap.speed_limit, 60 / 3.6)
    self.assertFalse(self.step(15., road=("Other Rd", 5)).speed_limit_valid)
    while len(self.sent("roadinfo")) < 2:
      self.step(15., road=("Other Rd", 5))
    self.worker.answer("roadinfo", {"status": 0, "road_base_info": {"road_name": "Other Rd", "speed_limit": 0}})
    self.assertFalse(self.step(15., road=("Other Rd", 5)).speed_limit_valid)   # bound, but Baidu has no limit for it
    while len(self.sent("roadinfo")) < 3:
      self.step(15., road=("Other Rd", 5))
    self.worker.answer("roadinfo", {"status": 1001, "message": "绑路失败"})
    self.step(15., road=("Other Rd", 5))
    self.assertIsNone(self.online.roadinfo)


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
      return baidu.Snapshot(online=True, lamps=[(150. - x, *latlon(150., 0.))], jams=[(200. - x, 200., 3)])
    out = None
    for i, x in enumerate(self.xs[:50]):
      out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, FLAT, jam(x))
      if out["event"] != EVENT_NONE:
        break
    assert out is not None
    self.assertEqual((out["event"], out["source"], out["jamLength"]), (EVENT_JAM, SOURCE_BAIDU, 200.))
    self.assertAlmostEqual(x, -300., delta=8.)   # 500 m before it in town

  def test_no_jam_prompt_in_the_queue(self):
    core = RoadEvents(self.tmp.name)
    snap = baidu.Snapshot(online=True, jams=[(300., 200., 3)])
    out = [core.update(i * 0.5, 2., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, snap)["event"] for i in range(10)]
    self.assertEqual(set(out), {EVENT_NONE})
    self.assertEqual(core.update(5., 15., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, snap)["event"], EVENT_JAM)

  def test_relaxed_distance_for_the_approach(self):
    core = RoadEvents(self.tmp.name)
    on = []
    for i, x in enumerate(-400. + 7.5 * i for i in range(90)):   # to x = 267.5, a jam from x = 200
      snap = baidu.Snapshot(online=True, jams=[(200. - x, 300., 3)])
      out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, FLAT, snap)
      if out["jamFollow"]:
        on.append(x)
        self.assertAlmostEqual(out["jamFollowDistance"], 200. - x, delta=0.1)
    self.assertAlmostEqual(on[0], -300., delta=8.)   # with the prompt, 500 m before it in town
    self.assertAlmostEqual(on[-1], 200., delta=8.)   # until the car reaches it
    self.assertEqual(len(on), 67)                    # without a gap

  def test_relaxed_distance_held_without_a_position_not_with_baidu_off(self):
    core = RoadEvents(self.tmp.name)
    jam = baidu.Snapshot(online=True, jams=[(400., 300., 3)])
    self.assertTrue(core.update(0., 15., 0., 0., EAST, *latlon(-300., 1.), True, FLAT, jam)["jamFollow"])
    self.assertTrue(core.update(10., 15., 0., 0., EAST, *latlon(-150., 1.), False, FLAT, None)["jamFollow"])
    self.assertFalse(core.update(10.5, 15., 0., 0., EAST, *latlon(-142., 1.), True, FLAT, baidu.Snapshot())["jamFollow"])

  def test_slow_traffic_counts_on_fast_roads(self):
    def slow(x):
      return baidu.Snapshot(online=True, jams=[(1000., 300., 2)])

    starts = drive(RoadEvents(self.tmp.name), self.xs[:10], slow)
    self.assertEqual(starts, [])
    with tempfile.TemporaryDirectory() as tmp:
      build_tile(tmp, stacked=True)            # the car is on the trunk above Main Rd
      starts = drive(RoadEvents(tmp), self.xs[:10], slow, y=4.)
    self.assertEqual([e for _, e, _ in starts], [EVENT_JAM])


if __name__ == "__main__":
  unittest.main()
