import math
import os
import tempfile
import unittest

import numpy as np

from openpilot.selfdrive.road_events import osm_map
from openpilot.selfdrive.road_events.events import RoadEvents
from openpilot.selfdrive.road_events.osm_map import OfflineMap, tile_key, tile_path
from openpilot.selfdrive.road_events.prompts import EVENT_CROSSING, EVENT_DOWNHILL, EVENT_NONE, EVENT_ROUNDABOUT, EVENT_UPHILL, HOLD, \
  MapPrompt, SlopePrompt
from openpilot.selfdrive.road_events.slope import grades_ahead, slope_ahead

T_IDXS = 10. * (np.arange(33) / 32.) ** 2
LAT0, LON0 = 32.1, 118.6
KX = osm_map.M_PER_DEG * math.cos(math.radians(LAT0))
EAST = math.pi / 2   # course


def latlon(x, y) -> tuple[float, float]:
  return float(LAT0 + y / osm_map.M_PER_DEG), float(LON0 + x / KX)


def car_frame_path(world_xyz, pitch, roll=0.):
  """World points (x fwd, y right, z up, car at the origin) -> modelV2's calibrated frame (z down) of a car at that attitude."""
  cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])           # body -> world for the NED-like frame, z down
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  ned = np.column_stack([world_xyz[:, 0], world_xyz[:, 1], -world_xyz[:, 2]])
  body = ned @ (ry @ rx)                                          # = (R^T p) row-wise
  return body[:, 0], body[:, 1], body[:, 2]


class TestSlope(unittest.TestCase):
  def test_flat_then_uphill(self):
    s = T_IDXS * 15.
    z = np.where(s > 80., (s - 80.) * 0.05, 0.)
    grades = grades_ahead(s, np.zeros(33), -z, 0., 0.)
    self.assertAlmostEqual(np.nanmax(grades), 0.05, delta=0.002)
    sign, dist = slope_ahead(grades, 0.03)
    self.assertEqual(sign, 1)
    self.assertTrue(70. <= dist <= 110., dist)

  def test_downhill_and_flat(self):
    s = T_IDXS * 15.
    self.assertEqual(slope_ahead(grades_ahead(s, np.zeros(33), np.zeros(33), 0., 0.), 0.03), (0, 0.))
    sign, _ = slope_ahead(grades_ahead(s, np.zeros(33), np.where(s > 50., (s - 50.) * 0.04, 0.), 0., 0.), 0.03)
    self.assertEqual(sign, -1)

  def test_turn_on_a_constant_grade_reads_the_grade(self):
    # hairpin on an 8 % climb: the world grade stays 8 %, relative to the tilted car frame it would read up to 16 %
    g, r = 0.08, 25.
    s = T_IDXS * 10.
    theta = s / r
    world = np.column_stack([r * np.sin(theta), r * (1 - np.cos(theta)) * -1, s * g])   # turning left, climbing
    pitch = math.atan(g)
    x, y, z = car_frame_path(world, pitch)
    grades = grades_ahead(x, y, z, 0., pitch)
    finite = grades[np.isfinite(grades)]
    np.testing.assert_allclose(finite, g, atol=0.004)
    # what the model's own frame says: the road after the turn looks 16 % steeper than under the car
    relative = -np.diff(z) / np.hypot(np.diff(x), np.diff(y))
    self.assertGreater(relative[np.cumsum(np.hypot(np.diff(x), np.diff(y))) > r * math.pi].max(), 0.12)

  def test_roll_does_not_make_a_slope(self):
    s = T_IDXS * 15.
    world = np.column_stack([s, np.zeros(33), np.zeros(33)])
    x, y, z = car_frame_path(world, 0., roll=0.05)
    self.assertEqual(slope_ahead(grades_ahead(x, y, z, 0.05, 0.), 0.03)[0], 0)


class TestSlopePrompt(unittest.TestCase):
  def run_seq(self, seq, dt=0.5):
    p, out = SlopePrompt(), []
    for i, (grade_now, sign) in enumerate(seq):
      out.append(p.update(i * dt, grade_now, sign, 120.)[0])
    return out

  def test_fires_after_a_second_from_flat_and_holds(self):
    out = self.run_seq([(0., 1)] * 30)
    first = out.index(EVENT_UPHILL)
    self.assertEqual(first, 2)                                   # 1 s at 2 Hz
    self.assertEqual(out[first:first + int(HOLD / 0.5)], [EVENT_UPHILL] * int(HOLD / 0.5))
    self.assertEqual(set(out[first + int(HOLD / 0.5):]), {EVENT_NONE})   # still the same slope ahead: no repeat

  def test_not_on_a_slope(self):
    self.assertEqual(set(self.run_seq([(0.04, 1)] * 20 + [(-0.03, -1)] * 20)), {EVENT_NONE})

  def test_a_glimpse_is_not_enough(self):
    self.assertEqual(set(self.run_seq([(0., -1), (0., 0)] * 20)), {EVENT_NONE})

  def test_next_slope_after_flat_road_again(self):
    seq = [(0., -1)] * 12 + [(-0.05, 0)] * 20 + [(0., 0)] * 6 + [(0., -1)] * 6
    out = self.run_seq(seq)
    starts = [i for i in range(1, len(out)) if out[i] == EVENT_DOWNHILL and out[i - 1] != EVENT_DOWNHILL]
    self.assertEqual(len(starts), 2)


class TestMapPrompt(unittest.TestCase):
  def test_window_and_dedup(self):
    p = MapPrompt(EVENT_ROUNDABOUT, 30., 300.)
    self.assertEqual(p.update(0., (350., LAT0, LON0))[0], EVENT_NONE)      # too far
    self.assertEqual(p.update(0.5, (290., LAT0, LON0))[0], EVENT_ROUNDABOUT)
    self.assertEqual(p.update(0.5 + HOLD, (200., LAT0, LON0))[0], EVENT_NONE)
    lat, lon = latlon(30., 0.)                                             # same roundabout, another node
    self.assertEqual(p.update(1. + HOLD, (190., lat, lon))[0], EVENT_NONE)
    lat, lon = latlon(500., 0.)                                            # the next one
    self.assertEqual(p.update(2. + HOLD, (250., lat, lon))[0], EVENT_ROUNDABOUT)

  def test_first_seen_too_close(self):
    p = MapPrompt(EVENT_CROSSING, 50., 250.)
    self.assertEqual(p.update(0., (40., LAT0, LON0))[0], EVENT_NONE)


def build_tile(root, stacked=False, main_class=5, dual_crossing=False, slip_class=6):
  """A town east of (LAT0, LON0): Main Rd (primary) west to east with a crossing primary at x=150, a residential side
  street at x=80, a slip road at x=200, a roundabout of radius 20 m centered at x=300 in four one-way quarters, and Main Rd
  on beyond it. stacked: an elevated trunk 4 m north of Main Rd. main_class: highway class of Main Rd and Cross Rd;
  slip_class: of the slip road.
  dual_crossing: two dual carriageways crossing at (600, 300), their four one-way pieces inside the junction ways of their
  own, as often mapped."""
  from openpilot.selfdrive.road_events.osm_map import _offline_schema
  ways = []

  def way(pts, cls, name="", ref="", oneway=False):
    ways.append(([latlon(x, y) for x, y in pts], cls, name, ref, oneway))
  if dual_crossing:
    # pieces first: the greedy chaining then closes their square, as it did on real tiles
    se, ne, nw, sw = (615., 285.), (615., 315.), (585., 315.), (585., 285.)
    for a, b in ((se, ne), (ne, nw), (nw, sw), (sw, se)):
      way([a, b], 5, "", "", True)
    way([(615., 100.), se], 5, "North Ave", "", True)
    way([ne, (615., 500.)], 5, "North Ave", "", True)
    way([(585., 500.), nw], 5, "North Ave", "", True)
    way([sw, (585., 100.)], 5, "North Ave", "", True)
    way([(800., 315.), ne], 5, "West St", "", True)
    way([nw, (400., 315.)], 5, "West St", "", True)
    way([(400., 285.), sw], 5, "West St", "", True)
    way([se, (800., 285.)], 5, "West St", "", True)
  xs = np.arange(-400., 281., 10.)
  way([(x, 0.) for x in xs], main_class, "Main Rd")
  way([(150., y) for y in np.arange(-200., 201., 20.)], main_class, "Cross Rd")
  way([(80., y) for y in np.arange(0., 151., 25.)], 12, "Side St")
  way([(200., 0.), (215., 30.), (230., 60.)], slip_class)
  quarter = [np.radians(np.linspace(a, a + 90., 7)) for a in (180., 270., 360., 450.)]
  for q in quarter:
    way([(300. + 20. * math.cos(a), 20. * math.sin(a)) for a in q], 5, "", "", True)
  way([(x, 0.) for x in np.arange(320., 801., 20.)], 5, "Main Rd")
  if stacked:
    way([(x, 4.) for x in np.arange(-400., 801., 50.)], 3, "Ring Expy", "", False)

  schema = _offline_schema()
  msg = schema.Offline.new_message()
  key = tile_key(LAT0, LON0)
  msg.minLat, msg.minLon, msg.maxLat, msg.maxLon, msg.overlap = key[0], key[1], key[0] + 0.25, key[1] + 0.25, 0.001
  lst = msg.init("ways", len(ways))
  for i, (pts, cls, name, ref, oneway) in enumerate(ways):
    w = lst[i]
    w.id, w.name, w.ref, w.oneWay, w.highwayClass = i + 1, name, ref, bool(oneway), int(cls)
    nodes = w.init("nodes", len(pts))
    for k, (la, lo) in enumerate(pts):
      nodes[k].latitude, nodes[k].longitude = la, lo
    w.minLat, w.maxLat = min(p[0] for p in pts), max(p[0] for p in pts)
    w.minLon, w.maxLon = min(p[1] for p in pts), max(p[1] for p in pts)
  path = tile_path(root, key)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(msg.to_bytes_packed())


class TestRampNear(unittest.TestCase):
  def test_a_ramp_junction_on_the_motorway_300_m_ahead_200_m_behind(self):
    # Main Rd a motorway, its slip road at x = 200 a motorway link; a primary road gets nothing
    for main_class, want in ((1, {-200.: False, -50.: True, 150.: True, 250.: True}), (5, dict.fromkeys((-200., -50., 150., 250.), False))):
      with tempfile.TemporaryDirectory() as tmp:
        build_tile(tmp, main_class=main_class, slip_class=2)
        core = RoadEvents(tmp)
        near = {x: core.update(0., 25., 0., 0., EAST, *latlon(x, 0.), True, None)["rampNear"] for x in want}
        self.assertEqual(near, want)


class TestOfflineMap(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    build_tile(self.tmp.name)
    self.map = OfflineMap(self.tmp.name)
    self.assertTrue(self.map.ensure(LAT0, LON0))

  def tearDown(self):
    self.tmp.cleanup()

  def test_roundabout_found_from_geometry(self):
    rb = set(self.map.roundabout)
    self.assertEqual(len(rb), 4)
    lat, lon, r = self.map.roundabout[next(iter(rb))]
    self.assertAlmostEqual(r, 20., delta=1.5)
    self.assertAlmostEqual((lon - LON0) * KX, 300., delta=1.)

  def test_dual_carriageways_crossing_is_no_roundabout(self):
    with tempfile.TemporaryDirectory() as tmp:
      build_tile(tmp, dual_crossing=True)
      m = OfflineMap(tmp)
      m.ensure(LAT0, LON0)
      self.assertEqual(len(m.roundabout), 4)                     # the roundabout's quarters, not the square
      self.assertTrue(all(abs(r - 20.) < 1.5 for _, _, r in m.roundabout.values()))

  def test_features_ahead(self):
    m = self.map.match(*latlon(20., 1.5), EAST)
    assert m is not None
    self.assertEqual(self.map.way_name[m.way], "Main Rd")
    roundabout, crossing = self.map.features_ahead(m)
    self.assertAlmostEqual(crossing.distance, 130., delta=1.)      # Cross Rd, not the side street or the slip road
    self.assertAlmostEqual(roundabout.distance, 260., delta=1.)

  def test_wrong_way_on_a_one_way_road_is_no_match(self):
    m = self.map.match(*latlon(300., -20.), -EAST)   # bottom of the roundabout, against its direction
    self.assertTrue(m is None or m.way not in self.map.roundabout)

  def test_in_the_roundabout_no_roundabout_prompt(self):
    m = self.map.match(*latlon(300., -20.), EAST)
    assert m is not None
    self.assertTrue(m.way in self.map.roundabout)
    roundabout, crossing = self.map.features_ahead(m)
    self.assertIsNone(roundabout)
    self.assertIsNone(crossing)

  def test_no_crossings_on_minor_roads(self):
    # two residential streets crossing: on minor roads every junction is one of equal class
    tmp = tempfile.TemporaryDirectory()
    build_tile(tmp.name, main_class=12)
    m = OfflineMap(tmp.name)
    m.ensure(LAT0, LON0)
    match = m.match(*latlon(20., 1.5), EAST)
    assert match is not None
    self.assertEqual(m.way_name[match.way], "Main Rd")
    self.assertIsNone(m.features_ahead(match)[1])
    tmp.cleanup()

  def test_stacked_roads_take_the_higher_class(self):
    tmp = tempfile.TemporaryDirectory()
    build_tile(tmp.name, stacked=True)
    m = OfflineMap(tmp.name)
    m.ensure(LAT0, LON0)
    match = m.match(*latlon(20., 1.5), EAST)
    assert match is not None
    self.assertEqual(m.way_name[match.way], "Ring Expy")
    self.assertIsNone(m.features_ahead(match)[1])      # Cross Rd is a lower class than the trunk
    tmp.cleanup()


class TestNoMapHere(unittest.TestCase):
  def test_area_without_tiles(self):
    with tempfile.TemporaryDirectory() as tmp:
      m = OfflineMap(tmp)
      self.assertTrue(m.ensure(LAT0, LON0))
      self.assertIsNone(m.match(LAT0, LON0, EAST))
      core = RoadEvents(tmp)
      self.assertEqual(core.update(0., 15., 0., 0., EAST, LAT0, LON0, True, None)["event"], EVENT_NONE)


class TestRoadEvents(unittest.TestCase):
  def test_drive_to_the_roundabout(self):
    with tempfile.TemporaryDirectory() as tmp:
      build_tile(tmp)
      core = RoadEvents(tmp)
      flat = (T_IDXS * 15., np.zeros(33), np.zeros(33))
      events = []
      for i in range(int(380 / 7.5)):
        x = -350. + 7.5 * i                  # 15 m/s, 2 Hz
        out = core.update(i * 0.5, 15., 0., 0., EAST, *latlon(x, 1.), True, flat)
        events.append((x, out["event"]))
      starts = [(x, e) for k, (x, e) in enumerate(events) if e != EVENT_NONE and (k == 0 or events[k - 1][1] != e)]
      self.assertEqual([e for _, e in starts], [EVENT_CROSSING, EVENT_ROUNDABOUT])
      self.assertAlmostEqual(starts[0][0], -100., delta=8.)   # crossing at 150 m enters its 250 m window
      self.assertAlmostEqual(starts[1][0], -20., delta=8.)    # roundabout entry at 280 m enters its 300 m window

  def test_no_map_still_gives_slopes(self):
    core = RoadEvents(None)
    s = T_IDXS * 15.
    up = (s, np.zeros(33), -np.where(s > 60., (s - 60.) * 0.06, 0.))
    out = [core.update(i * 0.5, 15., 0., 0., EAST, LAT0, LON0, False, up)["event"] for i in range(6)]
    self.assertTrue(EVENT_UPHILL in out)


if __name__ == "__main__":
  unittest.main()
