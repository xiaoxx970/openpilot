import time
import unittest
from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ramp_sign_filter
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_sign_filter import KEEP, RampSignFilter
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.common.test import OpenpilotTestCase

KPH = 1 / 3.6


class Car:
  """plannerd's SubMaster around the car: on a motorway (OSM class 1) near a ramp junction, a lane to its right."""
  def __init__(self):
    self.services = ['roadEventsIC', 'carStateSP', 'carState', 'modelV2', 'liveMapDataSP', 'gpsLocation']
    self.seen = dict.fromkeys(self.services, True)
    self.recv_time = dict.fromkeys(self.services, time.monotonic())
    self.road = SimpleNamespace(roadClass=1, mapMatched=True, rampNear=True, speedLimit=0., speedLimitValid=False, speedLimitAhead=0.,
                                speedLimitAheadDistance=0.)
    self.reading = 0.
    self.right_blinker = False
    self.gap = 6.        # m from the right lane line to the right road edge: a lane to the right
    self.edge_std = 0.5

  def __getitem__(self, key):
    if key == 'roadEventsIC':
      return self.road
    if key == 'carStateSP':
      return SimpleNamespace(speedLimit=self.reading)
    if key == 'carState':
      return SimpleNamespace(rightBlinker=self.right_blinker, vCruiseCluster=120.)
    if key == 'liveMapDataSP':
      return SimpleNamespace(speedLimit=0., speedLimitValid=False, speedLimitAhead=0., speedLimitAheadValid=False, speedLimitAheadDistance=0.)
    if key == 'gpsLocation':
      return SimpleNamespace(unixTimestampMillis=time.monotonic() * 1e3)
    if key == 'modelV2':
      x = np.linspace(0., 100., 33)
      line, edge = SimpleNamespace(x=x, y=np.full(33, 1.8)), SimpleNamespace(x=x, y=np.full(33, 1.8 + self.gap))
      return SimpleNamespace(laneLines=[line] * 4, roadEdges=[edge, edge], roadEdgeStds=[self.edge_std, self.edge_std])
    raise KeyError(key)

  def fresh(self):
    self.recv_time['roadEventsIC'] = time.monotonic()


class TestRampSignFilter(unittest.TestCase):
  def setUp(self):
    self.f, self.car = RampSignFilter(), Car()

  def drive(self, meters: float, v: float = 30.) -> float:
    out = 0.
    for _ in range(max(1, int(meters / (v * DT_MDL)))):
      self.car.fresh()
      out = self.f.update(self.car, v)
    return out

  def read(self, kph: float, meters: float = 30.) -> float:
    self.car.reading = kph * KPH
    return self.drive(meters)

  def test_a_ramp_sign_read_from_the_mainline_is_held_until_the_ramp(self):
    self.assertAlmostEqual(self.read(100), 100 * KPH)
    self.assertAlmostEqual(self.read(70), 100 * KPH)   # 70 %: carstate's 70 * KPH_TO_MS is a hair above 0.7 * 100 * KPH_TO_MS
    self.assertAlmostEqual(self.read(120), 120 * KPH)
    self.assertAlmostEqual(self.read(60, 200.), 120 * KPH)
    self.assertAlmostEqual(self.f.held, 60 * KPH)
    self.car.road.roadClass = 2                     # on the exit ramp
    self.assertAlmostEqual(self.drive(30.), 60 * KPH)
    self.assertEqual(self.f.held, 0.)

  def test_not_for_the_car_once_it_stays_on_the_mainline(self):
    self.read(120)
    self.read(60, KEEP + 50.)
    self.assertEqual(self.f.held, 0.)
    self.car.road.roadClass = 2                     # the next exit: not that ramp's sign
    self.assertAlmostEqual(self.drive(30.), 120 * KPH)
    self.car.road.roadClass = 1
    self.assertAlmostEqual(self.read(100), 100 * KPH)   # a new reading goes through as before

  def test_through_in_the_rightmost_lane_or_with_the_right_blinker(self):
    self.read(120)
    self.car.gap = 1.                               # the road edge right after the lane line, for a second or more
    self.drive(60.)
    self.assertAlmostEqual(self.read(60), 60 * KPH)
    self.f, self.car = RampSignFilter(), Car()
    self.read(120)
    self.car.right_blinker = True
    self.assertAlmostEqual(self.read(60), 60 * KPH)

  def test_a_held_reading_is_taken_with_the_right_blinker(self):
    self.read(120)
    self.read(60)
    self.car.right_blinker = True                   # moving over for the exit
    self.assertAlmostEqual(self.drive(10.), 60 * KPH)

  def test_a_vague_road_edge_is_not_the_rightmost_lane(self):
    self.read(120)
    self.car.gap, self.car.edge_std = 1., 3.
    self.drive(60.)
    self.assertAlmostEqual(self.read(60), 120 * KPH)

  def test_through_away_from_ramps_for_smaller_drops_and_off_the_mainline(self):
    self.read(120)
    self.car.road.rampNear = False                  # a 80 zone on the mainline
    self.assertAlmostEqual(self.read(80), 80 * KPH)
    self.car.road.rampNear = True
    self.assertAlmostEqual(self.read(100), 100 * KPH)   # 83 %
    self.car.road.roadClass = 5                         # a primary road
    self.assertAlmostEqual(self.read(40), 40 * KPH)

  def test_nothing_held_without_a_fresh_map_match(self):
    for lose in ("unmatched", "stale"):
      self.f, self.car = RampSignFilter(), Car()
      self.read(120)
      if lose == "unmatched":
        self.car.road.mapMatched = False
        self.assertAlmostEqual(self.read(60), 60 * KPH)
      else:
        self.car.reading = 60 * KPH
        self.car.recv_time['roadEventsIC'] = time.monotonic() - 2 * ramp_sign_filter.ROAD_MAX_AGE
        self.assertAlmostEqual(self.f.update(self.car, 30.), 60 * KPH)

  def test_the_first_reading_measured_against_the_map(self):
    self.car.reading = 60 * KPH
    self.assertAlmostEqual(self.f.update(self.car, 30.), 60 * KPH)   # nothing to compare with


class TestResolverRampSigns(OpenpilotTestCase):
  def test_assist_does_not_get_the_ramps_sign(self):
    resolver, car = SpeedLimitResolver(), Car()
    sm = cast(Any, car)   # plannerd's SubMaster
    resolver.policy = Policy.car_state_only
    for kph, meters, want in ((120, 100., 120), (60, 200., 120)):
      car.reading = kph * KPH
      for _ in range(int(meters / (30. * DT_MDL))):
        car.fresh()
        resolver.update(30., sm)
      self.assertAlmostEqual(resolver.speed_limit, want * KPH)
    self.assertAlmostEqual(resolver.ramp_filter.held, 60 * KPH)
    car.road.roadClass = 2
    car.fresh()
    resolver.update(30., sm)
    self.assertAlmostEqual(resolver.speed_limit, 60 * KPH)


if __name__ == "__main__":
  unittest.main()
