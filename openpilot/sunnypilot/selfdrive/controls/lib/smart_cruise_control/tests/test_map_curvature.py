import json
import math

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_curvature import EARTH_RADIUS, MapCurvatureAhead
from openpilot.common.test import OpenpilotTestCase

LAT0, LON0 = 44.0, 85.0


def to_latlon(x, y):
  """Local east / north metres around (LAT0, LON0) to latitude / longitude."""
  lat = LAT0 + np.degrees(np.asarray(y, dtype=float) / EARTH_RADIUS)
  lon = LON0 + np.degrees(np.asarray(x, dtype=float) / (EARTH_RADIUS * math.cos(math.radians(LAT0))))
  return lat, lon


def road_json(x, y, kappa):
  """A MapCurvatures table as mapd writes it."""
  lat, lon = to_latlon(x, y)
  return json.dumps([{'latitude': float(a), 'longitude': float(b), 'curvature': float(k)} for a, b, k in zip(lat, lon, kappa, strict=True)])


def north_road(length=600., spacing=20., curve=(300., 340.), kappa=1 / 40.):
  """Nodes every `spacing` m of a road running north from the origin, curved (by its curvature values) over `curve`."""
  y = np.arange(0., length + 1e-6, spacing)
  return np.zeros_like(y), y, np.where((y >= curve[0]) & (y <= curve[1]), kappa, 0.)


class Car:
  """Drives the map lookup like plannerd: a GPS fix per call and the distance driven so far."""

  def __init__(self, m, x, y, bearing, speed=15.):
    self.m, self.x, self.y, self.bearing, self.speed, self.odometer = m, x, y, bearing, speed, 0.

  def at(self, x=None, y=None, bearing=None, speed=None, advance=0.):
    if x is not None or y is not None:
      nx, ny = self.x if x is None else x, self.y if y is None else y
      self.odometer += math.hypot(nx - self.x, ny - self.y)
      self.x, self.y = nx, ny
    self.bearing = self.bearing if bearing is None else bearing
    self.speed = self.speed if speed is None else speed
    lat, lon = to_latlon(self.x, self.y)
    return self.m.ahead(float(lat), float(lon), self.bearing, self.speed, self.odometer, advance=advance)


class TestMapCurvatureAhead(OpenpilotTestCase):

  def setup_method(self):
    self.m = MapCurvatureAhead()
    self.road = road_json(*north_road())
    self.m.set_points(self.road)

  def confirmed_car(self, x=3., y=100.):
    car = Car(self.m, x, y, 0.)
    assert car.at() is None, 'a new table is only used once the car moves forward along it'
    assert car.at(y=y + 5.) is not None
    return car

  def test_distances_along_the_road(self):
    car = self.confirmed_car()
    dist, kappa = car.at(y=110.)
    assert abs(dist[0] - 10.) < 0.5 and dist[-1] <= self.m.horizon
    np.testing.assert_allclose(dist[kappa > 0][0], 300. - 110., atol=0.5)
    dist2, _ = car.at(advance=8.)
    np.testing.assert_allclose(dist2[0], dist[0] - 8., atol=0.5)

  def test_off_the_road(self):
    self.confirmed_car()
    assert Car(self.m, 40., 120., 0.).at() is None

  def test_wrong_heading_is_not_placed(self):
    # a car crossing the road at right angles is not on it
    assert Car(self.m, 3., 100., 90.).at() is None
    assert self.m.table.last_seg is None

  def test_reversed_table_is_not_used(self):
    car = self.confirmed_car()
    # mapd in a hairpin: the same road reversed, while the GPS bearing has swung round with the car
    x, y, k = north_road()
    self.m.set_points(road_json(x[::-1], y[::-1], k[::-1]))
    assert car.at(y=110., bearing=180.) is None
    assert car.at(y=116.) is None
    assert car.at(y=122.) is None, 'the car moves against this table, so it must stay unused'
    # mapd switches back: the original road is used again straight away
    self.m.set_points(self.road)
    assert car.at(y=126., bearing=0.) is not None

  def test_hairpin_other_leg(self):
    # up the road at x = 0, a hairpin, and back down at x = 20: the car going down is placed on the second leg
    up = np.arange(0., 201., 20.)
    x = np.concatenate((np.zeros_like(up), [10.], np.full_like(up, 20.)))
    y = np.concatenate((up, [212.], up[::-1]))
    self.m.set_points(road_json(x, y, np.zeros_like(x)))
    car = Car(self.m, 18., 150., 180.)
    car.at()
    dist, _ = car.at(y=144.)
    s_leg2 = self.m.table.s[len(up) + 1]          # where the second leg starts along the road
    assert self.m.s_car > s_leg2
    assert abs(dist[0] - (144. - 140.)) < 0.5

  def test_slow_driving_keeps_its_place(self):
    car = self.confirmed_car()
    # below 5 m/s the GPS bearing is noise: the place is kept near the last match whatever it says
    assert car.at(y=106., bearing=200., speed=2.) is not None

  def test_bad_table(self):
    self.m.set_points('not json')
    assert Car(self.m, 3., 100., 0.).at() is None
    self.m.set_points(None)
    assert Car(self.m, 3., 100., 0.).at() is None
