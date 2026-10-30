import unittest

import numpy as np

from openpilot.selfdrive.navigation import baidu_route
from openpilot.selfdrive.navigation.route import cumulative_distance
from openpilot.selfdrive.road_events import baidu
from openpilot.selfdrive.road_events.tests.test_baidu import gcj
from openpilot.selfdrive.road_events.tests.test_road_events import latlon


def step(points, road="Main Rd", duration=60., traffic=None):
  """One of Baidu's steps along the (x, y) m points, as the recorded answers have them."""
  length = float(sum(np.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(points, points[1:], strict=False)))
  return {"distance": round(length), "duration": duration, "road_name": road, "path": ";".join(gcj(x, y) for x, y in points),
          "traffic_condition": traffic or [{"status": 1, "geo_cnt": len(points) - 1, "distance": round(length, 2)}]}


def answer(steps):
  return {"status": 0, "message": "成功", "result": {"routes": [{"distance": sum(s["distance"] for s in steps), "steps": steps}]}}


def line(x0, y0, x1, y1, every=20.):
  n = max(1, int(np.hypot(x1 - x0, y1 - y0) // every))
  return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]


class TestBaiduRoute(unittest.TestCase):
  def test_maneuvers_from_the_geometry(self):
    # east 400 m on A, a new name straight on for 300 m (no maneuver), right (south) 200 m, then left (east) 100 m
    steps = [step(line(0, 0, 400, 0), "A", 40.), step(line(400, 0, 700, 0), "B", 30.), step(line(700, 0, 700, -200), "C", 20.),
             step(line(700, -200, 800, -200), baidu_route.UNNAMED, 10.)]
    nav_steps, points, jams = baidu_route.parse(answer(steps))
    self.assertEqual([round(s["distance"]) for s in nav_steps], [700, 200, 100])
    banners = [s["bannerInstructions"][0]["primary"] for s in nav_steps]
    self.assertEqual([(b["type"], b["modifier"], b["text"]) for b in banners],
                     [("turn", "right", "C"), ("turn", "left", ""), ("arrive", "straight", "目的地")])
    self.assertAlmostEqual(sum(s["duration"] for s in nav_steps), 100., delta=0.5)
    self.assertAlmostEqual(nav_steps[0]["duration"], 70., delta=0.5)
    # navd measures along each step's own geometry: it has to add up to the step's distance
    for s in nav_steps:
      coords = np.array(s["geometry"]["coordinates"])
      self.assertAlmostEqual(cumulative_distance(coords)[-1], s["distance"], delta=1.)
    self.assertAlmostEqual(baidu.as_path(points)[-1][0], 1000., delta=2.)
    self.assertEqual(jams, [])
    lat, lon = latlon(700, -200)
    self.assertAlmostEqual(nav_steps[1]["geometry"]["coordinates"][-1][1], lat, places=5)
    self.assertAlmostEqual(nav_steps[1]["geometry"]["coordinates"][-1][0], lon, places=5)

  def test_slight_turns_and_bends(self):
    # a 30 deg fork right onto a ramp is a maneuver, a 15 deg kink where the name changes is not
    a, b = line(0, 0, 300, 0), line(300, 0, 300 + 260 * np.cos(np.radians(15)), -260 * np.sin(np.radians(15)))
    c0 = b[-1]
    c = line(*c0, c0[0] + 200 * np.cos(np.radians(45)), c0[1] - 200 * np.sin(np.radians(45)))
    nav_steps, _, _ = baidu_route.parse(answer([step(a, "A"), step(b, "B"), step(c, "Ramp")]))
    self.assertEqual([s["bannerInstructions"][0]["primary"]["modifier"] for s in nav_steps], ["slight right", "straight"])
    self.assertAlmostEqual(nav_steps[0]["distance"], 560., delta=2.)

  def test_traffic_by_distance_from_the_start(self):
    traffic = [{"status": 1, "geo_cnt": 10, "distance": 200.}, {"status": 3, "geo_cnt": 5, "distance": 100.},
               {"status": 2, "geo_cnt": 5, "distance": 100.}]
    _, _, jams = baidu_route.parse(answer([step(line(0, 0, 400, 0), traffic=traffic)]))
    self.assertEqual(jams, [(200., 100., 3), (300., 100., 2)])

  def test_modifiers(self):
    self.assertEqual([baidu_route.modifier(t) for t in (30, -70, 130, -170)], ["slight right", "left", "sharp right", "uturn"])

  def test_no_route(self):
    self.assertIsNone(baidu_route.parse({"status": 2, "message": "参数错误"}))
    self.assertIsNone(baidu_route.parse({"status": 0, "result": {"routes": []}}))


if __name__ == "__main__":
  unittest.main()
