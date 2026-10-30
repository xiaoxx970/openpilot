import unittest

import numpy as np

from openpilot.selfdrive.navigation.renderer import EGO, ROUTE_INTENSITY, ROUTE_WIDTH, SIZE, draw_segments, jam_mask, mercator, project
from openpilot.selfdrive.navigation.route import RouteTracker, METERS_PER_DEGREE


class TestJamMask(unittest.TestCase):
  def setUp(self):
    lat = 32.0
    east = np.linspace(0, 2000, 41) / (METERS_PER_DEGREE * np.cos(np.radians(lat)))   # 2 km due east
    self.route = np.column_stack((118.8 + east, np.full_like(east, lat)))
    self.tracker = RouteTracker(self.route)

  def frame(self, progress, jams):
    pose = self.tracker.pose(progress)
    mask = jam_mask(self.route, jams, *pose, progress)
    image = np.zeros((SIZE, SIZE), dtype=np.uint8)   # the route as mapsd strokes it
    pixels = project(mercator(self.route), *pose)
    draw_segments(image, pixels[:-1], pixels[1:], ROUTE_WIDTH / 2, ROUTE_INTENSITY, overlay=True)
    return mask, image == ROUTE_INTENSITY

  def test_jams_cover_the_route_where_they_are(self):
    mask, route = self.frame(500, [(550, 50, 3), (620, 40, 2), (700, 100, 1)])
    ahead = lambda m: (int(EGO[0]), int(EGO[1] - m))   # x, y of the route m ahead: up the frame  # noqa: E731
    x, y = ahead(75)
    self.assertTrue(route[y, x])
    self.assertEqual(mask[y, x], 3)
    x, y = ahead(140)
    self.assertEqual(mask[y, x], 2)
    for m in (20, 110, 175):   # before, between, after (status 1 is free flow: not drawn)
      x, y = ahead(m)
      self.assertEqual(mask[y, x], 0, m)
    self.assertTrue((mask[route] > 0).any() and (mask[route] == 0).any())

  def test_out_of_view_skipped(self):
    mask, _ = self.frame(100, [(1500, 100, 4), (0, 50, 3)])   # one far ahead, one just behind
    self.assertEqual(int(mask.max()), 3)
    mask, _ = self.frame(900, [(0, 100, 4)])
    self.assertEqual(int(mask.max()), 0)


if __name__ == "__main__":
  unittest.main()
