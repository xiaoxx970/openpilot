import json
import os
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from openpilot.selfdrive.navigation import baidu_route, destinationd
from openpilot.selfdrive.navigation.tests.test_baidu_route import answer, line, step
from openpilot.selfdrive.road_events import baidu
from openpilot.selfdrive.road_events.tests.test_road_events import latlon


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value, block=False):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class TestDestinations(unittest.TestCase):
  def setUp(self):
    self.params = FakeParams({"NavigationEnabled": True, "IsOffroad": True, "LastGPSPositionLLK": json.dumps({"latitude": 39.9, "longitude": 116.4})})
    self.dest = destinationd.Destinations(self.params)

  def test_set_and_clear(self):
    state = self.dest.set_destination({"latitude": 39.8946, "longitude": 116.322, "name": "北京西站"})
    self.assertEqual(self.params.values["NavDestination"], {"latitude": 39.8946, "longitude": 116.322, "place_name": "北京西站"})
    self.assertEqual((state["destination"]["place_name"], state["onroad"], state["position"]), ("北京西站", False, (39.9, 116.4)))
    self.assertTrue("error" in self.dest.set_destination({"latitude": 91, "longitude": 0}))
    self.assertTrue("error" in self.dest.set_destination({"latitude": "x"}))
    self.assertIsNone(self.dest.clear()["destination"])
    self.assertNotIn("NavDestination", self.params.values)

  def test_no_interface(self):
    self.assertIsNone(destinationd.interface_ip("no-such-if0"))

  def test_only_tailscale_and_local(self):
    for address, ok in (("100.101.102.103", True), ("127.0.0.1", True), ("::ffff:100.64.0.1", True), ("fd7a:115c:a1e0::1", True),
                        ("192.168.43.10", False), ("10.0.0.2", False), ("::ffff:192.168.1.2", False), ("garbage", False)):
      self.assertEqual(destinationd.allowed(address), ok, address)

  def test_recent_newest_first_without_repeats(self):
    for i in range(10):
      self.dest.set_destination({"latitude": 32 + i * 0.01, "longitude": 118.8, "name": f"p{i}"})
    self.dest.set_destination({"latitude": 32.05, "longitude": 118.8, "name": "p5"})
    names = [p["name"] for p in self.dest.places.recent]
    self.assertEqual(names, ["p5", "p9", "p8", "p7", "p6", "p4", "p3", "p2"])

  def test_saved_places_kept(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "navigation", "places.json")
      os.makedirs(os.path.dirname(path))
      with open(path, "w") as f:
        json.dump({"saved": {"家": {"name": "家", "address": "", "latitude": 32.0, "longitude": 118.8}}, "recent": []}, f)
      dest = destinationd.Destinations(self.params, places=destinationd.Places(path))
      state = dest.set_destination({"latitude": 32.0, "longitude": 118.8, "name": "家"})
      self.assertEqual((state["saved_as"], dest.places.recent), ("家", []))   # saved: not in recent
      dest.set_destination({"latitude": 31.0, "longitude": 118.0, "name": "别处"})
      self.assertEqual([p["name"] for p in destinationd.Places(path).recent], ["别处"])


def route_answer():
  """Baidu's driving answer for 0..800 m east along y=0, congested 300..500 m."""
  steps = [step(line(0, 0, 800, 0), "A", 100., traffic=[{"status": 1, "geo_cnt": 15, "distance": 300.},
                                                       {"status": 3, "geo_cnt": 10, "distance": 200.},
                                                       {"status": 1, "geo_cnt": 15, "distance": 300.}])]
  return answer(steps)


class TestRoute(unittest.TestCase):
  def setUp(self):
    lat, lon = latlon(0, 0)
    self.params = FakeParams({"NavigationEnabled": True, "IsOffroad": True, "LastGPSPositionLLK": json.dumps({"latitude": lat, "longitude": lon})})
    self.live = None
    self.dest = destinationd.Destinations(self.params, live=lambda: self.live)

  def test_segments_by_traffic(self):
    segs = destinationd.route_segments(*baidu_route.parse(route_answer())[1:])
    self.assertEqual([s["status"] for s in segs], [1, 3, 1])
    path = baidu.as_path([tuple(p) for p in segs[1]["points"]])
    self.assertAlmostEqual(path[-1][0], 200., delta=2.)
    self.assertEqual(segs[0]["points"][-1], segs[1]["points"][0])   # runs join up
    self.assertEqual(destinationd.route_segments([(32., 118.)], []), [])

  def test_thinned(self):
    pts = [latlon(x, 0) for x in range(0, 20000, 5)]
    segs = destinationd.route_segments(pts, [])
    self.assertLess(len(segs[0]["points"]), destinationd.MAX_ROUTE_POINTS * 1.1)
    self.assertEqual(segs[0]["points"][-1], [round(pts[-1][0], 6), round(pts[-1][1], 6)])

  def test_no_route_without_navd(self):
    lat, lon = latlon(800, 0)
    self.dest.set_destination({"latitude": lat, "longitude": lon, "name": "终点"})
    r = self.dest.route()
    self.assertIsNone(r["route"])   # previews are comma-link's
    self.assertEqual(r["car"][:2], list(self.dest.near()))
    self.params.values.pop("LastGPSPositionLLK")
    self.assertEqual(self.dest.route(), {"route": None, "car": None})

  def test_navd_route_while_driving(self):
    _, points, jams = baidu_route.parse(route_answer())
    self.live = {"points": points, "jams": jams, "distance": 650, "duration": 80, "car": [32.0, 118.8, 90.]}
    r = self.dest.route()
    self.assertEqual((r["route"]["source"], r["route"]["distance"], r["car"]), ("nav", 650, [32.0, 118.8, 90.]))
    self.assertEqual([s["status"] for s in r["route"]["segments"]], [1, 3, 1])


class TestServer(unittest.TestCase):
  def test_api(self):
    params = FakeParams({"NavigationEnabled": True})
    dest = destinationd.Destinations(params)
    server = ThreadingHTTPServer(("127.0.0.1", 0), destinationd.handler(dest))
    threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True).start()
    self.addCleanup(server.server_close)
    self.addCleanup(server.shutdown)
    url = f"http://127.0.0.1:{server.server_port}"
    for gone in ("/", "/api/search"):   # the page and search are comma-link's
      with self.assertRaises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(url + gone, b"{}" if gone != "/" else None)
      self.assertEqual(e.exception.code, 404)
    body = json.dumps({"latitude": 39.8946, "longitude": 116.322}).encode()
    req = urllib.request.Request(url + "/api/destination", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as r:
      self.assertEqual(json.loads(r.read())["destination"]["latitude"], 39.8946)
    with urllib.request.urlopen(url + "/api/route?nav=1") as r:
      self.assertIsNone(json.loads(r.read())["route"])
    with urllib.request.urlopen(url + "/api/places") as r:
      self.assertEqual(len(json.loads(r.read())["recent"]), 1)
    with patch.object(destinationd, "allowed", return_value=False), self.assertRaises(urllib.error.HTTPError) as e:
      urllib.request.urlopen(url + "/api/state")
    self.assertEqual(e.exception.code, 403)


if __name__ == "__main__":
  unittest.main()
