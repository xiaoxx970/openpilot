import json
import os
import tempfile
import threading
import unittest
import urllib.error
import urllib.parse
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


def gcj_location(lat, lon):
  dlat, dlon = baidu._gcj_delta(lat, lon)
  return {"lat": lat + dlat, "lng": lon + dlon}


SUGGESTIONS = {"status": 0, "message": "ok", "result": [
  {"name": "北京西站", "location": gcj_location(39.8919, 116.3157), "uid": "e1bf", "city": "北京市", "district": "丰台区",
   "address": "北京市-丰台区-莲花池东路118号"},
  {"name": "北京西站-公交线路", "uid": "x"},   # no position: left out
]}


class TestDestinations(unittest.TestCase):
  def setUp(self):
    self.params = FakeParams({"NavigationEnabled": True, "IsOffroad": True, "LastGPSPositionLLK": json.dumps({"latitude": 39.9, "longitude": 116.4})})
    self.queries = []

    def fetch(pq):
      self.queries.append(pq)
      return SUGGESTIONS
    self.dest = destinationd.Destinations(self.params, ("ak-test", "sk-test"), fetch, count_file=None)

  def test_search_converts_to_wgs84(self):
    places = self.dest.search(" 北京西站 ")["places"]
    self.assertEqual(len(places), 1)
    self.assertEqual((places[0]["name"], places[0]["address"]), ("北京西站", "北京市-丰台区-莲花池东路118号"))
    self.assertAlmostEqual(places[0]["latitude"], 39.8919, places=5)
    self.assertAlmostEqual(places[0]["longitude"], 116.3157, places=5)
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.queries[0]).query)
    self.assertEqual((query["query"][0], query["location"][0], query["coord_type"][0]), ("北京西站", "39.900000,116.400000", "1"))
    self.assertTrue("sn" in query)

  def test_pasted_coordinates_need_no_search(self):
    for text in ("39.8946,116.3220", "39.8946， 116.3220", "39.8946 116.3220"):
      self.assertEqual(self.dest.search(text)["places"][0]["latitude"], 39.8946)
    self.assertEqual(self.queries, [])
    no_keys = destinationd.Destinations(self.params, None, count_file=None)
    self.assertTrue("error" in no_keys.search("北京西站"))
    self.assertEqual(no_keys.search("39.8946,116.3220")["places"][0]["longitude"], 116.322)

  def test_daily_budget(self):
    self.enterContext(patch.object(self.dest.daily, "left", return_value=0))
    self.assertTrue("error" in self.dest.search("北京西站"))
    self.assertEqual(self.queries, [])

  def test_a_failed_search_keeps_the_url_out(self):
    def fail(pq):
      raise urllib.error.URLError("https://api.map.baidu.com/?ak=ak-test")
    dest = destinationd.Destinations(self.params, ("ak-test", "sk-test"), fail, count_file=None)
    self.assertFalse("ak-test" in json.dumps(dest.search("北京西站"), ensure_ascii=False))

  def test_set_and_clear(self):
    state = self.dest.set_destination({"latitude": 39.8946, "longitude": 116.322, "name": "北京西站"})
    self.assertEqual(self.params.values["NavDestination"], {"latitude": 39.8946, "longitude": 116.322, "place_name": "北京西站"})
    self.assertEqual((state["destination"]["place_name"], state["onroad"]), ("北京西站", False))
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


DETAIL = {"status": 0, "message": "ok", "result": {"name": "大行宫", "location": {"lat": 32.041503, "lng": 118.79482},
                                                  "address": "地铁2号线;地铁3号线", "uid": "bda99fec8a4b6f474e776ef3"}}
SHARE = "大行宫 大行宫 https://j.map.baidu.com/f9/Z3J"
TARGET = "https://map.baidu.com/mobile/webapp/search/search/qt=inf&uid=bda99fec8a4b6f474e776ef3/vt=map"


class TestShares(unittest.TestCase):
  def setUp(self):
    self.params = FakeParams({"NavigationEnabled": True, "IsOffroad": True})
    self.queries, self.resolved = [], []

    def fetch(pq):
      self.queries.append(pq)
      return DETAIL

    def resolve(url):
      self.resolved.append(url)
      return TARGET
    self.dest = destinationd.Destinations(self.params, ("ak-test", "sk-test"), fetch, count_file=None, resolve=resolve)

  def test_baidu_share_becomes_the_destination(self):
    state = self.dest.share(SHARE)
    self.assertEqual(self.resolved, ["https://j.map.baidu.com/f9/Z3J"])
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.queries[0]).query)
    self.assertEqual((urllib.parse.urlsplit(self.queries[0]).path, query["uid"][0]), (destinationd.DETAIL_PATH, "bda99fec8a4b6f474e776ef3"))
    d = self.params.values["NavDestination"]
    self.assertEqual(d["place_name"], "大行宫")
    self.assertAlmostEqual(d["latitude"], 32.043565, places=4)   # WGS-84, as the Mac checked against the share
    self.assertAlmostEqual(d["longitude"], 118.789626, places=4)
    self.assertEqual(state["destination"], d)
    self.assertEqual(self.dest.places.recent[0]["name"], "大行宫")

  def test_uid_in_a_long_link_needs_no_redirect(self):
    long_link = "https://map.baidu.com/?newmap=1&s=inf%26uid%3Dbda99fec8a4b6f474e776ef3%26c%3D315"
    self.assertEqual(destinationd.share_uid(long_link, resolve=self._no_resolve), "bda99fec8a4b6f474e776ef3")

  def test_other_hosts_are_not_fetched(self):
    self.assertIsNone(destinationd.share_uid("https://example.com/?uid=bda99fec8a4b6f474e776ef3", resolve=self._no_resolve))
    self.assertTrue("error" in self.dest.share("随便一段文字"))
    self.assertEqual((self.queries, self.resolved), ([], []))

  def test_search_box_takes_a_share(self):
    places = self.dest.search(SHARE)["places"]
    self.assertEqual(places[0]["name"], "大行宫")
    self.assertFalse("NavDestination" in self.params.values)   # the page shows it first

  def test_saved_places(self):
    self.assertTrue("error" in self.dest.go("家"))
    self.dest.share(SHARE)
    self.assertEqual(self.dest.save_place({"label": "公司"})["saved"]["公司"]["name"], "大行宫")   # the current destination
    self.dest.clear()
    state = self.dest.go("公司")
    self.assertEqual((state["destination"]["place_name"], state["saved_as"]), ("大行宫", "公司"))
    self.dest.set_destination({"latitude": 31.0, "longitude": 118.0, "name": "别处"})
    self.dest.go("公司")
    self.assertEqual([p["name"] for p in self.dest.places.recent], ["别处", "大行宫"])   # saved: not put first again
    self.assertTrue("error" in self.dest.save_place({"label": ""}))
    self.assertEqual(self.dest.delete_place({"label": "公司"})["saved"], {})

  def test_recent_newest_first_without_repeats(self):
    for i in range(10):
      self.dest.set_destination({"latitude": 32 + i * 0.01, "longitude": 118.8, "name": f"p{i}"})
    self.dest.set_destination({"latitude": 32.05, "longitude": 118.8, "name": "p5"})
    names = [p["name"] for p in self.dest.places.recent]
    self.assertEqual(names, ["p5", "p9", "p8", "p7", "p6", "p4", "p3", "p2"])

  def test_places_kept_in_a_file(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "navigation", "places.json")
      places = destinationd.Places(path)
      places.save("家", {"name": "家", "address": "", "latitude": 32.0, "longitude": 118.8})
      self.assertEqual(destinationd.Places(path).saved["家"]["latitude"], 32.0)

  def _no_resolve(self, url):
    raise AssertionError(url)


def route_answer():
  """Baidu's driving answer for 0..800 m east along y=0, congested 300..500 m."""
  steps = [step(line(0, 0, 800, 0), "A", 100., traffic=[{"status": 1, "geo_cnt": 15, "distance": 300.},
                                                       {"status": 3, "geo_cnt": 10, "distance": 200.},
                                                       {"status": 1, "geo_cnt": 15, "distance": 300.}])]
  return answer(steps)


class Clock:
  def __init__(self):
    self.t = 1000.

  def __call__(self):
    return self.t


class TestRoute(unittest.TestCase):
  def setUp(self):
    lat, lon = latlon(0, 0)
    self.params = FakeParams({"NavigationEnabled": True, "IsOffroad": True, "LastGPSPositionLLK": json.dumps({"latitude": lat, "longitude": lon})})
    self.queries, self.clock, self.live = [], Clock(), None

    def fetch(pq):
      self.queries.append(pq)
      return route_answer()
    self.dest = destinationd.Destinations(self.params, ("ak-test", "sk-test"), fetch, count_file=None, live=lambda: self.live,
                                          clock=self.clock)
    lat, lon = latlon(800, 0)
    self.dest.set_destination({"latitude": lat, "longitude": lon, "name": "终点"})

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

  def test_preview_once_then_cached(self):
    r = self.dest.route()
    self.assertEqual((r["route"]["source"], r["route"]["distance"], r["route"]["duration"]), ("preview", 800, 100))
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.queries[0]).query)
    self.assertEqual(urllib.parse.urlsplit(self.queries[0]).path, baidu.ROUTE_PATH)
    self.assertEqual(query["coord_type"][0], "wgs84")
    self.clock.t += 60
    self.assertEqual(self.dest.route()["route"]["age"], 60)
    self.assertEqual(len(self.queries), 1)
    self.clock.t += destinationd.PREVIEW_STALE
    self.dest.route()
    self.assertEqual(len(self.queries), 2)   # its traffic aged: planned again

  def test_new_destination_new_preview(self):
    self.dest.route()
    self.dest.set_destination({"latitude": 32.1, "longitude": 118.9})
    self.assertEqual(self.dest.route()["error"], "稍后再试")   # PREVIEW_MIN_GAP
    self.clock.t += destinationd.PREVIEW_MIN_GAP
    self.assertEqual(self.dest.route()["route"]["source"], "preview")
    self.assertEqual(len(self.queries), 2)

  def test_no_destination_or_position(self):
    self.dest.clear()
    self.assertIsNone(self.dest.route()["route"])
    self.dest.set_destination({"latitude": 32.1, "longitude": 118.9})
    self.params.values.pop("LastGPSPositionLLK")
    self.assertTrue("error" in self.dest.route())
    self.assertEqual(self.queries, [])

  def test_navd_route_while_driving(self):
    _, points, jams = baidu_route.parse(route_answer())
    self.live = {"points": points, "jams": jams, "distance": 650, "duration": 80, "car": [32.0, 118.8, 90.]}
    r = self.dest.route()
    self.assertEqual((r["route"]["source"], r["route"]["distance"], r["car"]), ("nav", 650, [32.0, 118.8, 90.]))
    self.assertEqual(self.queries, [])

  def test_preview_before_sending(self):
    lat, lon = latlon(800, 0)
    self.dest.clear()
    r = self.dest.preview_route({"latitude": lat, "longitude": lon})
    self.assertEqual(r["route"]["distance"], 800)
    self.dest.set_destination({"latitude": lat, "longitude": lon})
    self.dest.route()
    self.assertEqual(len(self.queries), 1)   # the same place: the preview is reused
    self.assertTrue("error" in self.dest.preview_route({"latitude": "x"}))

  def test_nav_only(self):
    self.assertIsNone(self.dest.route(nav_only=True)["route"])
    self.assertEqual(self.queries, [])
    self.assertEqual(self.dest.state()["position"], self.dest.near())

  def test_preview_budget(self):
    self.enterContext(patch.object(self.dest.daily, "left", return_value=0))
    self.assertTrue("error" in self.dest.route())
    self.assertEqual(self.queries, [])


class TestServer(unittest.TestCase):
  def test_page_and_api(self):
    params = FakeParams({"NavigationEnabled": True})
    dest = destinationd.Destinations(params, None, count_file=None)
    server = ThreadingHTTPServer(("127.0.0.1", 0), destinationd.handler(dest))
    threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True).start()
    self.addCleanup(server.server_close)
    self.addCleanup(server.shutdown)
    url = f"http://127.0.0.1:{server.server_port}"
    with urllib.request.urlopen(url + "/") as r:
      self.assertTrue("搜地点" in r.read().decode())
    body = json.dumps({"latitude": 39.8946, "longitude": 116.322}).encode()
    req = urllib.request.Request(url + "/api/destination", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as r:
      self.assertEqual(json.loads(r.read())["destination"]["latitude"], 39.8946)
    with patch.object(destinationd, "allowed", return_value=False), self.assertRaises(urllib.error.HTTPError) as e:
      urllib.request.urlopen(url + "/api/state")
    self.assertEqual(e.exception.code, 403)


if __name__ == "__main__":
  unittest.main()
