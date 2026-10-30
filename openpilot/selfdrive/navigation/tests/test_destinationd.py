import json
import threading
import unittest
import urllib.error
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from openpilot.selfdrive.navigation import destinationd
from openpilot.selfdrive.road_events import baidu


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
      self.assertTrue("导航目的地" in r.read().decode())
    body = json.dumps({"latitude": 39.8946, "longitude": 116.322}).encode()
    req = urllib.request.Request(url + "/api/destination", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(req) as r:
      self.assertEqual(json.loads(r.read())["destination"]["latitude"], 39.8946)
    with patch.object(destinationd, "allowed", return_value=False), self.assertRaises(urllib.error.HTTPError) as e:
      urllib.request.urlopen(url + "/api/state")
    self.assertEqual(e.exception.code, 403)


if __name__ == "__main__":
  unittest.main()
