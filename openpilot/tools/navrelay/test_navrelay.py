import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from openpilot.selfdrive.navigation import destinationd
from openpilot.selfdrive.navigation.tests.test_destinationd import DETAIL, SHARE, TARGET, route_answer
from openpilot.selfdrive.road_events.tests.test_road_events import latlon
from openpilot.tools.navrelay import navrelay

KEY = "k" * 32


class FakeCar:
  """The car's destinationd: its answers, or OSError while offline."""
  def __init__(self):
    self.online = True
    self.calls = []
    self.destination = None
    self.position = list(latlon(0, 0))
    self.route = {"route": None, "car": None}
    self.places = {"saved": {"家": {"name": "家", "address": "", "latitude": 32.0, "longitude": 118.8}}, "recent": []}

  def state(self):
    return {"navigation": True, "destination": self.destination, "onroad": True, "search": True, "position": self.position}

  def call(self, path, body=None, timeout=None):
    if not self.online:
      raise OSError("offline")
    self.calls.append((path, body))
    if path == "/api/state":
      return self.state()
    if path == "/api/places":
      return self.places
    if path.startswith("/api/route"):
      return self.route
    if path == "/api/destination":
      if body["latitude"] > 90:
        return {"error": "坐标不对"}
      self.destination = {"latitude": body["latitude"], "longitude": body["longitude"], "place_name": body["name"]}
    elif path == "/api/clear":
      self.destination = None
    return self.state()


class FakePush:
  def __init__(self):
    self.sent = []

  def notify(self, title, body, url="/"):
    self.sent.append((title, body))

  def public_key(self):
    return "PUB"

  def subscribe(self, sub):
    return {"subscribed": True}


class Clock:
  def __init__(self):
    self.t = 1000.

  def __call__(self):
    return self.t


def fetch_for(queries):
  def fetch(pq):
    queries.append(pq)
    return route_answer() if "direction" in pq else DETAIL
  return fetch


class TestApp(unittest.TestCase):
  def setUp(self):
    self.car, self.clock, self.queries = FakeCar(), Clock(), []
    self.dir = self.enterContext(tempfile.TemporaryDirectory())
    self.app = self.make()

  def make(self, keys=("ak", "sk")):
    relay = navrelay.Relay(self.car, self.dir, clock=self.clock)
    dest = destinationd.Destinations(navrelay.CarParams(relay), keys, fetch_for(self.queries), count_file=None,
                                     places=destinationd.Places(None), resolve=lambda url: TARGET, clock=self.clock)
    relay.poll()
    self.push = FakePush()
    return navrelay.App(relay, dest, self.push)

  def test_shortcut_pushed(self):
    self.app.post("/api/share", {"text": SHARE}, notify=True)
    self.app.post("/api/go", {"label": "nope"}, notify=True)
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "页面"})   # the page: no push
    self.assertEqual(self.push.sent, [("已发到车上", "大行宫"), ("没发到车上", "没有叫“nope”的地点")])

  def test_queued_then_taken_pushed(self):
    self.car.online = False
    self.app.relay.poll()
    self.app.post("/api/go", {"label": "nope"})
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "甲"}, notify=True)
    self.car.online = True
    self.app.relay.poll()
    self.assertEqual(self.push.sent, [("等车上线", "车上线后自动导航去 甲"), ("车上线了", "开始导航去 甲")])
    self.app.post("/api/destination", {"latitude": 32.2, "longitude": 118.9, "name": "乙"})   # sent at once: not "late"
    self.assertEqual(len(self.push.sent), 2)

  def test_push_api(self):
    self.assertEqual(self.app.get("/api/push/key"), {"key": "PUB"})
    self.assertEqual(self.app.post("/api/push/subscribe", {}), {"subscribed": True})
    self.app.post("/api/push/test", {})
    self.assertEqual(self.push.sent[-1][0], "推送开好了")

  def test_share_resolved_here_then_sent(self):
    s = self.app.post("/api/share", {"text": SHARE})
    self.assertEqual(s["destination"]["place_name"], "大行宫")
    self.assertEqual(self.car.calls[-1][0], "/api/destination")
    self.assertEqual(self.car.destination["place_name"], "大行宫")
    self.assertEqual(self.app.dest.places.recent[0]["name"], "大行宫")   # kept on the server
    self.assertFalse(s["queued"])

  def test_offline_queued_newest_wins(self):
    self.car.online = False
    self.app.relay.poll()
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "甲"})
    s = self.app.post("/api/destination", {"latitude": 32.2, "longitude": 118.9, "name": "乙"})
    self.assertEqual((s["destination"]["place_name"], s["queued"], s["car_online"]), ("乙", True, False))
    self.assertEqual(self.car.calls, [("/api/state", None)])   # nothing tried while it is off
    # across a restart of the server
    self.app = self.make()
    self.car.online = True
    self.app.relay.poll()
    self.assertEqual(self.car.destination["place_name"], "乙")
    self.assertEqual([c[1]["name"] for c in self.car.calls if c[0] == "/api/destination"], ["乙"])
    self.assertFalse(self.app.state()["queued"])

  def test_clear_queued(self):
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "甲"})
    self.car.online = False
    self.app.relay.poll()
    self.assertIsNone(self.app.post("/api/clear", {})["destination"])
    self.car.online = True
    self.app.relay.poll()
    self.assertIsNone(self.car.destination)

  def test_stale_queue_dropped(self):
    self.car.online = False
    self.app.relay.poll()
    self.app.post("/api/go", {"label": "nope"})
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "甲"})
    self.clock.t += navrelay.QUEUE_TTL + 1
    self.car.online = True
    self.app.relay.poll()
    self.assertIsNone(self.car.destination)

  def test_preview_here_from_the_car_position(self):
    lat, lon = latlon(800, 0)
    self.app.post("/api/destination", {"latitude": lat, "longitude": lon, "name": "终点"})
    r = self.app.get("/api/route")
    self.assertEqual((r["route"]["source"], r["route"]["distance"]), ("preview", 800))
    self.assertTrue(any("direction" in q for q in self.queries))
    self.assertTrue(("/api/route?nav=1", None) in self.car.calls)   # the car only for its live route

  def test_live_route_from_the_car(self):
    self.car.route = {"route": {"source": "nav", "segments": [], "distance": 500, "duration": 60}, "car": [32.0, 118.8, 0.]}
    self.assertEqual(self.app.get("/api/route")["route"]["source"], "nav")
    self.car.online = False
    self.app.relay.poll()
    r = self.app.get("/api/route")   # no destination left here: the last navd route, marked offline
    self.assertEqual((r["route"]["source"], r["offline"]), ("nav", True))

  def test_places_imported_once(self):
    self.app.import_places()
    self.assertEqual(list(self.app.dest.places.saved), ["家"])
    self.app.dest.places.delete("家")
    self.app.dest.places.save("公司", self.car.places["saved"]["家"])
    self.app.import_places()
    self.assertEqual(list(self.app.dest.places.saved), ["公司"])

  def test_turned_down_not_retried(self):
    self.app.post("/api/destination", {"latitude": 32.1, "longitude": 118.9})
    self.app.relay.send("/api/destination", {"latitude": 100., "longitude": 0., "name": "", "address": ""})
    self.assertIsNone(self.app.relay.pending)

  def test_without_keys_the_car_searches(self):
    self.app = self.make(keys=None)
    self.app.post("/api/search", {"query": "x"})
    self.assertEqual(self.car.calls[-1], ("/api/search", {"query": "x"}))
    self.car.online = False
    self.app.relay.poll()
    self.assertTrue("error" in self.app.post("/api/search", {"query": "x"}))


class TestServer(unittest.TestCase):
  def setUp(self):
    self.car = FakeCar()
    relay = navrelay.Relay(self.car, None)
    dest = destinationd.Destinations(navrelay.CarParams(relay), None, count_file=None, places=destinationd.Places(None))
    server = ThreadingHTTPServer(("127.0.0.1", 0), navrelay.handler(navrelay.App(relay, dest), KEY,
                                                                    b"<html>const JSAK='__BAIDU_JS_AK__';</html>", "JSKEY123"))
    threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True).start()
    self.addCleanup(server.server_close)
    self.addCleanup(server.shutdown)
    self.url = f"http://127.0.0.1:{server.server_port}"

  def open(self, path, body=None, headers=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(self.url + path, data, {"Content-Type": "application/json", **(headers or {})})
    return urllib.request.urlopen(req, timeout=5)

  def open_form(self, path, body):
    class NoRedirect(urllib.request.HTTPRedirectHandler):
      def redirect_request(self, *args, **kwargs):
        return None
    req = urllib.request.Request(self.url + path, body.encode(), {"Content-Type": "application/x-www-form-urlencoded"})
    try:
      return urllib.request.build_opener(NoRedirect).open(req, timeout=5)
    except urllib.error.HTTPError as e:   # the 303, not followed
      return e

  def test_key(self):
    with self.assertRaises(urllib.error.HTTPError) as e:
      self.open("/api/state")
    self.assertEqual(e.exception.code, 403)
    with self.assertRaises(urllib.error.HTTPError):
      self.open("/api/state", headers={"X-Key": "wrong"})
    with self.open("/") as r:   # no key: the form, not the page
      self.assertTrue('name="key"' in r.read().decode())
    with self.open("/?k=" + KEY) as r:   # a key in the URL is not taken
      self.assertTrue('name="key"' in r.read().decode())
    with self.open_form("/login", "key=wrong") as r:
      self.assertTrue("密钥不对" in r.read().decode())
    with self.open_form("/login", "key=" + KEY) as r:
      self.assertEqual((r.status, r.headers["Location"], r.headers["Referrer-Policy"]), (303, "/", "no-referrer"))
      cookie = r.headers["Set-Cookie"].split(";")[0]
    with self.open("/", headers={"Cookie": cookie}) as r:
      self.assertEqual(r.read(), b"<html>const JSAK='JSKEY123';</html>")
    with self.open("/api/state", headers={"Cookie": cookie}) as r:
      self.assertTrue(json.loads(r.read())["navigation"])

  def test_shortcut_post(self):
    with self.open("/api/destination", {"latitude": 32.1, "longitude": 118.9, "name": "家"}, {"X-Key": KEY}) as r:
      self.assertEqual(json.loads(r.read())["destination"]["place_name"], "家")
    with self.assertRaises(urllib.error.HTTPError) as e:
      self.open("/api/other", {}, {"X-Key": KEY})
    self.assertEqual(e.exception.code, 404)

  def test_manifest_and_worker_without_key(self):
    with self.open("/manifest.json") as r:
      self.assertEqual(json.loads(r.read())["display"], "standalone")
    with self.open("/sw.js") as r:
      self.assertTrue(b"notificationclick" in r.read())

  def test_key_required(self):
    self.assertFalse(navrelay.key_ok("", ""))
    self.assertFalse(navrelay.key_ok(KEY, None))


if __name__ == "__main__":
  unittest.main()
