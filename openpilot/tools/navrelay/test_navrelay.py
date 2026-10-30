import json
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

from openpilot.tools.navrelay import navrelay

KEY = "k" * 32


class FakeCar:
  """destinationd's answers, or OSError while offline."""
  def __init__(self):
    self.online = True
    self.calls = []
    self.destination = None

  def call(self, path, body=None):
    if not self.online:
      raise OSError("offline")
    self.calls.append((path, body))
    if path == "/":
      return 200, "<html>导航目的地</html>".encode(), "text/html; charset=utf-8"
    if path == "/api/places":
      return 200, json.dumps({"saved": {"家": {"name": "家"}}, "recent": []}).encode(), "application/json"
    if path == "/api/go" and body.get("label") == "没有":
      return 200, json.dumps({"error": "还没有存这个地点"}).encode(), "application/json"
    if path == "/api/destination":
      self.destination = {"place_name": body.get("name")}
    elif path == "/api/go":
      self.destination = {"place_name": body["label"]}
    elif path == "/api/clear":
      self.destination = None
    return 200, json.dumps({"navigation": True, "destination": self.destination, "onroad": True}).encode(), "application/json"


class Clock:
  def __init__(self):
    self.t = 1000.

  def __call__(self):
    return self.t


class TestRelay(unittest.TestCase):
  def setUp(self):
    self.car, self.clock = FakeCar(), Clock()
    self.dir = self.enterContext(tempfile.TemporaryDirectory())
    self.relay = navrelay.Relay(self.car, self.dir, clock=self.clock)

  def test_online_passes_through(self):
    status, body, _ = self.relay.handle("POST", "/api/go", {"label": "家"})
    self.assertEqual((status, json.loads(body)["destination"]["place_name"]), (200, "家"))
    self.assertIsNone(self.relay.pending)

  def test_offline_queues_newest_and_delivers(self):
    self.relay.handle("GET", "/", None)
    self.relay.handle("GET", "/api/places", None)
    self.car.online = False
    self.relay.handle("POST", "/api/go", {"label": "家"})
    _, body, _ = self.relay.handle("POST", "/api/share", {"text": "大行宫 大行宫 https://j.map.baidu.com/f9/Z3J"})
    state = json.loads(body)
    self.assertEqual((state["queued"], state["destination"]["place_name"]), ("/api/share", "等车上线：大行宫 大行宫"))
    self.assertTrue("导航目的地" in self.relay.handle("GET", "/", None)[1].decode())
    self.assertEqual(json.loads(self.relay.handle("GET", "/api/places", None)[1])["saved"]["家"]["name"], "家")
    self.assertTrue("error" in json.loads(self.relay.handle("POST", "/api/search", {"query": "x"})[1]))
    self.assertFalse(self.relay.deliver())
    # kept across a restart of the relay
    self.relay = navrelay.Relay(self.car, self.dir, clock=self.clock)
    self.car.online = True
    self.assertTrue(self.relay.deliver())
    self.assertEqual(self.car.calls[-1], ("/api/share", {"text": "大行宫 大行宫 https://j.map.baidu.com/f9/Z3J"}))
    self.assertIsNone(self.relay.pending)
    self.assertEqual(len([c for c in self.car.calls if c[0] in navrelay.QUEUED]), 1)   # the older one never sent

  def test_a_turned_down_one_is_not_retried(self):
    self.car.online = False
    self.relay.handle("POST", "/api/go", {"label": "没有"})
    self.car.online = True
    self.assertTrue(self.relay.deliver())
    self.assertIsNone(self.relay.pending)

  def test_stale_queue_is_dropped(self):
    self.car.online = False
    self.relay.handle("POST", "/api/go", {"label": "家"})
    self.clock.t += navrelay.QUEUE_TTL + 1
    self.assertIsNone(self.relay.offline_state()["destination"])
    self.car.online = True
    self.assertTrue(self.relay.deliver())
    self.assertEqual(self.car.calls, [])

  def test_no_page_yet(self):
    self.car.online = False
    self.assertEqual(self.relay.handle("GET", "/", None)[0], 503)


class TestServer(unittest.TestCase):
  def setUp(self):
    self.car = FakeCar()
    relay = navrelay.Relay(self.car, None)
    server = ThreadingHTTPServer(("127.0.0.1", 0), navrelay.handler(relay, KEY))
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
      self.assertEqual(r.read().decode(), "<html>导航目的地</html>")   # the car's page
    with self.open("/api/state", headers={"Cookie": cookie}) as r:
      self.assertTrue(json.loads(r.read())["navigation"])

  def test_shortcut_post(self):
    with self.open("/api/go", {"label": "家"}, {"X-Key": KEY}) as r:
      self.assertEqual(json.loads(r.read())["destination"]["place_name"], "家")
    with self.assertRaises(urllib.error.HTTPError) as e:
      self.open("/api/other", {}, {"X-Key": KEY})
    self.assertEqual(e.exception.code, 404)

  def test_key_required(self):
    self.assertFalse(navrelay.key_ok("", ""))
    self.assertFalse(navrelay.key_ok(KEY, None))


if __name__ == "__main__":
  unittest.main()
