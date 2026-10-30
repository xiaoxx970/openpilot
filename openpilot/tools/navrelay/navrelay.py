#!/usr/bin/env python3
"""Navigation destinations from the phone, the way comma connect did it: the server is the app, the car takes destinations.

Runs on the user's server, on the tailnet with the car, behind the server's reverse proxy (1Panel's openresty), which does
https for the domain; it listens on 127.0.0.1, plain http.

  phone --https + key--> openresty --> this server --tailnet--> car destinationd (port 8082)

- The page (NAVRELAY_PAGE, the car's selfdrive/navigation/destination_page.html) is served from here, with Baidu's browser
  AK for its map (NAVRELAY_JS_AK) put in: it opens with the car offline, and changes to it need no install on the car.
- Search, Baidu share links, route previews and the saved and recent places are done here, by destinationd's own code,
  with a Baidu server AK of the server's own (NAVRELAY_BAIDU_AK / _SK). Without one, search and shares go to the car.
- The destination goes to the car's /api/destination at once, or waits here while the car is offline (newest wins,
  QUEUE_TTL), sent when it is back: comma's "next destination". While the car navigates, its route comes from navd.
- Key: typed into the page once (kept as a cookie), or the X-Key header (iOS Shortcuts). Never in a URL: access logs,
  history and Referer headers would keep it.
- Push (webpush.py, with python3-cryptography): a destination from a Shortcut, and one the car takes when it is back
  online, notify the page added to the phone's home screen; a tap opens it (a Shortcut's own notification opens Shortcuts).
- Python stdlib plus destinationd, baidu_route and road_events/baidu (pure python), copied next to it by install.sh.
"""
from __future__ import annotations

import hmac
import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.selfdrive.navigation import destinationd

try:
  from openpilot.tools.navrelay import webpush
except ImportError:   # no cryptography: no push
  webpush = None  # type: ignore[assignment]

CAR_TIMEOUT = 8.       # s for one call to the car
POLL_TIMEOUT = 3.      # s for the background look at the car
POLL = 10.             # s between looks at the car: its state, the queued destination
QUEUE_TTL = 3 * 3600.  # s a queued destination waits for the car
INBOX_TTL = 600.       # s a share that needs choosing waits for the page
MAX_BODY = 4096
COOKIE = "navkey"
POSTS = ("/api/search", "/api/destination", "/api/clear", "/api/share", "/api/go", "/api/places", "/api/places/delete",
         "/api/preview", "/api/push/subscribe", "/api/push/test")
SENT = ("/api/destination", "/api/share", "/api/go")   # what sets a destination
MANIFEST = json.dumps({"name": "导航", "short_name": "导航", "start_url": "/", "scope": "/", "display": "standalone",
                       "background_color": "#0b1116", "theme_color": "#0b1116"}, ensure_ascii=False).encode()
SERVICE_WORKER = """// navrelay's service worker: push only, no caching
self.addEventListener('push',e=>{let d={};try{d=e.data.json()}catch(x){}
  e.waitUntil(self.registration.showNotification(d.title||'导航',{body:d.body||'',data:{url:d.url||'/'},tag:'navrelay'}))});
self.addEventListener('notificationclick',e=>{e.notification.close();const url=(e.notification.data&&e.notification.data.url)||'/';
  e.waitUntil(clients.matchAll({type:'window',includeUncontrolled:true}).then(cs=>{for(const c of cs){if('focus' in c)return c.focus()}
    return clients.openWindow(url)}))});
""".encode()


class Car:
  """The car's destinationd over the tailnet."""
  def __init__(self, url: str):
    self.url = url.rstrip("/")

  def call(self, path: str, body: dict | None = None, timeout: float = CAR_TIMEOUT) -> dict:
    """The car's JSON answer; OSError when it is not reachable or does not answer with one."""
    data = None if body is None else json.dumps(body, ensure_ascii=False).encode()
    req = urllib.request.Request(self.url + path, data, {"Content-Type": "application/json"} if data else {})
    try:
      with urllib.request.urlopen(req, timeout=timeout) as r:
        answer = json.loads(r.read())
    except urllib.error.HTTPError as e:   # an older destinationd without the path, or one turning us away
      raise OSError(f"car answered {e.code}") from e
    except ValueError as e:
      raise OSError("car answered no JSON") from e
    if not isinstance(answer, dict):
      raise OSError("car answered no JSON object")
    return answer


def as_car_body(destination: dict) -> dict:
  """NavDestination -> the body of the car's /api/destination."""
  return {"latitude": destination["latitude"], "longitude": destination["longitude"],
          "name": destination.get("place_name") or "", "address": destination.get("place_address") or ""}


class Relay:
  """What the server knows of the car, and the destination waiting for it. Kept in a file across restarts."""
  def __init__(self, car, state_dir: str | None = None, clock=time.time, on_late=None):  # noqa: TID251  # queued times are kept across restarts
    self.car = car
    self.clock = clock
    self.on_late = on_late               # called with a destination's body the car took after it had waited for it
    self.lock = threading.RLock()
    self.state_path = os.path.join(state_dir, "navrelay.json") if state_dir else None
    self.pending: dict | None = None     # {"path", "body", "time"}: /api/destination or /api/clear
    self.car_state: dict = {}            # the car's /api/state, last seen
    self.route: dict | None = None       # navd's route, last seen
    self.last_seen: float | None = None
    self.online = False
    self._load()

  def _load(self):
    if self.state_path is None or not os.path.exists(self.state_path):
      return
    try:
      with open(self.state_path, encoding="utf-8") as f:
        s = json.load(f)
    except (OSError, ValueError):
      return
    self.pending, self.car_state, self.route, self.last_seen = s.get("pending"), s.get("car") or {}, s.get("route"), s.get("last_seen")

  def _save(self):
    if self.state_path is None:
      return
    tmp = self.state_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump({"pending": self.pending, "car": self.car_state, "route": self.route, "last_seen": self.last_seen}, f, ensure_ascii=False)
    os.replace(tmp, self.state_path)

  def _call(self, path: str, body: dict | None = None, timeout: float = CAR_TIMEOUT) -> dict:
    try:
      answer = self.car.call(path, body, timeout)
    except OSError:
      self.online = False
      raise
    self.online, self.last_seen = True, self.clock()
    return answer

  def _expired(self) -> bool:
    return self.pending is not None and self.clock() - self.pending["time"] > QUEUE_TTL

  # NavDestination as the server sees it: the one waiting for the car, else the car's
  def destination(self) -> dict | None:
    with self.lock:
      if self.pending is not None and not self._expired():
        if self.pending["path"] == "/api/clear":
          return None
        b = self.pending["body"]
        d = {"latitude": b["latitude"], "longitude": b["longitude"]}
        d.update({k: v for k, v in (("place_name", b.get("name")), ("place_address", b.get("address"))) if v})
        return d
      d = self.car_state.get("destination")
      return d if isinstance(d, dict) else None

  def position(self) -> tuple[float, float] | None:
    car = (self.route or {}).get("car") if self.online else None
    pos = car or self.car_state.get("position")
    return (float(pos[0]), float(pos[1])) if pos else None

  def send(self, path: str, body: dict):
    """A destination (or a clear) for the car: now, or when it is back."""
    with self.lock:
      self.pending = {"path": path, "body": body, "time": self.clock()}
      self._save()
    if self.online:   # else the next poll finds out: no phone waits on a car that is off
      self.deliver(POLL_TIMEOUT, late=False)

  def deliver(self, timeout: float = CAR_TIMEOUT, late: bool = True) -> bool:
    """Send the queued destination to the car, True once it is no longer queued."""
    with self.lock:
      if self.pending is None:
        return True
      if self._expired():
        self.pending = None
        self._save()
        return True
      p = dict(self.pending)
    try:
      answer = self._call(p["path"], p["body"], timeout)
    except OSError:
      return False
    with self.lock:
      if "error" not in answer:
        self.car_state = answer
      if self.pending is not None and self.pending["time"] == p["time"]:
        self.pending = None   # sent, or the car turned it down (an answer is final)
      self._save()
    if late and self.on_late is not None and p["path"] == "/api/destination" and "error" not in answer:
      self.on_late(p["body"])
    return True

  def poll(self):
    """The car's state, then the queued destination."""
    try:
      answer = self._call("/api/state", timeout=POLL_TIMEOUT)
    except OSError:
      return
    with self.lock:
      self.car_state = answer
      self._save()
    self.deliver()

  def live(self) -> dict | None:
    """navd's route while the car navigates, None otherwise (and with the car offline: the page shows the last one then)."""
    if not self.online:
      return None
    try:
      answer = self._call("/api/route?nav=1", timeout=POLL_TIMEOUT)
    except OSError:
      return None
    with self.lock:
      self.route = answer
      self._save()
    return answer if answer.get("route") else None


class CarParams:
  """The params destinationd's Destinations reads and writes, here: NavDestination goes to the car."""
  def __init__(self, relay: Relay):
    self.relay = relay

  def get(self, key: str):
    if key == "NavDestination":
      return self.relay.destination()
    if key == "LastGPSPositionLLK":
      pos = self.relay.position()
      return json.dumps({"latitude": pos[0], "longitude": pos[1]}) if pos else None
    return None

  def get_bool(self, key: str) -> bool:
    if key == "NavigationEnabled":
      return bool(self.relay.car_state.get("navigation", True))
    if key == "IsOffroad":
      return not (self.relay.online and self.relay.car_state.get("onroad"))
    return False

  def put(self, key: str, value: dict, block: bool = False):
    assert key == "NavDestination"
    self.relay.send("/api/destination", as_car_body(value))

  def remove(self, key: str):
    assert key == "NavDestination"
    self.relay.send("/api/clear", {})


class App:
  """The page's API: Destinations on the server, the car behind it."""
  def __init__(self, relay: Relay, dest: destinationd.Destinations, push=None):
    self.relay, self.dest, self.push = relay, dest, push
    self.inbox: dict | None = None   # a share only a search could place: {"query", "places", "time"}, for the page to show
    if push is not None:
      relay.on_late = lambda body: push.notify("车上线了", f"开始导航去 {body.get('name') or '目的地'}")

  def state(self) -> dict:
    return self._with_car(self.dest.state())

  def _with_car(self, answer: dict) -> dict:
    """A state answer, with what the server knows of the car added."""
    if "navigation" not in answer:
      return answer   # an error, places, search results
    with self.relay.lock:
      queued = self.relay.pending is not None and not self.relay._expired()
    return {**answer, "car_online": self.relay.online, "queued": queued, "last_seen": self.relay.last_seen}

  def route(self, phone: tuple[float, float] | None = None) -> dict:
    live = self.relay.live()
    if live is not None:
      return live
    if self.dest.keys is None:   # previews need a key: the car's then
      try:
        return self.relay._call("/api/route")
      except OSError:
        pass
    r = self.dest.route(phone=phone)
    if not self.relay.online and r.get("route") is None and self.relay.route and self.relay.route.get("route"):
      return {**self.relay.route, "offline": True}   # the last navd route seen
    return r

  def get(self, path: str, phone: str | None = None) -> dict | None:
    if path == "/api/state":
      return self.state()
    if path == "/api/places":
      return self.dest.places.as_dict()
    if path == "/api/route":
      return self.route(destinationd.as_origin(phone.split(",")) if phone else None)
    if path == "/api/inbox":   # taken once
      inbox, self.inbox = self.inbox, None
      fresh = inbox is not None and self.relay.clock() - inbox["time"] < INBOX_TTL
      return {"query": inbox["query"], "places": inbox["places"]} if fresh else {}
    if path == "/api/push/key":
      return {"key": self.push.public_key()} if self.push is not None else {"error": "服务器没开推送"}
    return None

  def post(self, path: str, body: dict, notify: bool = False) -> dict:
    """notify: from a Shortcut (the page sees its own answer), a destination set is pushed to the phone."""
    answer = self._with_car(self._post(path, body))
    if "choose" in answer:
      self.inbox = {"query": answer["choose"], "places": answer["places"], "time": self.relay.clock()}
    if notify and self.push is not None and path in SENT:
      d = answer.get("destination")
      if "choose" in answer:
        self.push.notify("选一下目的地", f"没认出准确位置，按“{answer['choose'][:20]}”找到 {len(answer['places'])} 个地点，点开选一个")
      elif "error" in answer:
        self.push.notify("没发到车上", str(answer["error"]))
      elif isinstance(d, dict):
        name = d.get("place_name") or "目的地"
        if answer.get("queued"):
          self.push.notify("等车上线", f"车上线后自动导航去 {name}")
        else:
          self.push.notify("已发到车上", name)
    return answer

  def _post(self, path: str, body: dict) -> dict:
    if path.startswith("/api/push/"):
      if self.push is None:
        return {"error": "服务器没开推送"}
      if path == "/api/push/subscribe":
        return self.push.subscribe(body)
      self.push.notify("推送开好了", "发到车上的地点会在这里提醒")
      return {"sent": True}
    d = self.dest
    if d.keys is None and path in ("/api/search", "/api/share"):   # no key here: the car searches
      try:
        answer = self.relay._call(path, body)
      except OSError:
        return {"error": "车不在线，现在搜不了"}
      if path == "/api/share":
        self.relay.poll()
      return answer
    if path == "/api/search":
      return d.search(str(body.get("query") or ""), destinationd.as_origin(body.get("origin")))
    if path == "/api/destination":
      return d.set_destination(body)
    if path == "/api/clear":
      return d.clear()
    if path == "/api/share":
      return d.share(str(body.get("text") or ""))
    if path == "/api/go":
      return d.go(str(body.get("label") or ""))
    if path == "/api/places":
      return d.save_place(body)
    if path == "/api/preview":
      if d.keys is None:
        return {"route": None, "error": "服务器没有百度 Key，预览不了"}
      return d.preview_route(body)
    return d.delete_place(body)

  def import_places(self):
    """The car's saved and recent places, once, when the server has none yet."""
    p = self.dest.places
    if p.saved or p.recent:
      return
    try:
      car = self.relay._call("/api/places", timeout=POLL_TIMEOUT)
    except OSError:
      return
    for label, place in (car.get("saved") or {}).items():
      p.save(label, place)
    for place in reversed(car.get("recent") or []):
      p.visited(place)


def key_ok(key: str, given: str | None) -> bool:
  return bool(key) and given is not None and hmac.compare_digest(key.encode(), given.encode())


LOGIN = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>导航目的地</title>
<style>
body{{font-family:-apple-system,system-ui,sans-serif;margin:0;padding:16px;background:#0b1116;color:#e8eef2;max-width:640px}}
input{{width:100%;box-sizing:border-box;font-size:17px;padding:10px;border-radius:8px;border:1px solid #33454f;background:#0f1a21;color:#e8eef2;margin:8px 0}}
button{{font-size:16px;padding:10px 14px;border-radius:8px;border:0;background:#2f8f5b;color:#fff}}
.e{{color:#f2b36b}}
</style></head><body>
<h1>导航目的地</h1>{msg}
<form method="post" action="/login"><input name="key" type="password" placeholder="密钥" autocomplete="current-password" autofocus>
<button>进入</button></form>
<p style="color:#93a4b0">输一次，这台手机就记住了。</p>
</body></html>
"""


def handler(app: App, key: str, page: bytes, js_ak: str = ""):
  page = page.replace(b"__BAIDU_JS_AK__", js_ak.encode()) if js_ak else page

  class Handler(BaseHTTPRequestHandler):
    server_version = "navrelay"
    timeout = 15.   # s; a stalled phone does not hold a thread
    sys_version = ""

    def _send(self, status: int, body: bytes, kind: str, cookie: str | None = None, location: str | None = None):
      self.send_response(status)
      self.send_header("Content-Type", kind)
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.send_header("Referrer-Policy", "no-referrer")
      if cookie:
        self.send_header("Set-Cookie", f"{COOKIE}={cookie}; Max-Age=31536000; Path=/; HttpOnly; Secure; SameSite=Strict")
      if location:
        self.send_header("Location", location)
      self.end_headers()
      self.wfile.write(body)

    def _json(self, obj: dict):
      self._send(200, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def _given(self) -> str | None:
      if self.headers.get("X-Key"):
        return self.headers["X-Key"]
      for part in (self.headers.get("Cookie") or "").split(";"):
        name, _, value = part.strip().partition("=")
        if name == COOKIE:
          return value
      return None

    def _authorized(self) -> bool:
      if key_ok(key, self._given()):
        return True
      time.sleep(1.)   # slow down guessing
      return False

    def _body(self) -> bytes:
      return self.rfile.read(min(max(0, int(self.headers.get("Content-Length") or 0)), MAX_BODY))

    def do_GET(self):
      path = urllib.parse.urlsplit(self.path).path
      if path == "/manifest.json":   # no secrets: fetched without the cookie
        self._send(200, MANIFEST, "application/manifest+json")
        return
      if path == "/sw.js":
        self._send(200, SERVICE_WORKER, "text/javascript")
        return
      if not self._authorized():
        if path == "/":   # the key is typed in once, never put in a URL (logs, history, Referer)
          self._send(200, LOGIN.format(msg="").encode(), "text/html; charset=utf-8")
        else:
          self._send(403, b"", "text/plain")
        return
      if path == "/":
        self._send(200, page, "text/html; charset=utf-8")
        return
      answer = app.get(path, self.headers.get("X-From"))
      if answer is None:
        self._send(404, b"", "text/plain")
      else:
        self._json(answer)

    def do_POST(self):
      path = urllib.parse.urlsplit(self.path).path
      if path == "/login":
        given = urllib.parse.parse_qs(self._body().decode(errors="replace")).get("key", [""])[0].strip()
        if key_ok(key, given):
          self._send(303, b"", "text/plain", cookie=given, location="/")
        else:
          time.sleep(1.)
          self._send(200, LOGIN.format(msg="<p class=e>密钥不对</p>").encode(), "text/html; charset=utf-8")
        return
      if not self._authorized():
        self._send(403, b"", "text/plain")
        return
      try:
        body = json.loads(self._body() or b"{}")
      except ValueError:
        body = None
      if path not in POSTS or not isinstance(body, dict):
        self._send(400 if path in POSTS else 404, b"", "text/plain")
        return
      self._json(app.post(path, body, notify=bool(self.headers.get("X-Key"))))   # the page sends the cookie

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # keys and places stay off the logs
      pass

  return Handler


def keys_from_env() -> tuple[str, str] | None:
  ak, sk = os.environ.get("NAVRELAY_BAIDU_AK", ""), os.environ.get("NAVRELAY_BAIDU_SK", "")
  return (ak, sk) if ak and sk else None


def main():
  logging.basicConfig(format="%(name)s: %(message)s")   # to the journal: links shares were not understood from
  key = os.environ.get("NAVRELAY_KEY", "")
  if len(key) < 24:
    raise SystemExit("NAVRELAY_KEY must be set, at least 24 characters")
  state = os.environ.get("NAVRELAY_STATE", "/var/lib/navrelay")
  relay = Relay(Car(os.environ.get("NAVRELAY_CAR", "http://100.69.175.43:8082")), state)
  dest = destinationd.Destinations(CarParams(relay), keys_from_env(), count_file=os.path.join(state, "baidu_calls"),
                                   places=destinationd.Places(os.path.join(state, "places.json")))
  push = None
  if webpush is not None:
    push = webpush.Push(state, os.environ.get("NAVRELAY_PUSH_SUBJECT", "https://comma.cd.xiaoxx.cc"))
  app = App(relay, dest, push)
  with open(os.environ.get("NAVRELAY_PAGE", "/opt/navrelay/destination_page.html"), "rb") as f:
    page = f.read()
  js_ak = os.environ.get("NAVRELAY_JS_AK", "")
  address = (os.environ.get("NAVRELAY_BIND", "127.0.0.1"), int(os.environ.get("NAVRELAY_PORT", "8089")))
  server = ThreadingHTTPServer(address, handler(app, key, page, js_ak if js_ak.isalnum() else ""))
  server.daemon_threads = True
  stop = threading.Event()

  def background():
    while True:
      relay.poll()
      if relay.online:
        app.import_places()
      if stop.wait(POLL):
        return
  threading.Thread(target=background, daemon=True).start()
  try:
    server.serve_forever()
  finally:
    stop.set()


if __name__ == "__main__":
  main()
