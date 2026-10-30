#!/usr/bin/env python3
"""Navigation destinations from the public internet, for a phone that is not on Tailscale.

Runs on the user's server, which is on the tailnet with the car. It passes the car's destination page and its API
(destinationd, port 8082, Tailscale only) through to the phone behind a secret key, and keeps the last destination
sent while the car is offline until the car is back. It listens on 127.0.0.1, plain http, behind the server's reverse
proxy (1Panel's openresty), which does https for the domain.

  phone --https + key--> openresty --> this relay --tailnet--> car destinationd

- Key: typed into the page once (kept as a cookie), or the X-Key header (iOS Shortcuts). Never in a URL: access logs,
  history and Referer headers would keep it.
- Car offline: share / go / destination / clear are queued (newest wins, QUEUE_TTL), the page is the last copy seen,
  /api/state shows the queued one, search answers that the car is offline.
- Python stdlib only. Config from the environment, see install.sh.
"""
from __future__ import annotations

import hmac
import json
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

QUEUED = ("/api/share", "/api/go", "/api/destination", "/api/clear")
PASSED = QUEUED + ("/api/search", "/api/places", "/api/places/delete")
CAR_TIMEOUT = 8.       # s for one call to the car
RETRY = 20.            # s between deliveries of the queued destination
QUEUE_TTL = 3 * 3600.  # s a queued destination waits for the car
MAX_BODY = 4096
COOKIE = "navkey"


class Car:
  """The car's destinationd over the tailnet."""
  def __init__(self, url: str, timeout: float = CAR_TIMEOUT):
    self.url = url.rstrip("/")
    self.timeout = timeout

  def call(self, path: str, body: dict | None = None) -> tuple[int, bytes, str]:
    """(status, body, content type); raises OSError when the car is not reachable."""
    data = None if body is None else json.dumps(body, ensure_ascii=False).encode()
    req = urllib.request.Request(self.url + path, data, {"Content-Type": "application/json"} if data else {})
    try:
      with urllib.request.urlopen(req, timeout=self.timeout) as r:
        return r.status, r.read(), r.headers.get("Content-Type", "application/json")
    except urllib.error.HTTPError as e:
      return e.code, e.read(), e.headers.get("Content-Type", "text/plain")


class Relay:
  def __init__(self, car, state_dir: str | None = None, clock=time.time):  # noqa: TID251  # queued times are kept across restarts
    self.car = car
    self.clock = clock
    self.lock = threading.Lock()
    self.state_path = os.path.join(state_dir, "navrelay.json") if state_dir else None
    self.pending: dict | None = None   # {"path", "body", "time"}
    self.page: bytes | None = None     # the car's page, last seen
    self.places: dict | None = None    # the car's places, last seen
    self.last_seen: float | None = None
    self._load()

  # the car's answers kept for when it is offline
  def _load(self):
    if self.state_path is None or not os.path.exists(self.state_path):
      return
    try:
      with open(self.state_path, encoding="utf-8") as f:
        s = json.load(f)
    except (OSError, ValueError):
      return
    self.pending, self.places, self.last_seen = s.get("pending"), s.get("places"), s.get("last_seen")
    self.page = s["page"].encode() if s.get("page") else None

  def _save(self):
    if self.state_path is None:
      return
    s = {"pending": self.pending, "places": self.places, "last_seen": self.last_seen,
         "page": self.page.decode() if self.page else None}
    tmp = self.state_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(s, f, ensure_ascii=False)
    os.replace(tmp, self.state_path)

  def _seen(self, path: str, status: int, body: bytes):
    self.last_seen = self.clock()
    if status != 200:
      return
    if path == "/":
      self.page = body
    elif path in ("/api/places", "/api/places/delete"):
      try:
        self.places = json.loads(body)
      except ValueError:
        pass

  def _expired(self) -> bool:
    return self.pending is not None and self.clock() - self.pending["time"] > QUEUE_TTL

  def offline_state(self) -> dict:
    s = {"navigation": True, "destination": None, "onroad": False, "offline": True, "last_seen": self.last_seen}
    if self.pending is not None and not self._expired():
      p, b = self.pending["path"], self.pending["body"]
      if p == "/api/destination":
        name = b.get("name") or f"{b.get('latitude')}, {b.get('longitude')}"
      elif p == "/api/go":
        name = b.get("label") or ""
      elif p == "/api/share":
        name = (b.get("text") or "").split("http")[0].strip()[:40] or "分享的地点"
      else:
        name = None
      if name is not None:
        s["destination"] = {"place_name": f"等车上线：{name}", "latitude": 0., "longitude": 0.}
      s["queued"] = p
    return s

  def handle(self, method: str, path: str, body: dict | None) -> tuple[int, bytes, str]:
    try:
      status, out, kind = self.car.call(path, body if method == "POST" else None)
    except OSError:
      return self._offline(method, path, body)
    with self.lock:
      self._seen(path, status, out)
      if status == 200 and path in QUEUED:
        self.pending = None   # a newer destination replaces the queued one
      self._save()
    return status, out, kind

  def _offline(self, method: str, path: str, body: dict | None) -> tuple[int, bytes, str]:
    with self.lock:
      if method == "POST" and path in QUEUED:
        self.pending = {"path": path, "body": body or {}, "time": self.clock()}
        self._save()
        return _json({**self.offline_state(), "queued_until_online": True})
      if path == "/":
        if self.page is None:
          return 503, "车不在线，还没存过页面。车开机后再打开一次。".encode(), "text/plain; charset=utf-8"
        return 200, self.page, "text/html; charset=utf-8"
      if path == "/api/state":
        return _json(self.offline_state())
      if path == "/api/places":
        return _json(self.places or {"saved": {}, "recent": []})
      if path == "/api/search":
        return _json({"error": "车不在线，现在搜不了。可以粘贴百度地图分享链接，或者点常用地点，车上线后自动发过去。"})
      return _json({"error": "车不在线"})

  def deliver(self) -> bool:
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
      status, out, _ = self.car.call(p["path"], p["body"])
    except OSError:
      return False
    with self.lock:
      self.last_seen = self.clock()
      if self.pending is not None and self.pending["time"] == p["time"]:
        self.pending = None   # sent, or the car turned it down (an answer is final)
      self._save()
    return True

  def run_deliveries(self, stop: threading.Event):
    while not stop.wait(RETRY):
      self.deliver()


def _json(obj: dict, status: int = 200) -> tuple[int, bytes, str]:
  return status, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8"


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


def handler(relay: Relay, key: str):
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
      if not self._authorized():
        if path == "/":   # the key is typed in once, never put in a URL (logs, history, Referer)
          self._send(200, LOGIN.format(msg="").encode(), "text/html; charset=utf-8")
        else:
          self._send(403, b"", "text/plain")
        return
      if path not in ("/", "/api/state", "/api/places"):
        self._send(404, b"", "text/plain")
        return
      self._send(*relay.handle("GET", path, None))

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
      if path not in PASSED or not isinstance(body, dict):
        self._send(400 if path in PASSED else 404, b"", "text/plain")
        return
      self._send(*relay.handle("POST", path, body))

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # keys and places stay off the logs
      pass

  return Handler


def main():
  key = os.environ.get("NAVRELAY_KEY", "")
  if len(key) < 24:
    raise SystemExit("NAVRELAY_KEY must be set, at least 24 characters")
  relay = Relay(Car(os.environ.get("NAVRELAY_CAR", "http://100.69.175.43:8082")), os.environ.get("NAVRELAY_STATE", "/var/lib/navrelay"))
  address = (os.environ.get("NAVRELAY_BIND", "127.0.0.1"), int(os.environ.get("NAVRELAY_PORT", "8089")))
  server = ThreadingHTTPServer(address, handler(relay, key))
  stop = threading.Event()
  threading.Thread(target=relay.run_deliveries, args=(stop,), daemon=True).start()
  try:
    server.serve_forever()
  finally:
    stop.set()


if __name__ == "__main__":
  main()
