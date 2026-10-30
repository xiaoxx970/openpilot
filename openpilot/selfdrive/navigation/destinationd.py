#!/usr/bin/env python3
"""The car's navigation API for comma-link, the owner's server (its own repository), which is the phone's web app: search,
map-app shares, saved places and route previews all happen there, with its own Baidu key. The car only takes destinations
and says where it is:

  GET /api/state          navigation on, the destination, onroad, position
  POST /api/destination   {latitude, longitude, name, address} (WGS-84) becomes NavDestination, the param navd routes to
  POST /api/clear
  GET /api/route?nav=1    navd's live route and traffic while navigating
  GET /api/places         the recent destinations (comma-link imports them once)

It listens on the Tailscale interface only (the cellular network's CGNAT shares Tailscale's address range, so the peer
address alone would not do) and answers Tailscale peers only.

NavDestination is cleared when the car goes offroad and when manager starts: set it before or after starting the car,
for the drive at hand.
"""
import fcntl
import json
import math
import os
import socket
import struct
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import ip_address, ip_network

from openpilot.selfdrive.navigation import baidu_route
from openpilot.selfdrive.road_events import baidu

PORT = 8082
PLACES_FILE = "/data/navigation/places.json"   # saved places by label, recent destinations
MAX_RECENT = 8
ALLOWED = (ip_network("100.64.0.0/10"), ip_network("fd7a:115c:a1e0::/48"), ip_network("127.0.0.0/8"), ip_network("::1/128"))
INTERFACE = "tailscale0"
MAX_ROUTE_POINTS = 600   # a route on the page, thinned: it goes over cellular
SIOCGIFADDR = 0x8915


def allowed(address: str) -> bool:
  try:
    ip = ip_address(address)
  except ValueError:
    return False
  if ip.version == 6 and ip.ipv4_mapped is not None:
    ip = ip.ipv4_mapped
  return any(ip.version == net.version and ip in net for net in ALLOWED)


def interface_ip(name: str = INTERFACE) -> str | None:
  """The IPv4 address of a network interface, None without one."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      return socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, struct.pack("256s", name.encode()[:15]))[20:24])
  except OSError:
    return None


def same_place(a: dict, b: dict) -> bool:
  return abs(a["latitude"] - b["latitude"]) < 1e-4 and abs(a["longitude"] - b["longitude"]) < 1e-4


def route_segments(points: list[tuple[float, float]], jams: list[tuple[float, float, int]]) -> list[dict]:
  """The route as runs of one traffic status (1 smooth, 2 slow, 3 congested, 4 severe), each {status, points [[lat, lon]]},
  split where the jams (start, length, status along it) start and end; thinned to at most about MAX_ROUTE_POINTS points."""
  if len(points) < 2:
    return []
  path = baidu.as_path(points)
  step = max(1, -(-len(path) // MAX_ROUTE_POINTS))
  kept = path[::step] + ([path[-1]] if (len(path) - 1) % step else [])
  cuts = sorted({0., path[-1][0], *(c for s0, n, _ in jams for c in (s0, s0 + n) if 0. < c < path[-1][0])})
  out: list[dict] = []
  for a, b in zip(cuts, cuts[1:], strict=False):
    mid = (a + b) / 2
    status = next((st for s0, n, st in jams if s0 <= mid < s0 + n), 1)
    run = [baidu_route.point_at(path, a)] + [(la, lo) for s, la, lo in kept if a < s < b] + [baidu_route.point_at(path, b)]
    run = [[round(la, 6), round(lo, 6)] for la, lo in run]
    if out and out[-1]["status"] == status:
      out[-1]["points"] += run[1:]
    else:
      out.append({"status": status, "points": run})
  return out


class Places:
  """Saved places by label and the recent destinations, in a file (in memory without one)."""
  def __init__(self, path: str | None = PLACES_FILE):
    self.path = path
    self.saved: dict[str, dict] = {}
    self.recent: list[dict] = []
    try:
      with open(path or "") as f:
        data = json.load(f)
      self.saved, self.recent = dict(data.get("saved") or {}), list(data.get("recent") or [])[:MAX_RECENT]
    except (OSError, ValueError, AttributeError):
      pass

  def as_dict(self) -> dict:
    return {"saved": self.saved, "recent": self.recent}

  def _write(self):
    if self.path is None:
      return
    try:
      os.makedirs(os.path.dirname(self.path), exist_ok=True)
      with open(self.path + ".tmp", "w") as f:
        json.dump(self.as_dict(), f, ensure_ascii=False)
      os.replace(self.path + ".tmp", self.path)
    except OSError:
      pass

  def label_of(self, place: dict) -> str | None:
    return next((k for k, p in self.saved.items() if same_place(p, place)), None)

  def visited(self, place: dict):
    if self.label_of(place) is not None:
      return   # a saved place is one tap away already
    self.recent = ([place] + [p for p in self.recent if not same_place(p, place)])[:MAX_RECENT]
    self._write()


def as_place(body) -> dict | None:
  """{name, address, latitude, longitude} of a request body, None when its coordinates are not valid."""
  try:
    lat, lon = float(body["latitude"]), float(body["longitude"])
  except (KeyError, TypeError, ValueError):
    return None
  if not (-90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(lat + lon)):
    return None
  name = body.get("name") if isinstance(body.get("name"), str) else ""
  address = body.get("address") if isinstance(body.get("address"), str) else ""
  return {"name": name[:64], "address": address[:96], "latitude": lat, "longitude": lon}


class Destinations:
  """What the API does, apart from HTTP: tests drive it with fake params."""
  def __init__(self, params, places: Places | None = None, live=None):
    self.params = params
    self.live = live      # navd's route while driving: {points, jams, distance, duration, car}, None without one
    self.places = places if places is not None else Places(None)

  def state(self) -> dict:
    current = self.current()
    return {"destination": self.params.get("NavDestination"),
            "saved_as": self.places.label_of(current) if current is not None else None,
            "navigation": self.params.get_bool("NavigationEnabled"), "onroad": not self.params.get_bool("IsOffroad"),
            "position": self.near()}

  def near(self) -> tuple[float, float] | None:
    try:
      pos = json.loads(self.params.get("LastGPSPositionLLK") or "")
      return float(pos["latitude"]), float(pos["longitude"])
    except (ValueError, KeyError, TypeError):
      return None

  def current(self) -> dict | None:
    d = self.params.get("NavDestination")
    if not isinstance(d, dict):
      return None
    return as_place({"name": d.get("place_name") or "", "address": d.get("place_address") or "", **d})

  def set_destination(self, body: dict) -> dict:
    place = as_place(body)
    if place is None:
      return {"error": "坐标不对"}
    destination: dict = {"latitude": place["latitude"], "longitude": place["longitude"]}
    if place["name"]:
      destination["place_name"] = place["name"]
    if place["address"]:
      destination["place_address"] = place["address"]
    self.params.put("NavDestination", destination, block=True)   # the answer reads it back
    self.places.visited(place)
    return self.state()

  def clear(self) -> dict:
    self.params.remove("NavDestination")
    return self.state()

  def route(self) -> dict:
    """navd's route while it navigates, else none; where the car is either way. Previews are comma-link's."""
    live = self.live() if self.live is not None else None
    if live is not None:
      return {"route": {"source": "nav", "segments": route_segments(live["points"], live["jams"]), "distance": live["distance"],
                        "duration": live["duration"]}, "car": live["car"]}
    near = self.near()
    return {"route": None, "car": [near[0], near[1], None] if near is not None else None}


def handler(dest: Destinations):
  class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: bytes, kind: str):
      self.send_response(code)
      self.send_header("Content-Type", kind)
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(body)

    def _json(self, obj: dict):
      self._send(200, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def _allowed(self) -> bool:
      if allowed(self.client_address[0]):
        return True
      self._send(403, b"", "text/plain")
      return False

    def do_GET(self):
      if not self._allowed():
        return
      path = urllib.parse.urlsplit(self.path).path
      if path == "/api/state":
        self._json(dest.state())
      elif path == "/api/places":
        self._json(dest.places.as_dict())
      elif path == "/api/route":
        self._json(dest.route())
      else:
        self._send(404, b"", "text/plain")

    def do_POST(self):
      if not self._allowed():
        return
      try:
        body = json.loads(self.rfile.read(min(max(0, int(self.headers.get("Content-Length") or 0)), 4096)) or b"{}")
      except ValueError:
        body = None
      if not isinstance(body, dict):
        self._send(400, b"", "text/plain")
        return
      path = urllib.parse.urlsplit(self.path).path
      if path == "/api/destination":
        self._json(dest.set_destination(body))
      elif path == "/api/clear":
        self._json(dest.clear())
      else:
        self._send(404, b"", "text/plain")

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override; places stay off the logs
      pass

  return Handler


LIVE_MAX_AGE = 15.   # s since navd last sent its route (it does every 10 s): older, it stopped navigating


def live_route():
  """navd's current route, for Destinations.live: read on each page poll, from any of the server's threads."""
  import threading

  import openpilot.cereal.messaging as messaging
  from openpilot.selfdrive.navigation.helpers import latest_gps_source

  sm = messaging.SubMaster(['navRouteIC', 'navInstruction', 'gpsLocation', 'gpsLocationExternal'])
  lock = threading.Lock()

  def get() -> dict | None:
    with lock:
      sm.update(0)
      now = time.monotonic()
      r = sm['navRouteIC']
      if now - sm.recv_time['navRouteIC'] > LIVE_MAX_AGE or not r.requestTime or now - r.requestTime * 1e-9 > baidu.NAV_STALE:
        return None
      points = [(c.latitude, c.longitude) for c in r.coordinates]
      if len(points) < 2:
        return None
      ins = sm['navInstruction']
      fresh = sm.valid['navInstruction'] and now - sm.recv_time['navInstruction'] < LIVE_MAX_AGE
      source = latest_gps_source(sm)
      gps = sm[source] if source is not None else None
      return {"points": points, "jams": [(j.start, j.length, j.status) for j in r.jams],
              "distance": round(ins.distanceRemaining) if fresh else None, "duration": round(ins.timeRemaining) if fresh else None,
              "car": [gps.latitude, gps.longitude, gps.bearingDeg] if gps is not None else None}
  return get


def main():
  from openpilot.common.params import Params

  dest = Destinations(Params(), places=Places(), live=live_route())
  while True:
    address = interface_ip()
    if address is not None:
      try:
        server = ThreadingHTTPServer((address, PORT), handler(dest))
        break
      except OSError:   # the port is still held by a previous run
        pass
    time.sleep(10)   # Tailscale not up yet
  server.daemon_threads = True
  server.serve_forever()


if __name__ == "__main__":
  main()
