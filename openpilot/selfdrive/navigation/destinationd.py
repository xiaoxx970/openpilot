#!/usr/bin/env python3
"""A small web page to set the navigation destination from a phone, without comma connect or prime.

Open http://<the device's Tailscale name>:8082 on a phone in the same tailnet: search a place (Baidu's place suggestions,
with the key file of road_events/baidu.py), paste a Baidu Maps share ("分享 > 复制链接") or "latitude,longitude"
(WGS-84), or pick a saved or recent place, and it becomes NavDestination, the param navd routes to. The same API serves
iOS Shortcuts: POST /api/share {"text": ...} from the share sheet, POST /api/go {"label": "家"} for Siri.
It listens on the Tailscale interface only (the cellular network's CGNAT shares Tailscale's address range, so the peer
address alone would not do) and answers Tailscale peers only; the keys never leave the device.

NavDestination is cleared when the car goes offroad and when manager starts: set it before or after starting the car,
for the drive at hand.
"""
import fcntl
import json
import math
import os
import re
import socket
import struct
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import ip_address, ip_network

from openpilot.selfdrive.navigation import baidu_route
from openpilot.selfdrive.road_events import baidu

PORT = 8082
SUGGESTION_PATH = "/place/v2/suggestion"
DETAIL_PATH = "/place/v2/detail"
PLACES_FILE = "/data/navigation/places.json"   # saved places by label, recent destinations
MAX_RECENT = 8
SHARE_URL = re.compile(r"https?://(?:j\.)?map\.baidu\.com/[^\s\"'<>]+")
UID = re.compile(r"uid(?:=|%3D)([0-9a-f]{24})")
COUNT_FILE = "/data/baidu_place_calls"
ALLOWED = (ip_network("100.64.0.0/10"), ip_network("fd7a:115c:a1e0::/48"), ip_network("127.0.0.0/8"), ip_network("::1/128"))
MAX_RESULTS = 10
INTERFACE = "tailscale0"
PREVIEW_STALE = 600.     # s a route preview is shown before it is planned again: its traffic ages
PREVIEW_MIN_GAP = 3.     # s between two previews: a tap at a time, not a script
PREVIEW_MOVED = 300.     # m the start may move and a kept preview still be used
PREVIEW_KEEP = 8         # places whose preview is kept, for going back and forth between them
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


def parse_coordinates(text: str) -> tuple[float, float] | None:
  """(latitude, longitude) of a pasted "lat,lon" (WGS-84), None when it is not one."""
  parts = text.replace("，", ",").replace(" ", ",").split(",")
  parts = [p for p in parts if p]
  if len(parts) != 2:
    return None
  try:
    lat, lon = float(parts[0]), float(parts[1])
  except ValueError:
    return None
  return (lat, lon) if -90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(lat + lon) else None


def as_origin(value) -> tuple[float, float] | None:
  """A start the page sends ([lat, lon], WGS-84: the phone's position when the car's is not known), None when it is not one."""
  try:
    lat, lon = float(value[0]), float(value[1])
  except (TypeError, ValueError, IndexError, KeyError):
    return None
  return (lat, lon) if -90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(lat + lon) else None


def suggestion_params(query: str, near: tuple[float, float] | None) -> list[tuple[str, str]]:
  params = [("query", query), ("region", "全国"), ("city_limit", "false"), ("ret_coordtype", "gcj02ll"), ("output", "json")]
  if near is not None:
    params += [("location", f"{near[0]:.6f},{near[1]:.6f}"), ("coord_type", "1")]   # 1: WGS-84
  return params


def parse_suggestions(resp: dict) -> list[dict]:
  """Places with a position, WGS-84: name, address, latitude, longitude."""
  places = []
  for r in resp.get("result") or [] if resp.get("status") == 0 else []:
    loc = r.get("location") or {}
    if "lat" not in loc or "lng" not in loc or not r.get("name"):
      continue   # bus lines and the like come without one
    lat, lon = baidu.gcj02_to_wgs84(float(loc["lat"]), float(loc["lng"]))
    address = r.get("address") or " ".join(p for p in (r.get("city"), r.get("district")) if p)   # its address has them already
    places.append({"name": r["name"], "address": address, "latitude": round(lat, 6), "longitude": round(lon, 6)})
  return places[:MAX_RESULTS]


class _NoRedirect(urllib.request.HTTPRedirectHandler):
  def redirect_request(self, req, fp, code, msg, headers, newurl):
    return None


def redirect_target(url: str) -> str:
  """Where a Baidu short link points, without following it: the uid is in that address."""
  opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect)
  try:
    with opener.open(urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"}), timeout=baidu.TIMEOUT) as r:
      return r.headers.get("Location") or ""
  except urllib.error.HTTPError as e:   # the 3xx itself, not followed
    return e.headers.get("Location") or ""


def share_uid(text: str, resolve=redirect_target) -> str | None:
  """The Baidu place id of a Baidu Maps share ("大行宫 大行宫 https://j.map.baidu.com/f9/Z3J"), None without one."""
  match = SHARE_URL.search(text)
  if match is None:
    return None
  url = match.group(0)
  uid = UID.search(url)
  if uid is None and urllib.parse.urlsplit(url).hostname == "j.map.baidu.com":
    uid = UID.search(resolve(url))
  return uid.group(1) if uid else None


def parse_detail(resp: dict) -> dict | None:
  r = resp.get("result") or {} if resp.get("status") == 0 else {}
  loc = r.get("location") or {}
  if "lat" not in loc or "lng" not in loc:
    return None
  lat, lon = baidu.gcj02_to_wgs84(float(loc["lat"]), float(loc["lng"]))
  return {"name": r.get("name") or "", "address": r.get("address") or "", "latitude": round(lat, 6), "longitude": round(lon, 6)}


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

  def save(self, label: str, place: dict):
    self.saved[label] = place
    self._write()

  def delete(self, label: str):
    self.saved.pop(label, None)
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
  """What the page does, apart from HTTP: tests drive it with fake params and a fake fetch."""
  def __init__(self, params, keys=None, fetch=baidu.http_get, count_file: str | None = COUNT_FILE, places: Places | None = None,
               resolve=redirect_target, live=None, clock=time.monotonic):
    self.params = params
    self.live = live      # navd's route while driving: {points, jams, distance, duration, car}, None without one
    self.clock = clock
    self.previews: list[dict] = []   # newest first
    self.preview_tried = -math.inf
    self.keys = keys
    self.fetch = fetch
    self.resolve = resolve
    self.daily = baidu.DailyCount(count_file)
    self.places = places if places is not None else Places(None)

  def state(self) -> dict:
    destination = self.params.get("NavDestination")
    current = self.current()
    return {"destination": destination, "saved_as": self.places.label_of(current) if current is not None else None,
            "navigation": self.params.get_bool("NavigationEnabled"),
            "onroad": not self.params.get_bool("IsOffroad"), "search": self.keys is not None, "position": self.near()}

  def near(self) -> tuple[float, float] | None:
    try:
      pos = json.loads(self.params.get("LastGPSPositionLLK") or "")
      return float(pos["latitude"]), float(pos["longitude"])
    except (ValueError, KeyError, TypeError):
      return None

  def bias(self, phone: tuple[float, float] | None = None) -> tuple[float, float] | None:
    """Where searches look first: the car, else the phone, else the destination, else a saved place (navrelay may not know
    where the car is)."""
    near = self.near() or phone
    if near is not None:
      return near
    place = self.current() or next(iter(self.places.saved.values()), None)
    return (place["latitude"], place["longitude"]) if place is not None else None

  def _baidu(self, path: str, params: list, kind: str = "place") -> dict:
    """One API call within the daily budget: the answer, or {"error": ...}."""
    if self.keys is None:
      return {"error": "没有百度 Key（/data/baidu_ak），只能粘贴“纬度,经度”"}
    now = baidu.unix_now()
    if self.daily.left(kind, now) <= 0:
      return {"error": "今天的地点查询次数用完了" if kind == "place" else "今天的路线预览次数用完了"}
    self.daily.add(kind, now)
    try:
      resp = self.fetch(baidu.signed_get(path, params, *self.keys))
    except Exception as e:   # only the type: a message could hold the signed URL
      return {"error": f"查询失败（{type(e).__name__}）"}
    if resp.get("status") != 0:
      return {"error": f"百度返回 {resp.get('status')}"}
    return resp

  def search(self, query: str, phone: tuple[float, float] | None = None) -> dict:
    query = query.strip()[:200]
    if not query:
      return {"places": []}
    pasted = parse_coordinates(query)
    if pasted is not None:
      return {"places": [{"name": query, "address": "", "latitude": pasted[0], "longitude": pasted[1]}]}
    if SHARE_URL.search(query):
      place = self.share_place(query)
      return {"places": [place]} if "error" not in place else place
    resp = self._baidu(SUGGESTION_PATH, suggestion_params(query[:64], self.bias(phone)))
    return resp if "error" in resp else {"places": parse_suggestions(resp)}

  def share_place(self, text: str) -> dict:
    """The place of a Baidu Maps share, or pasted coordinates, or {"error": ...}."""
    pasted = parse_coordinates(text.strip())
    if pasted is not None:
      return {"name": "", "address": "", "latitude": pasted[0], "longitude": pasted[1]}
    try:
      uid = share_uid(text, self.resolve)
    except Exception as e:
      return {"error": f"打不开分享链接（{type(e).__name__}）"}
    if uid is None:
      return {"error": "没认出百度地图的分享链接"}
    resp = self._baidu(DETAIL_PATH, [("uid", uid), ("ret_coordtype", "gcj02ll"), ("scope", "1"), ("output", "json")])
    if "error" in resp:
      return resp
    place = parse_detail(resp)
    return place if place is not None else {"error": "这个地点没有坐标"}

  def share(self, text: str) -> dict:
    """A share from the phone becomes the destination at once (iOS Shortcuts' share sheet)."""
    place = self.share_place(str(text)[:500])
    return place if "error" in place else self.set_destination(place)

  def go(self, label: str) -> dict:
    place = self.places.saved.get(label)
    return self.set_destination(place) if place is not None else {"error": f"没有叫“{label}”的地点"}

  def save_place(self, body: dict) -> dict:
    label = str(body.get("label") or "").strip()[:16]
    if not label:
      return {"error": "要起个名字"}
    place = as_place(body["place"]) if isinstance(body.get("place"), dict) else self.current()
    if place is None:
      return {"error": "没有要存的地点"}
    self.places.save(label, place)
    return self.places.as_dict()

  def delete_place(self, body: dict) -> dict:
    self.places.delete(str(body.get("label") or ""))
    return self.places.as_dict()

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

  def route(self, nav_only: bool = False, phone: tuple[float, float] | None = None) -> dict:
    """The route to show on the page: navd's while it navigates, else a preview from where the car is, or the phone without
    the car's position (not with nav_only: navrelay plans its previews itself)."""
    live = self.live() if self.live is not None else None
    if live is not None:
      return {"route": {"source": "nav", "segments": route_segments(live["points"], live["jams"]), "distance": live["distance"],
                        "duration": live["duration"]}, "car": live["car"]}
    near = self.near()
    car = [near[0], near[1], None] if near is not None else None
    d = self.current()
    if d is None or nav_only:
      return {"route": None, "car": car}
    preview = self._preview(d, near or phone)
    if "error" in preview:
      return {"route": None, "car": car, "error": preview["error"]}
    return {"route": {"source": "preview", "segments": preview["segments"], "distance": preview["distance"],
                      "duration": preview["duration"], "age": round(self.clock() - preview["time"]), "from": "car" if near else "phone"},
            "car": car}

  def preview_route(self, body: dict) -> dict:
    """A preview to a place before it becomes the destination (the page shows it first)."""
    place = as_place(body)
    if place is None:
      return {"error": "坐标不对"}
    car = self.near()
    near = car or as_origin(body.get("origin"))
    p = self._preview(place, near)
    if "error" in p:
      return {"route": None, "error": p["error"]}
    return {"route": {"source": "preview", "segments": p["segments"], "distance": p["distance"], "duration": p["duration"],
                      "age": round(self.clock() - p["time"]), "from": "car" if car else "phone"},
            "car": [car[0], car[1], None] if car else None}

  def _preview(self, d: dict, near: tuple[float, float] | None) -> dict:
    now = self.clock()

    def same_start(q: dict) -> bool:
      return near is not None and math.hypot(*baidu.ground(q["from"][0], q["from"][1], near[0], near[1])) < PREVIEW_MOVED
    p = next((q for q in self.previews if same_place(q["to"], d) and same_start(q)), None)
    if p is not None and now - p["time"] < PREVIEW_STALE:
      return p
    if near is None:
      return {"error": "不知道车和手机在哪，没法预览路线（手机上允许网页使用位置就行）"}
    if now - self.preview_tried < PREVIEW_MIN_GAP:
      return p if p is not None else {"error": "稍后再试"}
    self.preview_tried = now
    resp = self._baidu(baidu.ROUTE_PATH, baidu.route_params(near[0], near[1], 0., 0., (d["latitude"], d["longitude"])), "preview")
    if "error" in resp:
      return resp
    parsed = baidu_route.parse(resp)
    if parsed is None:
      return {"error": "百度没有给出路线"}
    steps, points, jams = parsed
    p = {"to": d, "from": near, "time": now, "segments": route_segments(points, jams),
         "distance": round(sum(st["distance"] for st in steps)), "duration": round(sum(st["duration"] for st in steps))}
    self.previews = ([p] + [q for q in self.previews if not (same_place(q["to"], d) and same_start(q))])[:PREVIEW_KEEP]
    return p


PAGE = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "destination_page.html"), encoding="utf-8").read()   # also served
# by tools/navrelay from the user's server, which puts the map's browser AK in: changes to it need no install on the car


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
      if path == "/":
        self._send(200, PAGE.encode(), "text/html; charset=utf-8")
      elif path == "/api/state":
        self._json(dest.state())
      elif path == "/api/places":
        self._json(dest.places.as_dict())
      elif path == "/api/route":
        phone = self.headers.get("X-From")   # the phone's position: a header, never in a URL
        self._json(dest.route(nav_only="nav=1" in urllib.parse.urlsplit(self.path).query, phone=as_origin(phone.split(",")) if phone else None))
      else:
        self._send(404, b"", "text/plain")

    def do_POST(self):
      if not self._allowed():
        return
      try:
        body = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length") or 0), 4096)) or b"{}")
      except ValueError:
        body = None
      if not isinstance(body, dict):
        self._send(400, b"", "text/plain")
        return
      path = urllib.parse.urlsplit(self.path).path
      if path == "/api/search":
        self._json(dest.search(str(body.get("query") or ""), as_origin(body.get("origin"))))
      elif path == "/api/destination":
        self._json(dest.set_destination(body))
      elif path == "/api/clear":
        self._json(dest.clear())
      elif path == "/api/share":
        self._json(dest.share(str(body.get("text") or "")))
      elif path == "/api/go":
        self._json(dest.go(str(body.get("label") or "")))
      elif path == "/api/places":
        self._json(dest.save_place(body))
      elif path == "/api/places/delete":
        self._json(dest.delete_place(body))
      elif path == "/api/preview":
        self._json(dest.preview_route(body))
      else:
        self._send(404, b"", "text/plain")

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override; queries stay off the logs
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

  dest = Destinations(Params(), baidu.load_keys(), places=Places(), live=live_route())
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
