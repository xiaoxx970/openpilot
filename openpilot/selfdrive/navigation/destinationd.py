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

  def visited(self, place: dict):
    same = lambda p: abs(p["latitude"] - place["latitude"]) < 1e-4 and abs(p["longitude"] - place["longitude"]) < 1e-4  # noqa: E731
    self.recent = ([place] + [p for p in self.recent if not same(p)])[:MAX_RECENT]
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
               resolve=redirect_target):
    self.params = params
    self.keys = keys
    self.fetch = fetch
    self.resolve = resolve
    self.daily = baidu.DailyCount(count_file)
    self.places = places if places is not None else Places(None)

  def state(self) -> dict:
    destination = self.params.get("NavDestination")
    return {"destination": destination, "navigation": self.params.get_bool("NavigationEnabled"),
            "onroad": not self.params.get_bool("IsOffroad"), "search": self.keys is not None}

  def near(self) -> tuple[float, float] | None:
    try:
      pos = json.loads(self.params.get("LastGPSPositionLLK") or "")
      return float(pos["latitude"]), float(pos["longitude"])
    except (ValueError, KeyError, TypeError):
      return None

  def _baidu(self, path: str, params: list) -> dict:
    """One place API call within the daily budget: the answer, or {"error": ...}."""
    if self.keys is None:
      return {"error": "没有百度 Key（/data/baidu_ak），只能粘贴“纬度,经度”"}
    now = baidu.unix_now()
    if self.daily.left("place", now) <= 0:
      return {"error": "今天的地点查询次数用完了"}
    self.daily.add("place", now)
    try:
      resp = self.fetch(baidu.signed_get(path, params, *self.keys))
    except Exception as e:   # only the type: a message could hold the signed URL
      return {"error": f"查询失败（{type(e).__name__}）"}
    if resp.get("status") != 0:
      return {"error": f"百度返回 {resp.get('status')}"}
    return resp

  def search(self, query: str) -> dict:
    query = query.strip()[:200]
    if not query:
      return {"places": []}
    pasted = parse_coordinates(query)
    if pasted is not None:
      return {"places": [{"name": query, "address": "", "latitude": pasted[0], "longitude": pasted[1]}]}
    if SHARE_URL.search(query):
      place = self.share_place(query)
      return {"places": [place]} if "error" not in place else place
    resp = self._baidu(SUGGESTION_PATH, suggestion_params(query[:64], self.near()))
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


PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>导航目的地</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;margin:0;padding:16px;background:#0b1116;color:#e8eef2;max-width:640px}
h1{font-size:20px;margin:0 0 12px}
#state{background:#16222b;border-radius:10px;padding:12px;margin-bottom:12px;line-height:1.5}
.muted{color:#93a4b0;font-size:14px}
form{display:flex;gap:8px;margin-bottom:12px}
input{flex:1;font-size:17px;padding:10px;border-radius:8px;border:1px solid #33454f;background:#0f1a21;color:#e8eef2}
button{font-size:16px;padding:10px 14px;border-radius:8px;border:0;background:#2f8f5b;color:#fff}
button.secondary{background:#3a4a55}
.place{background:#16222b;border-radius:10px;padding:12px;margin-bottom:8px}
.place b{display:block;margin-bottom:2px}
.place button{margin-top:8px;margin-right:6px}
h2{font-size:16px;margin:16px 0 8px;color:#93a4b0}
#saved{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}
#saved button{background:#25607f}
#msg{color:#f2b36b;margin-bottom:8px}
</style></head><body>
<h1>导航目的地</h1>
<div id="state">读取中…</div>
<div id="saved"></div>
<form id="f"><input id="q" placeholder="搜地点、粘贴百度地图分享或 纬度,经度" autocomplete="off"><button>搜索</button></form>
<div id="msg"></div><div id="list"></div>
<h2>最近去过</h2><div id="recent" class="muted">还没有</div>
<script>
const $=id=>document.getElementById(id);
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
async function api(path,body){
  const r=await fetch(path,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});return r.json()}
function show(s){
  let h='';
  if(!s.navigation)h+='<div class="muted">导航没开：设置 &gt; 开发者 &gt; navigation (alpha)</div>';
  const d=s.destination;
  if(d){h+='<div>当前目的地：<b>'+esc(d.place_name||(d.latitude.toFixed(5)+', '+d.longitude.toFixed(5)))+'</b></div>'
    +'<button class="secondary" onclick="clearDest()">取消导航</button> <button class="secondary" onclick="saveCur()">存为常用地点</button>'}
  else h+='<div>还没有目的地</div>';
  h+='<div class="muted">'+(s.onroad?'车已启动':'车没启动：熄火时目的地会被清掉，comma 重启也会')+'</div>';
  $('state').innerHTML=h;
}
function card(p,act){return '<div class="place"><b>'+esc(p.name||(p.latitude.toFixed(5)+', '+p.longitude.toFixed(5)))+'</b><span class="muted">'
  +esc(p.address||'')+'</span><br>'+act+'</div>'}
function showPlaces(r){
  window.saved=r.saved;window.recent=r.recent;
  $('saved').innerHTML=Object.keys(r.saved).map(k=>'<button onclick="go(this.dataset.k)" data-k="'+esc(k)+'">'+esc(k)+'</button>').join('');
  $('recent').innerHTML=r.recent.length?r.recent.map((p,i)=>card(p,'<button onclick="goRecent('+i+')">去这里</button>')).join(''):'还没有';
}
async function places(){showPlaces(await api('/api/places'))}
async function go(k){const r=await api('/api/go',{label:k});if(r.error){$('msg').textContent=r.error;return}
  show(r);places();$('msg').textContent='已设为目的地：'+k}
async function goRecent(i){show(await api('/api/destination',window.recent[i]));places()}
async function saveCur(){const k=prompt('起个名字，比如 家、公司');if(!k)return;const r=await api('/api/places',{label:k});
  if(r.error){$('msg').textContent=r.error;return}showPlaces(r)}
async function refresh(){try{show(await api('/api/state'))}catch(e){$('state').textContent='连不上车'}}
async function clearDest(){show(await api('/api/clear',{}))}
async function pick(i){const p=window.places[i];show(await api('/api/destination',p));$('list').innerHTML='';
  $('msg').textContent='已设为目的地：'+p.name;places()}
$('f').onsubmit=async e=>{e.preventDefault();$('msg').textContent='搜索中…';$('list').innerHTML='';
  const r=await api('/api/search',{query:$('q').value});
  if(r.error){$('msg').textContent=r.error;return}
  window.places=r.places;$('msg').textContent=r.places.length?'':'没找到';
  $('list').innerHTML=r.places.map((p,i)=>'<div class="place"><b>'+esc(p.name)+'</b><span class="muted">'+esc(p.address)+'</span><br>'
    +'<button onclick="pick('+i+')">设为目的地</button></div>').join('')};
refresh();places();setInterval(refresh,5000);
</script></body></html>
"""


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
        self._json(dest.search(str(body.get("query") or "")))
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
      else:
        self._send(404, b"", "text/plain")

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override; queries stay off the logs
      pass

  return Handler


def main():
  from openpilot.common.params import Params

  dest = Destinations(Params(), baidu.load_keys(), places=Places())
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
