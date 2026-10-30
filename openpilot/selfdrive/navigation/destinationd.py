#!/usr/bin/env python3
"""A small web page to set the navigation destination from a phone, without comma connect or prime.

Open http://<the device's Tailscale name>:8082 on a phone in the same tailnet: search a place (Baidu's place suggestions,
with the key file of road_events/baidu.py) or paste "latitude,longitude" (WGS-84), and it becomes NavDestination, the
param navd routes to. It listens on the Tailscale interface only (the cellular network's CGNAT shares Tailscale's address
range, so the peer address alone would not do) and answers Tailscale peers only; the keys never leave the device.

NavDestination is cleared when the car goes offroad and when manager starts: set it before or after starting the car,
for the drive at hand.
"""
import fcntl
import json
import math
import socket
import struct
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import ip_address, ip_network

from openpilot.selfdrive.road_events import baidu

PORT = 8082
SUGGESTION_PATH = "/place/v2/suggestion"
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


class Destinations:
  """What the page does, apart from HTTP: tests drive it with fake params and a fake fetch."""
  def __init__(self, params, keys=None, fetch=baidu.http_get, count_file: str | None = COUNT_FILE):
    self.params = params
    self.keys = keys
    self.fetch = fetch
    self.daily = baidu.DailyCount(count_file)

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

  def search(self, query: str) -> dict:
    query = query.strip()[:64]
    if not query:
      return {"places": []}
    pasted = parse_coordinates(query)
    if pasted is not None:
      return {"places": [{"name": query, "address": "", "latitude": pasted[0], "longitude": pasted[1]}]}
    if self.keys is None:
      return {"error": "没有百度 Key（/data/baidu_ak），只能粘贴“纬度,经度”"}
    now = baidu.unix_now()
    if self.daily.left("place", now) <= 0:
      return {"error": "今天的地点搜索次数用完了"}
    self.daily.add("place", now)
    try:
      resp = self.fetch(baidu.signed_get(SUGGESTION_PATH, suggestion_params(query, self.near()), *self.keys))
    except Exception as e:   # only the type: a message could hold the signed URL
      return {"error": f"搜索失败（{type(e).__name__}）"}
    if resp.get("status") != 0:
      return {"error": f"百度返回 {resp.get('status')}"}
    return {"places": parse_suggestions(resp)}

  def set_destination(self, body: dict) -> dict:
    try:
      lat, lon = float(body["latitude"]), float(body["longitude"])
    except (KeyError, TypeError, ValueError):
      return {"error": "坐标不对"}
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
      return {"error": "坐标不对"}
    destination = {"latitude": lat, "longitude": lon}
    if isinstance(body.get("name"), str) and body["name"]:
      destination["place_name"] = body["name"][:64]
    self.params.put("NavDestination", destination, block=True)   # the answer reads it back
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
.place button{margin-top:8px}
#msg{color:#f2b36b;margin-bottom:8px}
</style></head><body>
<h1>导航目的地</h1>
<div id="state">读取中…</div>
<form id="f"><input id="q" placeholder="搜地点，或粘贴 纬度,经度" autocomplete="off"><button>搜索</button></form>
<div id="msg"></div><div id="list"></div>
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
    +'<button class="secondary" onclick="clearDest()">取消导航</button>'}
  else h+='<div>还没有目的地</div>';
  h+='<div class="muted">'+(s.onroad?'车已启动':'车没启动：熄火时目的地会被清掉，comma 重启也会')+'</div>';
  $('state').innerHTML=h;
}
async function refresh(){try{show(await api('/api/state'))}catch(e){$('state').textContent='连不上车'}}
async function clearDest(){show(await api('/api/clear',{}))}
async function pick(i){const p=window.places[i];show(await api('/api/destination',p));$('list').innerHTML='';$('msg').textContent='已设为目的地：'+p.name}
$('f').onsubmit=async e=>{e.preventDefault();$('msg').textContent='搜索中…';$('list').innerHTML='';
  const r=await api('/api/search',{query:$('q').value});
  if(r.error){$('msg').textContent=r.error;return}
  window.places=r.places;$('msg').textContent=r.places.length?'':'没找到';
  $('list').innerHTML=r.places.map((p,i)=>'<div class="place"><b>'+esc(p.name)+'</b><span class="muted">'+esc(p.address)+'</span><br>'
    +'<button onclick="pick('+i+')">设为目的地</button></div>').join('')};
refresh();setInterval(refresh,5000);
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
      else:
        self._send(404, b"", "text/plain")

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override; queries stay off the logs
      pass

  return Handler


def main():
  from openpilot.common.params import Params

  dest = Destinations(Params(), baidu.load_keys())
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
