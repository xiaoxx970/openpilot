#!/usr/bin/env python3
"""The car's drives for comma-link, the owner's server (its own repository), which shows them on the phone: the drive list
with each drive's track and timeline, the qcamera video, the logs and videos to download, and the device's state.

  GET /api/drives                          the drives, newest first
  GET /api/drives/<route>                  one drive: its track, timeline (engaged, jams, alerts, speed limits), segments
  GET /api/drives/<route>/qcamera.m3u8     its qcamera video as one HLS playlist
  GET /api/drives/<route>/<n>/<file>       a segment's file as it is on disk (Range works); big ones only while offroad
  POST /api/drives/<route>/keep {keep}     keep the drive from the deleter (or no longer)
  GET /api/device                          onroad, temperatures, fan, cores, storage, branch, the LAN address
  POST /api/link {key_hash, origin, cert, key}   from comma-link: what the LAN side needs (Tailscale only)

A drive is read from its qlogs once, offroad, into a small index file per segment: never while driving. The files go out
paced, the big ones (rlog, the full cameras) not at all while driving. Low priority, on the little cores only: cores 4 and 5
are card/controlsd/selfdrived's and plannerd/radard's. Like destinationd, it listens on the Tailscale interface and answers
Tailscale peers only there.

On the car's Wi-Fi (its hotspot, or a network it joined) it also answers the phone directly, with https on LAN_PORT, once
comma-link has sent the certificate of the car's own name (device.comma.<domain>, which comma-link points at the car's LAN
address): then the page skips the server. There it answers the page's origin only (CORS), with the key's cookie (the
server's cookie, for its domain and so for the car's name too): the car keeps the key's hash, never the key. Never on the
cellular interface.
"""
import hashlib
import hmac
import ssl
import json
import math
import os
import re
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.selfdrive.navigation.destinationd import allowed, interface_ip

PORT = 8083
LAN_PORT = 8443
LAN_INTERFACES = ("wlan0",)   # Wi-Fi: the hotspot or a joined network; never the modem's
LINK_DIR = "/data/drives_link"   # from comma-link: link.json (the key's hash, the page's origin), cert.pem, key.pem
LINK_EVERY = 30.   # s between looks at the LAN address and the link files
COOKIE = "navkey"   # comma-link's key cookie
INDEX_DIR = "/data/drives_index"
INDEX_VERSION = 2   # 2: bookmarks
KEEP_ATTR = "user.keep"   # a kept drive's segments, see the deleter
SEGMENT = re.compile(r"^(?P<route>[0-9A-Za-z-]+?)--(?P<n>\d+)$")
FILES = ("qlog.zst", "qlog", "qcamera.ts", "rlog.zst", "rlog", "fcamera.hevc", "ecamera.hevc", "dcamera.hevc")
ONROAD_FILES = ("qlog.zst", "qlog", "qcamera.ts")   # small enough to send while driving
SPAN = re.compile(r"^(\d{1,5})(?:-(\d{1,5}))?$")   # a segment, or segments a-b: one file, theirs one after another
MAX_SPAN = 120   # segments in one download
MAX_LIST_MARKS = 60   # engaged runs and alerts a drive in the list carries
KINDS = {".ts": "video/mp2t", ".hevc": "application/octet-stream", ".zst": "application/zstd", "": "application/octet-stream"}
CHUNK = 64 * 1024
RATE_ONROAD = 512 * 1024        # B/s while driving: qcamera is about 50 kB/s
RATE_OFFROAD = 4 * 1024 * 1024  # B/s
SETTLED = 90.    # s since a segment's qlog last changed: older, it is no longer being recorded
INDEX_EVERY = 30.   # s between looks for segments to index (offroad)
TRACK_EVERY = 5.    # s between kept GPS points of a segment
MAX_TRACK = 600     # points of a drive's track on the page
THUMB = 24          # points of a drive's track in the list
SEGMENT_SECONDS = 60.
MS_TO_KPH = 3.6


def kph(v: float) -> int:
  return int(round(v * MS_TO_KPH))


def thin(points: list, n: int) -> list:
  if len(points) <= n:
    return points
  step = (len(points) - 1) / (n - 1)
  return [points[round(i * step)] for i in range(n)]


def merge_runs(runs: list[list[float]], gap: float = 2.) -> list[list[float]]:
  out: list[list[float]] = []
  for a, b in sorted(runs):
    if out and a - out[-1][1] <= gap:
      out[-1][1] = max(out[-1][1], b)
    else:
      out.append([a, b])
  return out


class Runs:
  """Runs [start, end] of seconds while a flag is on, from its samples."""
  def __init__(self):
    self.runs: list[list[float]] = []
    self.start: float | None = None
    self.last = 0.

  def sample(self, t: float, on: bool):
    if on and self.start is None:
      self.start = t
    elif not on and self.start is not None:
      self.runs.append([self.start, t])
      self.start = None
    self.last = t

  def done(self) -> list[list[float]]:
    if self.start is not None:
      self.runs.append([self.start, self.last])
      self.start = None
    return [[round(a, 1), round(b, 1)] for a, b in self.runs if b > a]


def summarize(events) -> dict:
  """One segment from its qlog's events: seconds from the segment's start, its wall clock start, track, distance, engaged and
  jam runs, alerts and speed limit changes, temperatures and core loads."""
  t0 = None
  wall = None
  track: list[list[float]] = []
  distance = 0.
  last_speed: tuple[float, float] | None = None
  engaged, jams = Runs(), Runs()
  events_out: list[dict] = []
  alert = ""
  limit = 0
  temps = {"cpu": 0., "memory": 0.}
  cores: list[float] = []
  core_samples = 0
  t = 0.
  for e in events:
    try:
      which = e.which()
    except Exception:  # a message this cereal does not know
      continue
    mono = e.logMonoTime * 1e-9
    if t0 is None:
      t0 = mono
    t = mono - t0
    if which in ("gpsLocationExternal", "gpsLocation"):
      g = getattr(e, which)
      if not g.hasFix or (g.latitude == 0. and g.longitude == 0.):
        continue
      if wall is None and g.unixTimestampMillis > 0:
        wall = g.unixTimestampMillis * 1e-3 - t
      if not track or t - track[-1][0] >= TRACK_EVERY:
        track.append([round(t, 1), round(g.latitude, 6), round(g.longitude, 6)])
    elif which == "carState":
      v = e.carState.vEgo
      if last_speed is not None and 0. < t - last_speed[0] < 2.:
        distance += (v + last_speed[1]) / 2 * (t - last_speed[0])
      last_speed = (t, v)
    elif which == "selfdriveState":
      s = e.selfdriveState
      engaged.sample(t, s.enabled)
      shown = s.alertType if str(s.alertStatus) != "normal" else ""
      if shown and shown != alert:
        events_out.append({"t": round(t, 1), "kind": "alert", "text": s.alertText1 or shown.split("/")[0],
                           "type": shown.split("/")[0], "status": str(s.alertStatus)})
      alert = shown
    elif which == "userBookmark":   # the bookmark button: openpilot keeps the segment too
      events_out.append({"t": round(t, 1), "kind": "bookmark"})
    elif which == "roadEventsIC":
      jams.sample(t, e.roadEventsIC.jamFollow)
    elif which == "longitudinalPlanSP":
      r = e.longitudinalPlanSP.speedLimit.resolver
      now = kph(r.speedLimit) if r.speedLimitValid and r.speedLimit > 0 else 0
      if now and abs(now - limit) >= 5:
        events_out.append({"t": round(t, 1), "kind": "limit", "value": now, "from": limit or None})
        limit = now
    elif which == "deviceState":
      d = e.deviceState
      temps["cpu"] = max(temps["cpu"], max(d.cpuTempC, default=0.))
      temps["memory"] = max(temps["memory"], d.memoryTempC)
      usage = list(d.cpuUsagePercent)
      if usage:
        if len(cores) != len(usage):
          cores, core_samples = [0.] * len(usage), 0
        cores = [c + u for c, u in zip(cores, usage, strict=True)]
        core_samples += 1
  return {"version": INDEX_VERSION, "duration": round(t, 1), "wall": round(wall, 1) if wall is not None else None, "track": track,
          "distance": round(distance), "engaged": engaged.done(), "jams": jams.done(), "events": events_out,
          "temps": {k: round(v, 1) for k, v in temps.items()},
          "cores": [round(c / core_samples) for c in cores] if core_samples else []}


def read_qlog(path: str):
  """The events of a qlog (zstd or not); a cut-off one gives what it has."""
  import zstandard

  from openpilot.cereal import log as capnp_log

  with open(path, "rb") as f:
    data = f.read()
  if path.endswith(".zst"):
    try:
      data = zstandard.ZstdDecompressor().decompressobj().decompress(data)   # a cut-off file gives what it has
    except zstandard.ZstdError:
      return
  try:
    yield from capnp_log.Event.read_multiple_bytes(data)
  except Exception:  # capnp: the last message cut off
    return


def qlog_of(seg_dir: str) -> str | None:
  for name in ("qlog.zst", "qlog"):
    p = os.path.join(seg_dir, name)
    if os.path.exists(p):
      return p
  return None


def kept(seg_dir: str) -> bool:
  try:
    return os.getxattr(seg_dir, KEEP_ATTR) == b"1"
  except OSError:
    return False


def bookmarked(seg_dir: str) -> bool:
  """loggerd marks a segment it keeps for a bookmark."""
  try:
    return os.getxattr(seg_dir, "user.preserve") == b"1"
  except OSError:
    return False


def coarse_runs(runs: list[list[float]], duration: float) -> list[list[int]]:
  """Engaged runs for the list's small timeline: gaps under 1% of the drive (at least 5 s) closed, whole seconds."""
  gap = max(5., duration / 100)
  out: list[list[float]] = []
  for a, b in runs:
    if out and a - out[-1][1] < gap:
      out[-1][1] = max(out[-1][1], b)
    else:
      out.append([a, b])
  return [[round(a), round(b)] for a, b in out[:MAX_LIST_MARKS]]


class Drives:
  """The drives on disk (log_root) and their index (index_dir)."""
  def __init__(self, log_root: str, index_dir: str = INDEX_DIR, clock=time.time, reader=read_qlog):  # noqa: TID251  # vs file mtimes
    self.log_root, self.index_dir, self.clock, self.reader = log_root, index_dir, clock, reader
    self.lock = threading.Lock()

  def segments(self) -> dict[str, list[tuple[int, str]]]:
    """route -> [(n, segment directory)], by segment number."""
    routes: dict[str, list[tuple[int, str]]] = {}
    try:
      names = os.listdir(self.log_root)
    except OSError:
      return routes
    for name in names:
      m = SEGMENT.match(name)
      if m and os.path.isdir(os.path.join(self.log_root, name)):
        routes.setdefault(m["route"], []).append((int(m["n"]), os.path.join(self.log_root, name)))
    for segs in routes.values():
      segs.sort()
    return routes

  def _index_path(self, seg_dir: str) -> str:
    return os.path.join(self.index_dir, os.path.basename(seg_dir) + ".json")

  def index_of(self, seg_dir: str) -> dict | None:
    try:
      with open(self._index_path(seg_dir), encoding="utf-8") as f:
        s = json.load(f)
    except (OSError, ValueError):
      return None
    return s if s.get("version") == INDEX_VERSION else None

  def stale(self, seg_dir: str) -> bool:
    """A recorded segment without an index of its qlog as it is now."""
    q = qlog_of(seg_dir)
    if q is None:
      return False
    try:
      changed = os.path.getmtime(q)
    except OSError:
      return False
    if self.clock() - changed <= SETTLED:
      return False
    try:
      return os.path.getmtime(self._index_path(seg_dir)) < changed
    except OSError:
      return True

  def index(self, seg_dir: str):
    q = qlog_of(seg_dir)
    if q is None:
      return
    s = summarize(self.reader(q))
    os.makedirs(self.index_dir, exist_ok=True)
    path = self._index_path(seg_dir)
    with open(path + ".tmp", "w", encoding="utf-8") as f:
      json.dump(s, f, separators=(",", ":"))
    os.replace(path + ".tmp", path)

  def index_all(self, onroad=lambda: False) -> int:
    """Index what is not yet, newest drive first; stops when the car starts. The number indexed."""
    done = 0
    routes = self.segments()
    for route in sorted(routes, reverse=True):
      for _, seg_dir in routes[route]:
        if onroad():
          return done
        if self.stale(seg_dir):
          try:
            self.index(seg_dir)
            done += 1
          except OSError:
            pass
    self.prune(routes)
    return done

  def prune(self, routes: dict[str, list[tuple[int, str]]]):
    """Index files of segments the deleter removed."""
    have = {os.path.basename(d) + ".json" for segs in routes.values() for _, d in segs}
    try:
      names = os.listdir(self.index_dir)
    except OSError:
      return
    for name in names:
      if name.endswith(".json") and name not in have:
        try:
          os.remove(os.path.join(self.index_dir, name))
        except OSError:
          pass

  def _route(self, route: str, segs: list[tuple[int, str]], detail: bool) -> dict:
    track: list[list[float]] = []
    engaged: list[list[float]] = []
    jams: list[list[float]] = []
    events: list[dict] = []
    distance = 0.
    wall = None
    indexed = 0
    temps = {"cpu": 0., "memory": 0.}
    loads: list[list[int]] = []
    size = 0
    files = []
    last_end = 0.
    for n, seg_dir in segs:
      offset = n * SEGMENT_SECONDS
      seg_files = {}
      for name in FILES:
        try:
          seg_files[name] = os.path.getsize(os.path.join(seg_dir, name))
        except OSError:
          pass
      size += sum(seg_files.values())
      files.append({"n": n, "files": seg_files})
      s = self.index_of(seg_dir)
      if s is None:
        last_end = max(last_end, offset)
        continue
      indexed += 1
      if wall is None and s["wall"] is not None:
        wall = s["wall"] - offset
      track += [[round(t + offset, 1), la, lo] for t, la, lo in s["track"]]
      engaged += [[a + offset, b + offset] for a, b in s["engaged"]]
      jams += [[a + offset, b + offset] for a, b in s["jams"]]
      events += [{**e, "t": round(e["t"] + offset, 1)} for e in s["events"]]
      distance += s["distance"]
      last_end = max(last_end, offset + s["duration"])
      for k in temps:
        temps[k] = max(temps[k], s["temps"].get(k, 0.))
      if s["cores"]:
        loads.append(s["cores"])
    if wall is None:   # no GPS time: the first segment's directory
      try:
        wall = os.path.getctime(segs[0][1])
      except OSError:
        wall = None
    duration = max(last_end, len(segs) * SEGMENT_SECONDS if not indexed else 0.)
    engaged = merge_runs(engaged)
    on = sum(b - a for a, b in engaged)
    out = {"id": route, "start": round(wall) if wall else None, "duration": round(duration), "distance": round(distance),
           "engaged": round(on / duration, 3) if duration and indexed else None, "segments": len(segs),
           "indexed": indexed, "size": size, "kept": all(kept(d) for _, d in segs),
           "alerts": sum(1 for e in events if e["kind"] == "alert"),
           "bookmarks": sum(1 for e in events if e["kind"] == "bookmark") or sum(1 for _, d in segs if bookmarked(d)),
           "jam": round(sum(b - a for a, b in merge_runs(jams))),
           "thumb": [[la, lo] for _, la, lo in thin(track, THUMB)],
           "runs": coarse_runs(engaged, duration), "alert_t": [round(e["t"]) for e in events if e["kind"] == "alert"][:MAX_LIST_MARKS],
           "bookmark_t": [round(e["t"]) for e in events if e["kind"] == "bookmark"][:MAX_LIST_MARKS]}
    if detail:
      out.update({"track": thin(track, MAX_TRACK), "timeline": {"engaged": engaged, "jams": merge_runs(jams), "events": events},
                  "files": files, "temps": temps, "cores": [round(sum(c) / len(c)) for c in zip(*[x for x in loads if len(x) == len(loads[0])], strict=True)]
                  if loads else []})
    return out

  def list(self) -> dict:
    routes = self.segments()
    return {"drives": [self._route(r, routes[r], False) for r in sorted(routes, reverse=True)]}

  def drive(self, route: str) -> dict | None:
    segs = self.segments().get(route)
    return self._route(route, segs, True) if segs else None

  def playlist(self, route: str) -> bytes | None:
    segs = self.segments().get(route)
    if not segs:
      return None
    lines = ["#EXTM3U", "#EXT-X-VERSION:3", f"#EXT-X-TARGETDURATION:{math.ceil(SEGMENT_SECONDS) + 1}", "#EXT-X-PLAYLIST-TYPE:VOD"]
    first = True
    for n, seg_dir in segs:
      if not os.path.exists(os.path.join(seg_dir, "qcamera.ts")):
        continue
      if not first:
        lines.append("#EXT-X-DISCONTINUITY")   # each segment's video starts its own clock
      lines += [f"#EXTINF:{SEGMENT_SECONDS:.1f},", f"{n}/qcamera.ts"]   # relative: the server puts its own prefix in front
      first = False
    lines.append("#EXT-X-ENDLIST")
    return ("\n".join(lines) + "\n").encode()

  def file(self, route: str, n: int, name: str) -> str | None:
    paths = self.files(route, n, n, name)
    return paths[0] if paths else None

  def files(self, route: str, a: int, b: int, name: str) -> tuple[str, ...]:
    """The file of segments a..b that have it, in order. qlog, rlog, qcamera and the hevc streams play on when put one after
    another (zstd frames, log events, MPEG-TS packets, Annex B NAL units), so a stretch of a drive downloads as one file."""
    if name not in FILES or b < a or b - a >= MAX_SPAN:
      return ()
    out = []
    for m, seg_dir in self.segments().get(route, []):
      p = os.path.join(seg_dir, name)
      if a <= m <= b and os.path.isfile(p):
        out.append(p)
    return tuple(out)

  def keep(self, route: str, on: bool) -> dict | None:
    segs = self.segments().get(route)
    if not segs:
      return None
    for _, seg_dir in segs:
      try:
        if on:
          os.setxattr(seg_dir, KEEP_ATTR, b"1")
        elif kept(seg_dir):
          os.removexattr(seg_dir, KEEP_ATTR)
      except OSError:
        return {"error": "没改成，车上的文件系统不让写"}
    return {"id": route, "kept": on}


class Device:
  """The device's state, for the page: deviceState as last sent, the params, the disk."""
  def __init__(self, params, sm=None, data_dir: str = "/data"):
    self.params, self.sm, self.data_dir = params, sm, data_dir
    self.lan: dict | None = None   # the LAN side, see LanServer
    self.lock = threading.Lock()

  def onroad(self) -> bool:
    return not self.params.get_bool("IsOffroad")   # (there is no IsOnroad) unset at manager start: onroad till it says

  def state(self, drives: Drives | None = None) -> dict:
    out = {"onroad": self.onroad(), "branch": self.params.get("GitBranch"), "commit": (self.params.get("GitCommit") or "")[:9]}
    if self.lan is not None:
      out["lan"] = dict(self.lan)
    if self.sm is not None:
      with self.lock:
        self.sm.update(0)
        fresh = self.sm.updated['deviceState'] or time.monotonic() - self.sm.recv_time['deviceState'] < 5.
        d = self.sm['deviceState']
        if fresh:
          out.update({"temps": {"cpu": round(max(d.cpuTempC, default=0.), 1), "memory": round(d.memoryTempC, 1),
                                "max": round(d.maxTempC, 1)},
                      "fan": d.fanSpeedPercentDesired, "cores": list(d.cpuUsagePercent), "thermal": str(d.thermalStatus),
                      "network": str(d.networkType)})
    try:
      st = os.statvfs(self.data_dir)
      out["disk"] = {"total": st.f_blocks * st.f_frsize, "free": st.f_bavail * st.f_frsize}
    except OSError:
      pass
    if drives is not None:
      listed = drives.list()["drives"]
      out["drives"] = {"size": sum(d["size"] for d in listed), "kept": sum(d["size"] for d in listed if d["kept"]),
                       "count": len(listed)}
      last = next((d for d in listed if d["indexed"]), None)
      if last is not None:
        detail = drives.drive(last["id"]) or {}
        out["last_drive"] = {"id": last["id"], "start": last["start"], "temps": detail.get("temps"), "cores": detail.get("cores")}
    return out


def parse_range(header: str | None, size: int) -> tuple[int, int] | None:
  """bytes=a-b, a-, -n -> (first, last) within size; None for the whole file or a range that does not fit."""
  if not header or not header.startswith("bytes=") or "," in header:
    return None
  a, _, b = header[6:].strip().partition("-")
  try:
    if a == "":
      first, last = max(0, size - int(b)), size - 1
    else:
      first, last = int(a), min(size - 1, int(b)) if b else size - 1
  except ValueError:
    return None
  return (first, last) if 0 <= first <= last < size else None


class Link:
  """What comma-link sent for the LAN side: the key's hash, the page's origin, the car name's certificate and key."""
  def __init__(self, path: str = LINK_DIR):
    self.path = path
    self.lock = threading.Lock()

  def _read(self, name: str) -> bytes | None:
    try:
      with open(os.path.join(self.path, name), "rb") as f:
        return f.read()
    except OSError:
      return None

  def config(self) -> dict:
    try:
      c = json.loads(self._read("link.json") or b"{}")
    except ValueError:
      return {}
    return c if isinstance(c, dict) else {}

  def ready(self) -> bool:
    c = self.config()
    return bool(c.get("key_hash") and c.get("origin")) and self._read("cert.pem") is not None and self._read("key.pem") is not None

  def digest(self) -> str | None:
    """What the car has, for comma-link to tell whether to send it again."""
    parts = [self._read(n) for n in ("link.json", "cert.pem", "key.pem")]
    return hashlib.sha256(b"\0".join(parts)).hexdigest()[:16] if all(p is not None for p in parts) else None  # type: ignore[misc]

  def save(self, body: dict) -> dict:
    key_hash, origin = str(body.get("key_hash") or ""), str(body.get("origin") or "")
    cert, key = str(body.get("cert") or ""), str(body.get("key") or "")
    if not re.fullmatch(r"[0-9a-f]{64}", key_hash) or not re.fullmatch(r"https://[0-9A-Za-z.-]+(:\d+)?", origin):
      return {"error": "key_hash or origin"}
    if "-----BEGIN CERTIFICATE-----" not in cert or "PRIVATE KEY-----" not in key:
      return {"error": "cert or key"}
    with self.lock:
      os.makedirs(self.path, mode=0o700, exist_ok=True)
      for name, data in (("cert.pem", cert.encode()), ("key.pem", key.encode()),
                         ("link.json", json.dumps({"key_hash": key_hash, "origin": origin}).encode())):
        tmp = os.path.join(self.path, name + ".tmp")
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "wb") as f:
          f.write(data)
        os.replace(tmp, os.path.join(self.path, name))
    return {"link": self.digest()}

  def key_ok(self, headers) -> bool:
    want = self.config().get("key_hash")
    if not want:
      return False
    given = headers.get("X-Key")
    for part in (headers.get("Cookie") or "").split(";"):
      name, _, value = part.strip().partition("=")
      if name == COOKIE and given is None:
        given = value
    return given is not None and hmac.compare_digest(hashlib.sha256(given.encode()).hexdigest(), want)

  def context(self) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(os.path.join(self.path, "cert.pem"), os.path.join(self.path, "key.pem"))
    return ctx


def handler(drives: Drives, device: Device, link: Link | None = None, lan: bool = False):
  """lan: the phone on the car's Wi-Fi, with the key's cookie, from the page's origin; else Tailscale peers."""
  class Handler(BaseHTTPRequestHandler):
    timeout = 30.

    def end_headers(self):
      origin = self.headers.get("Origin")
      if lan and link is not None and origin and origin == link.config().get("origin"):
        self.send_header("Access-Control-Allow-Origin", origin)
        self.send_header("Access-Control-Allow-Credentials", "true")
        self.send_header("Vary", "Origin")
      super().end_headers()

    def _send(self, code: int, body: bytes, kind: str):
      self.send_response(code)
      self.send_header("Content-Type", kind)
      self.send_header("Content-Length", str(len(body)))
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      self.wfile.write(body)

    def _json(self, obj: dict | None, code: int = 200):
      if obj is None:
        self._send(404, b"", "text/plain")
      else:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def _allowed(self) -> bool:
      if (link is not None and link.key_ok(self.headers)) if lan else allowed(self.client_address[0]):
        return True
      self._send(403, b"", "text/plain")
      return False

    def _file(self, paths: tuple[str, ...], name: str, download: str):
      """The files one after another, as one: Range over the whole."""
      onroad = device.onroad()
      if onroad and name not in ONROAD_FILES:
        self._json({"error": "车在行驶，大文件停车后才能下"}, 409)
        return
      sizes = [os.path.getsize(p) for p in paths]
      size = sum(sizes)
      span = parse_range(self.headers.get("Range"), size)
      first, last = span if span is not None else (0, size - 1)
      self.send_response(206 if span is not None else 200)
      self.send_header("Content-Type", KINDS.get(os.path.splitext(name)[1], KINDS[""]))
      self.send_header("Content-Length", str(max(0, last - first + 1)))
      self.send_header("Accept-Ranges", "bytes")
      self.send_header("Content-Disposition", f'attachment; filename="{download}"')   # the name a save gives it, cross-origin too
      if span is not None:
        self.send_header("Content-Range", f"bytes {first}-{last}/{size}")
      self.end_headers()
      rate = RATE_ONROAD if onroad else RATE_OFFROAD
      at = 0   # where each file starts in the whole
      for p, n in zip(paths, sizes, strict=True):
        lo, hi = max(first, at), min(last, at + n - 1)
        at += n
        if lo > hi:
          continue
        left = hi - lo + 1
        with open(p, "rb") as f:
          f.seek(lo - (at - n))
          while left > 0:
            start = time.monotonic()
            chunk = f.read(min(CHUNK, left))
            if not chunk:
              break
            self.wfile.write(chunk)
            left -= len(chunk)
            time.sleep(max(0., len(chunk) / rate - (time.monotonic() - start)))   # paced: the disk and the modem stay free

    def do_GET(self):
      if not self._allowed():
        return
      parts = [urllib.parse.unquote(p) for p in urllib.parse.urlsplit(self.path).path.strip("/").split("/")]
      if parts == ["api", "ping"]:
        self._json({"ok": True, "lan": lan})
      elif parts == ["api", "device"]:
        self._json(device.state(drives))
      elif parts == ["api", "drives"]:
        self._json(drives.list())
      elif len(parts) == 3 and parts[:2] == ["api", "drives"]:
        self._json(drives.drive(parts[2]))
      elif len(parts) == 4 and parts[:2] == ["api", "drives"] and parts[3] == "qcamera.m3u8":
        body = drives.playlist(parts[2])
        if body is None:
          self._send(404, b"", "text/plain")
        else:
          self._send(200, body, "application/vnd.apple.mpegurl")
      elif len(parts) == 5 and parts[:2] == ["api", "drives"] and (m := SPAN.match(parts[3])):
        a = int(m.group(1))
        b = int(m.group(2)) if m.group(2) else a
        paths = drives.files(parts[2], a, b, parts[4])
        if not paths:
          self._send(404, b"", "text/plain")
        else:
          try:
            self._file(paths, parts[4], f"{parts[2]}--{parts[3]}--{parts[4]}")
          except (BrokenPipeError, ConnectionResetError):   # the phone stopped watching
            pass
      else:
        self._send(404, b"", "text/plain")

    def do_POST(self):
      if not self._allowed():
        return
      parts = urllib.parse.urlsplit(self.path).path.strip("/").split("/")
      try:
        body = json.loads(self.rfile.read(min(max(0, int(self.headers.get("Content-Length") or 0)), 65536)) or b"{}")
      except ValueError:
        body = None
      if len(parts) == 4 and parts[:2] == ["api", "drives"] and parts[3] == "keep" and isinstance(body, dict):
        self._json(drives.keep(urllib.parse.unquote(parts[2]), bool(body.get("keep", True))))
      elif parts == ["api", "link"] and not lan and link is not None and isinstance(body, dict):
        self._json(link.save(body))
      else:
        self._send(404, b"", "text/plain")

    def do_OPTIONS(self):   # the browser's preflight, without the cookie
      if not lan:
        self._send(404, b"", "text/plain")
        return
      self.send_response(204)
      self.send_header("Access-Control-Allow-Methods", "GET, POST")
      self.send_header("Access-Control-Allow-Headers", "Content-Type, Range")
      self.send_header("Access-Control-Allow-Private-Network", "true")
      self.send_header("Access-Control-Max-Age", "600")
      self.send_header("Content-Length", "0")
      self.end_headers()

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override; places stay off the logs
      pass

  return Handler


def lan_address() -> str | None:
  return next((ip for ip in map(interface_ip, LAN_INTERFACES) if ip is not None), None)


class LanServer:
  """The https side on the car's Wi-Fi: (re)started when its address or the link files change, stopped without either."""
  def __init__(self, drives: Drives, device: Device, link: Link, address=lan_address, port: int = LAN_PORT):
    self.drives, self.device, self.link, self.address, self.port = drives, device, link, address, port
    self.key: tuple | None = None
    self.server: ThreadingHTTPServer | None = None

  def _stop(self):
    if self.server is not None:
      self.server.shutdown()
      self.server.server_close()
    self.server, self.key = None, None

  def step(self):
    ip = self.address()
    want = (ip, self.link.digest()) if ip is not None and self.link.ready() else None
    self.device.lan = {"ip": ip, "port": self.port, "serving": False, "link": self.link.digest()}
    if want != self.key:
      self._stop()
      if want is not None:
        try:
          server = ThreadingHTTPServer((ip, self.port), handler(self.drives, self.device, self.link, lan=True))
          server.socket = self.link.context().wrap_socket(server.socket, server_side=True, do_handshake_on_connect=False)
        except (OSError, ssl.SSLError):   # the port still held, or a bad certificate: the next look
          return
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.server, self.key = server, want
    self.device.lan["serving"] = self.server is not None

  def run(self, stop: threading.Event):
    while True:
      try:
        self.step()
      except Exception:  # the LAN side is extra: the Tailscale side goes on without it
        pass
      if stop.wait(LINK_EVERY):
        self._stop()
        return


def indexer(drives: Drives, device: Device, stop: threading.Event):
  while not stop.wait(INDEX_EVERY):
    try:
      if not device.onroad():
        with drives.lock:
          drives.index_all(onroad=device.onroad)
    except Exception:  # a bad qlog or params: the next look
      pass


def main():
  import openpilot.cereal.messaging as messaging
  from openpilot.common.hardware.hw import Paths
  from openpilot.common.params import Params
  from openpilot.common.realtime import set_core_affinity

  set_core_affinity([0, 1, 2, 3])
  os.nice(15)
  drives = Drives(Paths.log_root())
  device = Device(Params(), messaging.SubMaster(['deviceState']))
  link = Link()
  stop = threading.Event()
  threading.Thread(target=indexer, args=(drives, device, stop), daemon=True).start()
  threading.Thread(target=LanServer(drives, device, link).run, args=(stop,), daemon=True).start()
  while True:
    address = interface_ip()
    if address is not None:
      try:
        server = ThreadingHTTPServer((address, PORT), handler(drives, device, link))
        break
      except OSError:   # the port is still held by a previous run
        pass
    time.sleep(10)   # Tailscale not up yet
  server.daemon_threads = True
  server.serve_forever()


if __name__ == "__main__":
  main()
