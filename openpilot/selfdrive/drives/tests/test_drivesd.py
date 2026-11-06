import datetime
import hashlib
import json
import os
import socket
import ssl
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS
from unittest.mock import patch

from openpilot.selfdrive.drives import drivesd


def ev(t, which, **fields):
  return NS(logMonoTime=int((100. + t) * 1e9), which=lambda: which, **{which: NS(**fields)})


def gps(t, lat, lon, wall=None):
  return ev(t, "gpsLocationExternal", hasFix=True, latitude=lat, longitude=lon, unixTimestampMillis=int((wall or 0) * 1e3))


def sds(t, enabled, alert="", status="normal", text=""):
  return ev(t, "selfdriveState", enabled=enabled, alertType=alert, alertStatus=status, alertText1=text)


def limit(t, kph):
  return NS(logMonoTime=int((100. + t) * 1e9), which=lambda: "longitudinalPlanSP",
            longitudinalPlanSP=NS(speedLimit=NS(resolver=NS(speedLimit=kph / 3.6, speedLimitValid=kph > 0))))


def segment_events():
  out = [gps(0., 32.1, 118.9, wall=1_762_300_000.)]
  out += [ev(t / 10, "carState", vEgo=10.) for t in range(600)]   # 10 m/s for 60 s
  out += [sds(0., False), sds(10., True), sds(30., True, "commIssue/softDisable", "userPrompt", "通讯问题"), sds(31., True),
          sds(50., False)]
  out += [ev(20., "roadEventsIC", jamFollow=True), ev(40., "roadEventsIC", jamFollow=False)]
  out.append(ev(33., "userBookmark"))
  out += [limit(5., 60), limit(6., 62), limit(45., 40)]
  out += [gps(t, 32.1 + t * 1e-5, 118.9) for t in range(1, 60)]
  out += [ev(15., "deviceState", cpuTempC=[60., 70.], memoryTempC=50., cpuUsagePercent=[10, 90]),
          ev(45., "deviceState", cpuTempC=[80.], memoryTempC=55., cpuUsagePercent=[30, 70])]
  out.append(ev(60., "carState", vEgo=10.))
  return sorted(out, key=lambda e: e.logMonoTime)


class TestSummarize(unittest.TestCase):
  def test_segment(self):
    s = drivesd.summarize(segment_events())
    self.assertEqual(s["wall"], 1_762_300_000.)
    self.assertAlmostEqual(s["distance"], 600, delta=2)
    self.assertEqual(s["engaged"], [[10., 50.]])
    self.assertEqual(s["jams"], [[20., 40.]])
    alerts = [e for e in s["events"] if e["kind"] == "alert"]
    self.assertEqual([(e["t"], e["type"], e["text"]) for e in alerts], [(30., "commIssue", "通讯问题")])
    self.assertEqual([(e["t"], e["value"]) for e in s["events"] if e["kind"] == "limit"], [(5., 60), (45., 40)])   # 60 -> 62 is noise
    self.assertEqual(s["temps"], {"cpu": 80., "memory": 55.})
    self.assertEqual(s["cores"], [20, 80])
    self.assertLessEqual(len(s["track"]), 13)   # one point per 5 s

  def test_init_data_of_the_route(self):
    # a later segment's qlog opens with the route's initData, a segment (or more) before its own events
    s = drivesd.summarize([NS(logMonoTime=int(40e9), which=lambda: "initData")] + segment_events())
    self.assertEqual(s["engaged"], [[10., 50.]])
    self.assertAlmostEqual(s["duration"], 60., delta=0.5)

  def test_runs_open_at_the_end(self):
    s = drivesd.summarize([sds(0., True), sds(20., True)])
    self.assertEqual(s["engaged"], [[0., 20.]])

  def test_no_gps(self):
    s = drivesd.summarize([sds(0., False)])
    self.assertIsNone(s["wall"])
    self.assertEqual(s["track"], [])


class Disk:
  """A log root with drives, an index dir and a clock past SETTLED."""
  def __init__(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.root = os.path.join(self.tmp.name, "realdata")
    self.index = os.path.join(self.tmp.name, "index")
    os.makedirs(self.root)
    self.read = []
    self.drives = drivesd.Drives(self.root, self.index, clock=lambda: 4e9, reader=self.reader)

  def reader(self, path):
    self.read.append(path)
    return iter(segment_events())

  def segment(self, name, files=("qlog.zst", "qcamera.ts", "rlog.zst")):
    d = os.path.join(self.root, name)
    os.makedirs(d)
    for f in files:
      with open(os.path.join(d, f), "wb") as fh:
        fh.write(f.encode() * 10)
    return d


class TestDrives(unittest.TestCase):
  def setUp(self):
    self.disk = Disk()
    self.addCleanup(self.disk.tmp.cleanup)
    for n in range(3):
      self.disk.segment(f"00000101--abcdef0123--{n}")
    self.disk.segment("00000102--0123abcdef--0")
    os.makedirs(os.path.join(self.disk.root, "boot"))

  def test_list_before_indexing(self):
    drives = self.disk.drives.list()["drives"]
    self.assertEqual([d["id"] for d in drives], ["00000102--0123abcdef", "00000101--abcdef0123"])
    self.assertEqual(drives[1]["segments"], 3)
    self.assertEqual(drives[1]["indexed"], 0)
    self.assertIsNone(drives[1]["engaged"])

  def test_index_and_drive(self):
    self.assertEqual(self.disk.drives.index_all(), 4)
    self.assertEqual(self.disk.drives.index_all(), 0)   # indexed once
    d = self.disk.drives.drive("00000101--abcdef0123")
    self.assertEqual(d["indexed"], 3)
    self.assertEqual(d["start"], 1_762_300_000)
    self.assertAlmostEqual(d["distance"], 1800, delta=6)
    self.assertEqual(d["timeline"]["engaged"][:2], [[10., 50.], [70., 110.]])   # each segment at n * 60 s
    self.assertEqual(d["alerts"], 3)
    self.assertEqual(d["files"][0]["files"]["qcamera.ts"], len("qcamera.ts") * 10)
    self.assertEqual(d["cores"], [20, 80])
    self.assertAlmostEqual(d["engaged"], 120 / d["duration"], places=3)
    listed = self.disk.drives.list()["drives"][1]
    self.assertEqual(listed["runs"], [[10, 50], [70, 110], [130, 170]])
    self.assertEqual(listed["alert_t"], [30, 90, 150])
    self.assertEqual((listed["bookmarks"], listed["bookmark_t"]), (3, [33, 93, 153]))

  def test_index_of_an_older_version_redone(self):
    self.disk.drives.index_all()
    path = os.path.join(self.disk.index, "00000101--abcdef0123--0.json")
    with open(path, encoding="utf-8") as f:
      old = {**json.load(f), "version": drivesd.INDEX_VERSION - 1}
    with open(path, "w", encoding="utf-8") as f:   # newer than its qlog, as on the car
      json.dump(old, f)
    self.assertEqual(self.disk.drives.index_all(), 1)
    self.assertIsNotNone(self.disk.drives.index_of(os.path.join(self.disk.root, "00000101--abcdef0123--0")))

  def test_stops_when_the_car_starts(self):
    self.assertEqual(self.disk.drives.index_all(onroad=lambda: True), 0)

  def test_recording_segment_waits(self):
    disk = Disk()
    self.addCleanup(disk.tmp.cleanup)
    disk.drives.clock = lambda: os.path.getmtime(os.path.join(seg, "qlog.zst")) + 10
    seg = disk.segment("00000103--aaaaaaaaaa--0")
    self.assertEqual(disk.drives.index_all(), 0)

  def test_prune(self):
    self.disk.drives.index_all()
    import shutil
    shutil.rmtree(os.path.join(self.disk.root, "00000102--0123abcdef--0"))
    self.disk.drives.index_all()
    self.assertEqual(sorted(os.listdir(self.disk.index)), [f"00000101--abcdef0123--{n}.json" for n in range(3)])

  def test_playlist(self):
    os.remove(os.path.join(self.disk.root, "00000101--abcdef0123--1", "qcamera.ts"))
    body = self.disk.drives.playlist("00000101--abcdef0123").decode()
    self.assertIn("0/qcamera.ts", body)
    self.assertNotIn("1/qcamera.ts", body)
    self.assertIn("#EXT-X-DISCONTINUITY\n#EXTINF:60.0,\n2/qcamera.ts", body)
    self.assertTrue(body.rstrip().endswith("#EXT-X-ENDLIST"))
    self.assertIsNone(self.disk.drives.playlist("nope"))

  def test_file(self):
    self.assertTrue(self.disk.drives.file("00000101--abcdef0123", 2, "rlog.zst").endswith("--2/rlog.zst"))
    self.assertIsNone(self.disk.drives.file("00000101--abcdef0123", 2, "../../etc/passwd"))
    self.assertIsNone(self.disk.drives.file("00000101--abcdef0123", 9, "rlog.zst"))
    self.assertIsNone(self.disk.drives.file("00000101--abcdef0123", 0, "fcamera.hevc"))

  def test_keep(self):
    try:
      answer = self.disk.drives.keep("00000101--abcdef0123", True)
    except OSError:
      self.skipTest("no user xattrs here")
    if "error" in answer:
      self.skipTest("no user xattrs here")
    self.assertTrue(self.disk.drives.list()["drives"][1]["kept"])
    self.assertFalse(self.disk.drives.list()["drives"][0]["kept"])
    self.disk.drives.keep("00000101--abcdef0123", False)
    self.assertFalse(self.disk.drives.list()["drives"][1]["kept"])


class TestCoarseRuns(unittest.TestCase):
  def test_runs(self):
    self.assertEqual(drivesd.coarse_runs([[0., 10.], [12., 20.], [40.4, 50.6]], 100.), [[0, 20], [40, 51]])
    self.assertEqual(drivesd.coarse_runs([[0., 10.], [40., 50.]], 6000.), [[0, 50]])   # 30 s of 100 min: closed
    self.assertEqual(drivesd.coarse_runs([], 100.), [])


class TestRange(unittest.TestCase):
  def test_ranges(self):
    self.assertEqual(drivesd.parse_range("bytes=0-99", 1000), (0, 99))
    self.assertEqual(drivesd.parse_range("bytes=900-", 1000), (900, 999))
    self.assertEqual(drivesd.parse_range("bytes=-100", 1000), (900, 999))
    self.assertEqual(drivesd.parse_range("bytes=0-5000", 1000), (0, 999))
    self.assertIsNone(drivesd.parse_range("bytes=2000-", 1000))
    self.assertIsNone(drivesd.parse_range("bytes=0-1,5-6", 1000))
    self.assertIsNone(drivesd.parse_range(None, 1000))


class FakeParams:
  """Params with the keys the car has: an unknown one raises, as Params does."""
  KEYS = ("IsOffroad", "GitBranch", "GitCommit")

  def __init__(self, values):
    self.values = values

  def get(self, key):
    if key not in self.KEYS:
      raise KeyError(key)
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))


class TestServer(unittest.TestCase):
  def setUp(self):
    self.disk = Disk()
    self.addCleanup(self.disk.tmp.cleanup)
    self.disk.segment("00000101--abcdef0123--0", files=("qlog.zst", "qcamera.ts", "fcamera.hevc"))
    with open(os.path.join(self.disk.root, "00000101--abcdef0123--0", "qcamera.ts"), "wb") as f:
      f.write(bytes(range(256)) * 4)
    self.disk.drives.index_all()
    self.params = FakeParams({"IsOffroad": True, "GitBranch": "golf8-dev", "GitCommit": "f1306b2c6aaaa"})
    device = drivesd.Device(self.params, data_dir=self.disk.tmp.name)
    link = drivesd.Link(os.path.join(self.disk.tmp.name, "link"))
    self.server = ThreadingHTTPServer(("127.0.0.1", 0), drivesd.handler(self.disk.drives, device, link))
    threading.Thread(target=self.server.serve_forever, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)
    self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

  def get(self, path, headers=None):
    with urllib.request.urlopen(urllib.request.Request(self.base + path, headers=headers or {}), timeout=5) as r:
      return r.status, dict(r.headers), r.read()

  def test_list_and_drive(self):
    _, _, body = self.get("/api/drives")
    self.assertEqual(json.loads(body)["drives"][0]["id"], "00000101--abcdef0123")
    _, _, body = self.get("/api/drives/00000101--abcdef0123")
    self.assertEqual(json.loads(body)["timeline"]["engaged"], [[10., 50.]])

  def test_video_range(self):
    status, headers, body = self.get("/api/drives/00000101--abcdef0123/0/qcamera.ts", {"Range": "bytes=10-19"})
    self.assertEqual(status, 206)
    self.assertEqual(body, bytes(range(10, 20)))
    self.assertEqual(headers["Content-Range"], "bytes 10-19/1024")
    status, headers, body = self.get("/api/drives/00000101--abcdef0123/qcamera.m3u8")
    self.assertEqual(headers["Content-Type"], "application/vnd.apple.mpegurl")

  def test_segments_as_one_file(self):
    self.disk.segment("00000101--abcdef0123--1", files=("qcamera.ts",))
    with open(os.path.join(self.disk.root, "00000101--abcdef0123--1", "qcamera.ts"), "wb") as f:
      f.write(b"B" * 100)
    status, headers, body = self.get("/api/drives/00000101--abcdef0123/0-1/qcamera.ts")
    self.assertEqual((status, len(body)), (200, 1124))
    self.assertEqual(body, bytes(range(256)) * 4 + b"B" * 100)
    self.assertIn('filename="00000101--abcdef0123--0-1--qcamera.ts"', headers["Content-Disposition"])
    status, headers, body = self.get("/api/drives/00000101--abcdef0123/0-1/qcamera.ts", {"Range": "bytes=1020-1029"})
    self.assertEqual((status, body, headers["Content-Range"]), (206, bytes([252, 253, 254, 255]) + b"B" * 6, "bytes 1020-1029/1124"))
    with self.assertRaises(urllib.error.HTTPError) as e:
      self.get("/api/drives/00000101--abcdef0123/1-0/qcamera.ts")
    self.assertEqual(e.exception.code, 404)
    with self.assertRaises(urllib.error.HTTPError):
      self.get("/api/drives/00000101--abcdef0123/0-500/qcamera.ts")   # more than MAX_SPAN

  def test_big_files_wait_for_offroad(self):
    self.params.values["IsOffroad"] = False
    with self.assertRaises(urllib.error.HTTPError) as e:
      self.get("/api/drives/00000101--abcdef0123/0/fcamera.hevc")
    self.assertEqual(e.exception.code, 409)
    status, _, _ = self.get("/api/drives/00000101--abcdef0123/0/qcamera.ts")
    self.assertEqual(status, 200)

  def test_device(self):
    _, _, body = self.get("/api/device")
    d = json.loads(body)
    self.assertEqual((d["onroad"], d["branch"], d["commit"]), (False, "golf8-dev", "f1306b2c6"))
    self.assertEqual(d["drives"]["count"], 1)
    self.assertIn("free", d["disk"])

  def test_not_found(self):
    for path in ("/api/drives/nope", "/api/drives/00000101--abcdef0123/0/../../x", "/api/other"):
      with self.assertRaises(urllib.error.HTTPError) as e:
        self.get(path)
      self.assertEqual(e.exception.code, 404, path)

  def test_link_over_tailscale(self):
    cert, key = self_signed()
    body = json.dumps({"key_hash": "a" * 64, "origin": ORIGIN, "cert": cert, "key": key}).encode()
    req = urllib.request.Request(self.base + "/api/link", body, {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
      self.assertEqual(len(json.loads(r.read())["link"]), 16)

  def test_peers_only(self):
    with patch.object(drivesd, "allowed", return_value=False), self.assertRaises(urllib.error.HTTPError) as e:
      self.get("/api/drives")
    self.assertEqual(e.exception.code, 403)


def self_signed(name="device.comma.example.cc"):
  """A certificate and key for name, as PEM text."""
  from cryptography import x509
  from cryptography.hazmat.primitives import hashes, serialization
  from cryptography.hazmat.primitives.asymmetric import ec
  from cryptography.x509.oid import NameOID
  key = ec.generate_private_key(ec.SECP256R1())
  subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, name)])
  now = datetime.datetime.now(datetime.UTC)
  cert = (x509.CertificateBuilder().subject_name(subject).issuer_name(subject).public_key(key.public_key())
          .serial_number(1).not_valid_before(now - datetime.timedelta(days=1)).not_valid_after(now + datetime.timedelta(days=30))
          .add_extension(x509.SubjectAlternativeName([x509.DNSName(name)]), critical=False).sign(key, hashes.SHA256()))
  return (cert.public_bytes(serialization.Encoding.PEM).decode(),
          key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()).decode())


ORIGIN = "https://comma.example.cc:7443"
KEY = "k" * 32


def free_port():
  with socket.socket() as s:
    s.bind(("127.0.0.1", 0))
    return s.getsockname()[1]


class TestLink(unittest.TestCase):
  def setUp(self):
    self.tmp = self.enterContext(tempfile.TemporaryDirectory())
    self.link = drivesd.Link(os.path.join(self.tmp, "link"))
    self.cert, self.key = self_signed()

  def body(self, **kw):
    return {"key_hash": hashlib.sha256(KEY.encode()).hexdigest(), "origin": ORIGIN, "cert": self.cert, "key": self.key, **kw}

  def test_save_and_check(self):
    self.assertFalse(self.link.ready())
    self.assertIsNone(self.link.digest())
    answer = self.link.save(self.body())
    self.assertEqual(answer, {"link": self.link.digest()})
    self.assertTrue(self.link.ready())
    self.assertEqual(os.stat(os.path.join(self.tmp, "link", "key.pem")).st_mode & 0o777, 0o600)
    self.assertTrue(self.link.key_ok({"Cookie": "a=b; navkey=" + KEY}))
    self.assertTrue(self.link.key_ok({"X-Key": KEY}))
    self.assertFalse(self.link.key_ok({"Cookie": "navkey=wrong"}))
    self.assertFalse(self.link.key_ok({}))
    self.assertNotIn(KEY.encode(), open(os.path.join(self.tmp, "link", "link.json"), "rb").read())   # only its hash

  def test_turned_down(self):
    self.assertIn("error", self.link.save(self.body(origin="http://x")))
    self.assertIn("error", self.link.save(self.body(key_hash="abc")))
    self.assertIn("error", self.link.save(self.body(cert="nope")))
    self.assertFalse(self.link.ready())

  def test_digest_follows_the_files(self):
    self.link.save(self.body())
    first = self.link.digest()
    cert, key = self_signed()
    self.link.save(self.body(cert=cert, key=key))
    self.assertNotEqual(first, self.link.digest())


class TestLan(unittest.TestCase):
  def setUp(self):
    self.disk = Disk()
    self.addCleanup(self.disk.tmp.cleanup)
    self.disk.segment("00000101--abcdef0123--0", files=("qlog.zst", "qcamera.ts", "fcamera.hevc"))
    self.disk.drives.index_all()
    self.device = drivesd.Device(FakeParams({"IsOffroad": True}), data_dir=self.disk.tmp.name)
    self.link = drivesd.Link(os.path.join(self.disk.tmp.name, "link"))
    self.cert, key = self_signed("localhost")
    self.ip = "127.0.0.1"
    self.lan = drivesd.LanServer(self.disk.drives, self.device, self.link, address=lambda: self.ip, port=free_port())
    self.addCleanup(self.lan._stop)
    self.link_body = {"key_hash": hashlib.sha256(KEY.encode()).hexdigest(), "origin": ORIGIN, "cert": self.cert, "key": key}
    self.ctx = ssl.create_default_context(cadata=self.cert)

  def get(self, path, headers=None, method="GET"):
    req = urllib.request.Request(f"https://localhost:{self.lan.port}{path}", headers=headers or {}, method=method)
    with urllib.request.urlopen(req, timeout=5, context=self.ctx) as r:
      return r.status, dict(r.headers), r.read()

  def test_off_until_linked(self):
    self.lan.step()
    self.assertIsNone(self.lan.server)
    self.assertEqual(self.device.state()["lan"], {"ip": "127.0.0.1", "port": self.lan.port, "serving": False, "link": None})

  def test_off_without_wifi(self):
    self.link.save(self.link_body)
    self.ip = None
    self.lan.step()
    self.assertIsNone(self.lan.server)

  def test_https_with_the_cookie_from_the_page(self):
    self.link.save(self.link_body)
    self.lan.step()
    self.assertTrue(self.device.state()["lan"]["serving"])
    status, headers, body = self.get("/api/drives", {"Cookie": "navkey=" + KEY, "Origin": ORIGIN})
    self.assertEqual(json.loads(body)["drives"][0]["id"], "00000101--abcdef0123")
    self.assertEqual((headers["Access-Control-Allow-Origin"], headers["Access-Control-Allow-Credentials"]), (ORIGIN, "true"))
    _, headers, _ = self.get("/api/ping", {"Cookie": "navkey=" + KEY, "Origin": "https://evil.example"})
    self.assertNotIn("Access-Control-Allow-Origin", headers)
    status, _, _ = self.get("/api/drives/00000101--abcdef0123/0/qcamera.ts", {"Cookie": "navkey=" + KEY, "Range": "bytes=0-3"})
    self.assertEqual(status, 206)

  def test_key_required(self):
    self.link.save(self.link_body)
    self.lan.step()
    for headers in ({}, {"Cookie": "navkey=wrong"}):
      with self.assertRaises(urllib.error.HTTPError) as e:
        self.get("/api/drives", headers)
      self.assertEqual(e.exception.code, 403)
    with self.assertRaises(urllib.error.HTTPError) as e:   # the link comes over Tailscale only
      self.get("/api/link", {"Cookie": "navkey=" + KEY, "Content-Type": "application/json"}, method="POST")
    self.assertEqual(e.exception.code, 404)

  def test_preflight(self):
    self.link.save(self.link_body)
    self.lan.step()
    status, headers, _ = self.get("/api/drives", {"Origin": ORIGIN, "Access-Control-Request-Method": "GET"}, method="OPTIONS")
    self.assertEqual(status, 204)
    self.assertEqual(headers["Access-Control-Allow-Private-Network"], "true")
    self.assertEqual(headers["Access-Control-Allow-Origin"], ORIGIN)

  def test_restarted_for_a_new_address_or_certificate(self):
    self.link.save(self.link_body)
    self.lan.step()
    first = self.lan.server
    self.lan.step()
    self.assertIs(self.lan.server, first)
    cert, key = self_signed("localhost")
    self.link.save({**self.link_body, "cert": cert, "key": key})
    self.lan.step()
    self.assertIsNot(self.lan.server, first)
    self.ctx = ssl.create_default_context(cadata=cert)
    self.assertEqual(self.get("/api/ping", {"X-Key": KEY})[0], 200)


if __name__ == "__main__":
  unittest.main()
