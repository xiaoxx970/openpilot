#!/usr/bin/env python3
"""Classify the cars the VW cluster draws (lead, nearest left / right lane car) with a small YOLO model.

Opt-in (param VwVisionCarTypes) and only when the model is installed, since the weights are not in
this repository: put `yolo26n.onnx` (Ultralytics YOLO26n, exported at imgsz 320) into MODEL_DIR.
The first start compiles it for the CPU (~3.5 min) and caches the result there.

Load: one 320x320 inference per second at most (~0.7 s on a big core), on camerad's core 6 under
SCHED_IDLE, so camerad and every normal task always preempt it. It pauses while modeld drops frames
or the CPU is hot. The GPU is not used.

Output: /dev/shm/yolo_lead.json, read by opendbc volkswagen/vision_car_types.py (ACC_19 car types).
A stale file means "car", i.e. the stock icons.
"""
import json
import math
import os
import pickle
import threading
import time

import numpy as np

MODEL_DIR = "/data/yolo_lead"
ONNX_PATH = os.path.join(MODEL_DIR, "yolo26n.onnx")
PKL_PATH = os.path.join(MODEL_DIR, "yolo26n_cpu_tinygrad.pkl")
OUT_PATH = "/dev/shm/yolo_lead.json"
CORE = 6

INPUT = 320                 # model input side, px
COCO = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}

# ACC_19 car types, verified on a Golf 8 cluster
TYPE_PERSON, TYPE_TRUCK, TYPE_CAR, TYPE_TWO_WHEELER = 1, 2, 3, 4

# confidence needed to show a type (tuned on recorded drives and road tests)
TRUCK_MIN_CONF = 0.60       # truck + bus
SMALL_MIN_CONF = 0.45       # person / motorcycle / bicycle
CAR_MIN_CONF = 0.50

MIN_PERIOD = 1.0            # s between inferences
TRACK_FORGET = 30.0         # s without seeing a radar track before its type is dropped

# geometry (calibrated frame: x forward, y right, z down, origin at the road camera)
RADAR_TO_CAMERA = 1.52      # m, the constant radard uses between radar and camera
CAM_HEIGHT = 1.22           # m above the road
MIN_ROI_PX = 192
MAX_ROI_PX = 1600
MIN_FAR_PX = 18             # narrowest expected car (1.8 m) in model-input px before a group is split

# side lane cars: the same pick as carstate.parse_neighbour_leads
NEIGHBOUR_MIN_LAT_OFFSET = 1.0
MAX_TARGET_DISTANCE = 150.0

MAX_DROP_PERC = 1.0         # pause while modeld drops frames
MAX_CPU_TEMP = 85.0         # C
PAUSE_S = 10.0


# ---------------------------------------------------------------- image + model

def nv12_crop_rgb(y_plane, uv_plane, x0, y0, side, out=INPUT):
  """NV12 square region -> (1, 3, out, out) float RGB, area-averaged down to `out` px.

  Pixels outside the frame are grey (114), like the letterbox YOLO was trained with.
  """
  h, w = y_plane.shape
  blk = max(1, side // out)
  n = blk * out
  if blk > 1:                               # centre the block-aligned n x n region in the square
    x1, y1 = x0 + (side - n) // 2, y0 + (side - n) // 2
  else:
    x1, y1 = x0, y0
  x1 -= x1 % 2
  y1 -= y1 % 2
  xs = np.arange(x1, x1 + n) if blk > 1 else (x1 + (np.arange(out) + 0.5) * side / out).astype(np.int64)
  ys = np.arange(y1, y1 + n) if blk > 1 else (y1 + (np.arange(out) + 0.5) * side / out).astype(np.int64)
  inside = ((ys >= 0) & (ys < h))[:, None] & ((xs >= 0) & (xs < w))[None, :]
  xc = np.clip(xs, 0, w - 1)
  yc = np.clip(ys, 0, h - 1)
  Y = y_plane[yc[:, None], xc[None, :]].astype(np.float32)
  uv = uv_plane[(yc // 2)[:, None], (xc // 2)[None, :]].astype(np.float32) - 128.0
  U, V = uv[..., 0], uv[..., 1]
  rgb = np.stack([Y + 1.402 * V, Y - 0.344136 * U - 0.714136 * V, Y + 1.772 * U])
  rgb[:, ~inside] = 114.0
  if blk > 1:
    rgb = rgb.reshape(3, out, blk, out, blk).mean(axis=(2, 4))
  return (np.clip(rgb, 0, 255) / 255.0)[None].astype(np.float32)


def decode(raw, thr=0.25, iou=0.5):
  """Raw YOLO head (1, 84, N): cx, cy, w, h + 80 class scores -> [(name, conf, (x1, y1, x2, y2))], NMS per class."""
  o = raw[0]
  scores = o[4:]
  cls = scores.argmax(0)
  conf = scores.max(0)
  keep = (conf > thr) & np.isin(cls, list(COCO))
  if not keep.any():
    return []
  b = o[:4, keep].T
  cls, conf = cls[keep], conf[keep]
  xyxy = np.c_[b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2, b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2]
  picked: list[int] = []
  for i in conf.argsort()[::-1]:
    ok = True
    for j in picked:
      if cls[i] != cls[j]:
        continue
      a, c = xyxy[i], xyxy[j]
      inter = max(0.0, min(a[2], c[2]) - max(a[0], c[0])) * max(0.0, min(a[3], c[3]) - max(a[1], c[1]))
      union = (a[2] - a[0]) * (a[3] - a[1]) + (c[2] - c[0]) * (c[3] - c[1]) - inter
      if union > 0 and inter / union > iou:
        ok = False
        break
    if ok:
      picked.append(int(i))
  return [(COCO[int(cls[i])], float(conf[i]), tuple(float(v) for v in xyxy[i])) for i in picked]


def type_for(name, conf):
  """ACC_19 car type for a confident detection, None if it should not change what is shown."""
  if name in ("truck", "bus") and conf >= TRUCK_MIN_CONF:
    return TYPE_TRUCK
  if name in ("motorcycle", "bicycle") and conf >= SMALL_MIN_CONF:
    return TYPE_TWO_WHEELER
  if name == "person" and conf >= SMALL_MIN_CONF:
    return TYPE_PERSON
  if name == "car" and conf >= CAR_MIN_CONF:
    return TYPE_CAR
  return None


# ---------------------------------------------------------------- radar targets -> image

def expected_target(tid, d_rel, y_rel, device_from_calib, view_from_device, K):
  """Ground-contact point (u, v) and expected car width (px) of a radar object (yRel left-positive)."""
  x = d_rel + RADAR_TO_CAMERA
  p = view_from_device @ (device_from_calib @ np.array([x, -y_rel, CAM_HEIGHT]))
  if p[2] <= 0.5:
    return None
  uv = K @ (p / p[2])
  return {"tid": tid, "d": d_rel, "u": float(uv[0]), "vg": float(uv[1]), "w": float(K[0, 0] * 1.8 / x)}


def roi_for(exps):
  """Square (x0, y0, side) covering every target's expected box with some margin."""
  x0 = min(e["u"] - 1.4 * e["w"] for e in exps)
  x1 = max(e["u"] + 1.4 * e["w"] for e in exps)
  y0 = min(e["vg"] - 2.2 * e["w"] for e in exps)
  y1 = max(e["vg"] + 0.4 * e["w"] for e in exps)
  side = int(min(MAX_ROI_PX, max(MIN_ROI_PX, x1 - x0, y1 - y0)))
  side -= side % 2
  return int(round((x0 + x1 - side) / 2)) & ~1, int(round((y0 + y1 - side) / 2)) & ~1, side


def group_targets(exps):
  """Farthest first; add nearer targets while every car in the group stays >= MIN_FAR_PX wide."""
  rest = sorted(exps, key=lambda e: e["d"])
  groups = []
  while rest:
    g = [rest.pop()]
    for e in sorted(rest, key=lambda e: -e["d"]):
      if min(x["w"] for x in g + [e]) * INPUT / roi_for(g + [e])[2] >= MIN_FAR_PX:
        g.append(e)
    rest = [e for e in rest if e not in g]
    groups.append(g)
  return groups


def match_targets(dets, exps, x0, y0, side):
  """Detections (model-input px) -> {tid: (name, conf)}, joint greedy matching on position + size,
  so a car next to a target is never taken for it once it matches its own radar object better."""
  scale = side / INPUT
  pairs = []
  for i, (_, _, (a, _, c, e)) in enumerate(dets):
    cx, bottom, w = x0 + (a + c) / 2 * scale, y0 + e * scale, (c - a) * scale
    for j, ex in enumerate(exps):
      du = abs(cx - ex["u"]) / max(ex["w"], 8.0)
      dv = abs(bottom - ex["vg"]) / max(ex["w"] * 0.8, 8.0)
      ratio = w / max(ex["w"], 1.0)
      if du <= 1.2 and dv <= 1.5 and 0.4 < ratio < 3.5:
        pairs.append((du + dv, i, j))
  pairs.sort()
  used_d, used_t, out = set(), set(), {}
  for _, i, j in pairs:
    if i not in used_d and j not in used_t:
      used_d.add(i)
      used_t.add(j)
      out[exps[j]["tid"]] = (dets[i][0], dets[i][1])
  return out


def side_targets(points):
  """Nearest radar object in the left / right lane: {"left"|"right": (trackId, dRel, yRel)}."""
  best = {}
  for p in points:
    lane = str(p.laneAssignment)
    if lane == "left" and p.yRel >= NEIGHBOUR_MIN_LAT_OFFSET:
      slot = "left"
    elif lane == "right" and p.yRel <= -NEIGHBOUR_MIN_LAT_OFFSET:
      slot = "right"
    else:
      continue
    if 0 < p.dRel <= MAX_TARGET_DISTANCE and (slot not in best or p.dRel < best[slot][1]):
      best[slot] = (int(p.trackId), float(p.dRel), float(p.yRel))
  return best


class TypeTracker:
  """Per radar track: the latest confident type, with hysteresis back to "car"."""

  def __init__(self):
    self.tracks: dict[int, dict] = {}

  def _track(self, tid, now):
    tr = self.tracks.setdefault(tid, {"last": -math.inf, "type": None, "conf": 0.0, "seen": now, "last_raw": None})
    tr["seen"] = now
    return tr

  def touch(self, tid, now):
    if tid in self.tracks:
      self.tracks[tid]["seen"] = now

  def record(self, tid, now, typ, conf):
    tr = self._track(tid, now)
    tr["last"] = now
    if typ is None:
      return
    # a special type only falls back to "car" after two car results in a row, so one ambiguous
    # crop (a car right next to the truck) does not flip the icon
    if typ == TYPE_CAR and tr["type"] not in (None, TYPE_CAR) and tr["last_raw"] != TYPE_CAR:
      tr["last_raw"] = typ
      return
    tr["last_raw"] = typ
    tr["type"], tr["conf"] = typ, conf

  def shown(self, tid):
    tr = self.tracks.get(tid)
    return (tr["type"], tr["conf"]) if tr and tr["type"] is not None else (TYPE_CAR, 0.0)

  def known(self, tid):
    tr = self.tracks.get(tid)
    return tr is not None and tr["type"] is not None

  def last(self, tid, default):
    tr = self.tracks.get(tid)
    return tr["last"] if tr else default

  def forget_old(self, now):
    for tid in [t for t, tr in self.tracks.items() if now - tr["seen"] > TRACK_FORGET]:
      del self.tracks[tid]


def pick_group(groups, drawn, types, now):
  """Drawn-but-unknown cars first, then drawn, then undrawn-unknown (pre-classify), then the stalest."""
  def priority(g):
    score = 0.0
    for e in g:
      known = types.known(e["tid"])
      if e["tid"] in drawn:
        score += 1.0 if known else 3.0
      else:
        score += 0.2 if known else 2.0
      score += 0.1 * min(10.0, now - types.last(e["tid"], now - 10.0))
    return score
  return max(groups, key=priority)


def write_result(shown):
  """shown: {"lead"|"left"|"right": (tid, type, conf)}"""
  out: dict = {"t": time.monotonic()}
  for slot, (tid, typ, conf) in shown.items():
    out[slot] = {"tid": tid, "type": typ, "conf": round(conf, 3)}
  tmp = OUT_PATH + ".tmp"
  with open(tmp, "w") as f:
    json.dump(out, f)
  os.replace(tmp, OUT_PATH)


def core_online(core=CORE):
  try:
    with open(f"/sys/devices/system/cpu/cpu{core}/online") as f:
      return f.read().strip() == "1"
  except OSError:
    return True


def pin_idle():
  """camerad's core, lowest scheduling class: any runnable normal task preempts us immediately."""
  try:
    os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
  except (AttributeError, OSError):
    pass
  if core_online() and os.sched_getaffinity(0) != {CORE}:
    try:
      os.sched_setaffinity(0, {CORE})
    except OSError:
      pass


def load_model():
  """TinyJit'ed model from the cache (~4 s), else compile the ONNX for the CPU once (~3.5 min)."""
  os.environ["DEV"] = "CPU"                 # never touch the GPU modeld uses
  from tinygrad import Tensor, TinyJit
  infer = None
  if os.path.exists(PKL_PATH):
    try:
      with open(PKL_PATH, "rb") as f:
        infer = pickle.load(f)
      infer(Tensor(np.zeros((1, 3, INPUT, INPUT), np.float32))).numpy()
    except Exception:
      infer = None                          # built by another tinygrad version: rebuild
  if infer is None:
    from tinygrad.nn.onnx import OnnxRunner
    runner = OnnxRunner(ONNX_PATH)

    @TinyJit
    def jitted(x):
      return next(iter(runner({"images": x}).values())).realize()

    for _ in range(3):
      jitted(Tensor(np.zeros((1, 3, INPUT, INPUT), np.float32))).numpy()
    tmp = PKL_PATH + ".tmp"
    with open(tmp, "wb") as f:
      pickle.dump(jitted, f)
    os.replace(tmp, PKL_PATH)
    infer = jitted
  return lambda x: infer(Tensor(x)).numpy()


# ---------------------------------------------------------------- device loop

def main():
  import openpilot.cereal.messaging as messaging
  from msgq.visionipc import VisionIpcClient
  from openpilot.cereal.visionipc import VisionStreamType
  from openpilot.common.swaglog import cloudlog
  from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
  from openpilot.common.transformations.orientation import rot_from_euler

  pin_idle()
  if not os.path.exists(ONNX_PATH):
    cloudlog.warning(f"yolo_leadd: no model at {ONNX_PATH}, exiting")
    return
  run = load_model()
  cloudlog.info("yolo_leadd: model ready")

  sm = messaging.SubMaster(["radarState", "radarTracks", "extrinsicsCalibration", "modelV2", "deviceState", "narrowRoadCameraState"])

  def connect():
    c = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
    while not c.connect(False):
      time.sleep(0.5)
    return c

  vipc = connect()
  last_frame = time.monotonic()
  types = TypeTracker()
  paused_until = 0.0
  last_infer = -math.inf

  # The result file has to stay fresh while an inference blocks this loop, otherwise the reader
  # sees it as stale and flashes every icon back to "car". The tinygrad kernels run in C, so a
  # writer thread gets the GIL in between.
  shown_lock = threading.Lock()
  shown_now = dict.fromkeys(("lead", "left", "right"), (-1, TYPE_CAR, 0.0))

  def writer():
    while True:
      with shown_lock:
        snapshot = dict(shown_now)
      try:
        write_result(snapshot)
      except OSError:
        pass
      time.sleep(0.1)

  threading.Thread(target=writer, daemon=True).start()

  while True:
    buf = vipc.recv()
    sm.update(0)
    now = time.monotonic()
    if buf is None:
      if now - last_frame > 2.0:            # camerad restarted, the old connection is dead
        cloudlog.info("yolo_leadd: no frames, reconnecting to camerad")
        vipc = connect()
        last_frame = time.monotonic()
      continue
    last_frame = now
    pin_idle()                              # the kernel drops our affinity when core 6 goes offline

    if sm.updated["modelV2"] and sm["modelV2"].frameDropPerc > MAX_DROP_PERC:
      paused_until = now + PAUSE_S
    if sm.valid["deviceState"] and len(sm["deviceState"].cpuTempC) and max(sm["deviceState"].cpuTempC) > MAX_CPU_TEMP:
      paused_until = now + PAUSE_S

    # the three cars the cluster draws
    drawn_targets = {}
    lead = sm["radarState"].leadOne
    if sm.valid["radarState"] and lead.radar and 3.0 < lead.dRel < 100.0:
      drawn_targets["lead"] = (int(lead.radarTrackId), float(lead.dRel), float(lead.yRel))
    if sm.valid["radarTracks"]:
      drawn_targets.update(side_targets(sm["radarTracks"].points))

    # every radar object is a candidate, so a car is often known before the cluster draws it
    pool = {t[0]: t for t in drawn_targets.values()}
    if sm.valid["radarTracks"]:
      for p in sm["radarTracks"].points:
        if 3.0 < p.dRel <= MAX_TARGET_DISTANCE:
          pool.setdefault(int(p.trackId), (int(p.trackId), float(p.dRel), float(p.yRel)))
    for tid in pool:
      types.touch(tid, now)

    cam_key = (str(sm["deviceState"].deviceType), str(sm["narrowRoadCameraState"].sensor))
    ready = sm.seen["deviceState"] and sm.seen["narrowRoadCameraState"] and cam_key in DEVICE_CAMERAS
    if pool and ready and now >= paused_until and core_online() and now - last_infer >= MIN_PERIOD:
      cam = DEVICE_CAMERAS[cam_key].narrow_road
      K = cam.intrinsics * np.array([[buf.width / cam.width], [buf.height / cam.height], [1.0]])
      rpy = list(sm["extrinsicsCalibration"].rpyCalib)
      dfc = rot_from_euler(rpy if len(rpy) == 3 else [0.0, 0.0, 0.0])
      exps = [e for e in (expected_target(*t, dfc, view_frame_from_device_frame, K) for t in pool.values()) if e]
      if exps:
        group = pick_group(group_targets(exps), {t[0] for t in drawn_targets.values()}, types, now)
        x0, y0, side = roi_for(group)
        data = np.frombuffer(buf.data, dtype=np.uint8)
        y_plane = data[:buf.uv_offset].reshape(-1, buf.stride)[:buf.height, :buf.width]
        uv_plane = data[buf.uv_offset:buf.uv_offset + buf.stride * (buf.height // 2)]
        uv_plane = uv_plane.reshape(buf.height // 2, buf.stride)[:, :buf.width].reshape(buf.height // 2, buf.width // 2, 2)
        matched = match_targets(decode(run(nv12_crop_rgb(y_plane, uv_plane, x0, y0, side))), group, x0, y0, side)
        last_infer = t_done = time.monotonic()
        slot_of = {t[0]: k for k, t in drawn_targets.items()}
        for e in group:
          name, conf = matched.get(e["tid"], (None, 0.0))
          typ = type_for(name, conf) if name else None
          types.record(e["tid"], t_done, typ, conf)
          if name:
            cloudlog.info(f"yolo_leadd: {slot_of.get(e['tid'], 'pre')} tid {e['tid']} d {e['d']:.0f} -> {name} {conf:.2f} type {typ}")

    shown = {}
    for slot in ("lead", "left", "right"):
      if slot in drawn_targets:
        tid = drawn_targets[slot][0]
        shown[slot] = (tid, *types.shown(tid))
      else:
        shown[slot] = (-1, TYPE_CAR, 0.0)
    types.forget_old(now)
    with shown_lock:
      shown_now.update(shown)


if __name__ == "__main__":
  main()
