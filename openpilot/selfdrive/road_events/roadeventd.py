#!/usr/bin/env python3
"""Road prompts for the VW cluster (ACC_Events): uphill / downhill ahead from the model, roundabout / crossing ahead from
mapd's offline OSM tiles, and with Baidu online data a speed camera ("speed limit ahead" with its limit, cruise API) and a
traffic jam ahead (route planning along the road ahead), and the cameras' limits for sunnypilot's speed limit resolver
(they replace the offline map's near a camera). card shows the prompts in openpilot's ACC HUD; the resolver's limits only
reach the cruise speed through Speed Limit Assist, which asks the driver every time. A traffic jam ahead also asks
selfdrived for the relaxed following distance until the car reaches it (prompts.JamAhead). While navigating with Baidu,
the traffic comes with navd's route (navRouteIC) and no route planning of our own is asked meanwhile.

Opt-in: param VwRoadEventPrompts for the prompts (1 while openpilot's ACC is on, 2 always: in ACC standby and with the main
switch off too), VwBaiduOnline (and a key file, see baidu.py) for Baidu; the jam following distance needs both. Light:
2 Hz, the latest modelV2 / liveLocationKalman only (conflated sockets), nice 10 on the little cores, away from
card/selfdrived/controlsd (4), plannerd/radard (5), camerad (6) and modeld (7). Map tiles are read only when the car
reaches new ones; Baidu's network I/O runs in its own thread.
"""
import math
import os
import time
from typing import Any

from openpilot.selfdrive.road_events.events import ModelPath

RATE = 2.             # Hz
INPUT_TIMEOUT = 1.0   # s
PARAMS_PERIOD = 5.    # s
KEY_RETRY = 60.       # s between looks for the Baidu key file
NAV_TIMEOUT = 30.     # s: navd sends its route every 10 s, a route not sent again for this long is gone
LITTLE_CORES = {0, 1, 2, 3}
SOURCES = ("none", "model", "map", "baidu")   # RoadEventsIC.Source by events.SOURCE_*
PROMPTS_OFF, PROMPTS_ENGAGED, PROMPTS_ALWAYS = 0, 1, 2   # VwRoadEventPrompts


def lower_priority() -> None:
  try:
    os.nice(10)
  except OSError:
    pass
  try:
    if LITTLE_CORES <= set(range(os.cpu_count() or 0)):
      os.sched_setaffinity(0, LITTLE_CORES)
  except (AttributeError, OSError):   # not on Linux
    pass


def pose_inputs(pose) -> tuple | None:
  """(v, roll, pitch, course, lat, lon, position_ok) from liveLocationKalman, None while its orientation is not valid."""
  if not pose.calibratedOrientationNED.valid:
    return None
  roll, pitch, yaw = pose.calibratedOrientationNED.value
  vn, ve, _ = pose.velocityNED.value
  v = math.hypot(vn, ve)
  lat, lon, _ = pose.positionGeodetic.value
  return v, roll, pitch, math.atan2(ve, vn) if v > 3. else yaw, lat, lon, bool(pose.gpsOK and pose.positionGeodetic.valid)


def model_path(model) -> ModelPath | None:
  if len(model.position.x) != 33:
    return None
  return list(model.position.x), list(model.position.y), list(model.position.z)


def nav_inputs(route) -> tuple:
  """(asked at, monotonic s; [(lat, lon), ...]; [(start, length, status), ...]) of navd's navRouteIC."""
  return (route.requestTime * 1e-9, [(c.latitude, c.longitude) for c in route.coordinates],
          [(j.start, j.length, j.status) for j in route.jams])


def fill_message(ev, out: dict, snap=None) -> None:
  ev.event = out["event"]
  ev.distance = out["distance"]
  ev.source = SOURCES[out["source"]]
  ev.grade = out["grade"]
  ev.gradeAhead = out["gradeAhead"]
  ev.mapMatched = out["mapMatched"]
  ev.roadClass = out["roadClass"]
  ev.jamLength = out.get("jamLength", 0.)
  ev.eventSpeedLimit = out.get("eventSpeedLimit", 0.)
  ev.jamFollow = out.get("jamFollow", False)
  ev.jamFollowDistance = out.get("jamFollowDistance", 0.)
  ev.rampNear = out.get("rampNear", False)
  if snap is not None:
    ev.baiduOnline = snap.online or snap.jams_online
    ev.speedLimitAhead = snap.camera_limit
    ev.speedLimitAheadDistance = snap.camera_distance


NO_PROMPT: dict[str, Any] = {"event": 0, "distance": 0., "source": 0, "grade": 0., "gradeAhead": 0., "mapMatched": False, "roadClass": 0, "roadKey": None,
                             "eventSpeedLimit": 0., "jamFollow": False, "jamFollowDistance": 0., "rampNear": False}


def main():
  import openpilot.cereal.messaging as messaging
  from openpilot.common.hardware.hw import Paths
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.road_events import baidu
  from openpilot.selfdrive.road_events.events import RoadEvents

  lower_priority()
  params = Params()
  model_sock = messaging.sub_sock("modelV2", conflate=True)
  pose_sock = messaging.sub_sock("liveLocationKalman", conflate=True)
  nav_sock = messaging.sub_sock("navRouteIC", conflate=True)
  pm = messaging.PubMaster(["roadEventsIC"])
  core = RoadEvents(os.path.join(Paths.mapd_root(), "offline"))
  map_failed = False

  def road_ahead(lat: float, lon: float, course: float, length: float):
    """The offline map's road ahead, for Baidu's route planning; None once the map failed."""
    nonlocal map_failed
    if map_failed:
      return None
    try:
      return core.points_ahead(lat, lon, course, length)
    except Exception:
      cloudlog.exception("roadeventd: map lookup failed, map prompts off for this drive")
      map_failed = True
      return None

  model = pose = None
  model_t = pose_t = nav_t = -math.inf
  nav = None
  prompts, baidu_on = PROMPTS_OFF, False
  online: baidu.BaiduOnline | None = None
  params_t = key_t = -math.inf
  odo, odo_t, road_key = 0., None, None
  rk = Ratekeeper(RATE, print_delay_threshold=None)

  while True:
    now = time.monotonic()
    if now - params_t >= PARAMS_PERIOD:
      params_t = now
      prompts = params.get("VwRoadEventPrompts", return_default=True) or PROMPTS_OFF
      baidu_on = params.get_bool("VwBaiduOnline")
    prompts_on = prompts != PROMPTS_OFF
    if not baidu_on:
      online = None
    elif online is None and now - key_t >= KEY_RETRY:
      key_t = now
      keys = baidu.load_keys()
      if keys is not None:
        online = baidu.BaiduOnline(keys, log=cloudlog.warning, count_file=baidu.COUNT_FILE)
        cloudlog.info("roadeventd: Baidu online data on")

    msg = messaging.recv_one_or_none(model_sock)
    if msg is not None:
      model, model_t = msg.modelV2, now
    msg = messaging.recv_one_or_none(pose_sock)
    if msg is not None:
      pose, pose_t = msg.liveLocationKalman, now
    msg = messaging.recv_one_or_none(nav_sock)
    if msg is not None:
      if nav is None or msg.navRouteIC.requestTime * 1e-9 != nav[0]:
        nav = nav_inputs(msg.navRouteIC) if len(msg.navRouteIC.coordinates) >= 2 else None
      nav_t = now

    out = snap = None
    inputs = pose_inputs(pose) if pose is not None and now - pose_t < INPUT_TIMEOUT else None
    if inputs is not None:
      v, roll, pitch, course, lat, lon, position_ok = inputs
      odo += v * (now - odo_t) if odo_t is not None else 0.
      odo_t = now
      if online is None:
        snap = baidu.Snapshot()   # Baidu off: no jam ahead, so no jam following distance either
      elif position_ok:
        snap = online.update(odo, lat, lon, course, v, road_key, road_ahead, nav if now - nav_t < NAV_TIMEOUT else None)
      out = dict(NO_PROMPT)
      if prompts_on or online is not None:
        path = model_path(model) if model is not None and now - model_t < INPUT_TIMEOUT else None
        try:
          out = core.update(now, v, roll, pitch, course, lat, lon, position_ok and not map_failed, path, snap)
        except Exception:
          # a broken map tile must not take the model's prompts with it
          cloudlog.exception("roadeventd: map lookup failed, map prompts off for this drive")
          map_failed = True
        road_key = out["roadKey"]
        if not prompts_on:   # Baidu's speed cameras for the resolver only: the map just tells it which road the car is on
          out.update(event=0, distance=0., source=0, jamLength=0., eventSpeedLimit=0., jamFollow=False, jamFollowDistance=0.)
    else:
      odo_t = None

    dat = messaging.new_message("roadEventsIC")
    dat.valid = out is not None
    if out is not None:
      fill_message(dat.roadEventsIC, out, snap)
    dat.roadEventsIC.promptAlways = prompts == PROMPTS_ALWAYS
    pm.send("roadEventsIC", dat)
    rk.keep_time()


if __name__ == "__main__":
  main()
