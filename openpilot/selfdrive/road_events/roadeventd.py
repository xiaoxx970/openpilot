#!/usr/bin/env python3
"""Road prompts for the VW cluster (ACC_Events): uphill / downhill ahead from the model, roundabout / crossing ahead from
mapd's offline OSM tiles. Display only: card shows them in openpilot's ACC HUD, nothing reads them for control.

Opt-in (param VwRoadEventPrompts). Light: 2 Hz, the latest modelV2 / liveLocationKalman only (conflated sockets), nice 10
on the little cores, away from card/selfdrived/controlsd (4), plannerd/radard (5), camerad (6) and modeld (7). The map
tiles around the car are read only when it reaches new ones.
"""
import math
import os
import time

from openpilot.selfdrive.road_events.events import ModelPath

RATE = 2.             # Hz
INPUT_TIMEOUT = 1.0   # s
LITTLE_CORES = {0, 1, 2, 3}
SOURCES = ("none", "model", "map")   # RoadEventsIC.Source by events.SOURCE_*


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


def fill_message(ev, out: dict) -> None:
  ev.event = out["event"]
  ev.distance = out["distance"]
  ev.source = SOURCES[out["source"]]
  ev.grade = out["grade"]
  ev.gradeAhead = out["gradeAhead"]
  ev.mapMatched = out["mapMatched"]
  ev.roadClass = out["roadClass"]


def main():
  import openpilot.cereal.messaging as messaging
  from openpilot.common.hardware.hw import Paths
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.road_events.events import RoadEvents

  lower_priority()
  model_sock = messaging.sub_sock("modelV2", conflate=True)
  pose_sock = messaging.sub_sock("liveLocationKalman", conflate=True)
  pm = messaging.PubMaster(["roadEventsIC"])
  core = RoadEvents(os.path.join(Paths.mapd_root(), "offline"))
  map_failed = False
  model = pose = None
  model_t = pose_t = -math.inf
  rk = Ratekeeper(RATE, print_delay_threshold=None)

  while True:
    msg = messaging.recv_one_or_none(model_sock)
    if msg is not None:
      model, model_t = msg.modelV2, time.monotonic()
    msg = messaging.recv_one_or_none(pose_sock)
    if msg is not None:
      pose, pose_t = msg.liveLocationKalman, time.monotonic()
    now = time.monotonic()

    out = None
    inputs = pose_inputs(pose) if pose is not None and now - pose_t < INPUT_TIMEOUT else None
    if inputs is not None:
      v, roll, pitch, course, lat, lon, position_ok = inputs
      path = model_path(model) if model is not None and now - model_t < INPUT_TIMEOUT else None
      try:
        out = core.update(now, v, roll, pitch, course, lat, lon, position_ok and not map_failed, path)
      except Exception:
        # a broken map tile must not take the model's prompts with it
        cloudlog.exception("roadeventd: map lookup failed, map prompts off for this drive")
        map_failed = True

    dat = messaging.new_message("roadEventsIC")
    dat.valid = out is not None
    if out is not None:
      fill_message(dat.roadEventsIC, out)
    pm.send("roadEventsIC", dat)
    rk.keep_time()


if __name__ == "__main__":
  main()
