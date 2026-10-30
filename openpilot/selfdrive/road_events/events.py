"""From the car's inputs to one cluster road prompt. Shared by roadeventd and the offline replay; it imports nothing
outside this package."""
import math

import numpy as np
from numpy.typing import ArrayLike

from openpilot.selfdrive.road_events.osm_map import OfflineMap
from openpilot.selfdrive.road_events.prompts import EVENT_CROSSING, EVENT_JAM, EVENT_NONE, EVENT_ROUNDABOUT, EVENT_SPEED_LIMIT_AHEAD, CameraPrompt, \
  JamAhead, JamPrompt, MapPrompt, SlopePrompt
from openpilot.selfdrive.road_events.slope import grades_ahead, slope_ahead

MIN_SPEED = 5.          # m/s: below it the model path is short and the course unreliable
ROUNDABOUT_WINDOW = (30., 300.)   # m ahead where a roundabout is announced
RAMP_NEAR = (300., 200.)          # m ahead / behind the car a ramp junction counts for the car's sign recognition
MAINLINE = frozenset((1, 3))      # motorway, trunk
CROSSING_WINDOW = (50., 250.)
FAST_ROADS = frozenset((1, 2, 3, 4))   # OSM classes with motorway jam rules: motorway, trunk and their ramps

SOURCE_NONE, SOURCE_MODEL, SOURCE_MAP, SOURCE_BAIDU = 0, 1, 2, 3

ModelPath = tuple[ArrayLike, ArrayLike, ArrayLike]   # modelV2.position x, y, z


class RoadEvents:
  def __init__(self, map_root: str | None):
    self.map = OfflineMap(map_root) if map_root else None
    self.slope = SlopePrompt()
    self.roundabout = MapPrompt(EVENT_ROUNDABOUT, *ROUNDABOUT_WINDOW)
    self.crossing = MapPrompt(EVENT_CROSSING, *CROSSING_WINDOW)
    self.jam = JamPrompt()
    self.jam_ahead = JamAhead()
    self.camera = CameraPrompt()
    self.prev_way: int | None = None
    self.road_key: tuple[str, int] | None = None   # (name, OSM class) of the road the car was last placed on

  def update(self, t: float, v: float, roll: float, pitch: float, course: float, lat: float, lon: float, position_ok: bool,
             path: ModelPath | None, baidu=None) -> dict:
    """path: the model's (x, y, z) in the calibrated frame, or None. course: rad from north, clockwise.
    baidu: baidu.Snapshot of the road ahead (an empty one with Baidu off), or None without one for the car's position: its
    traffic jams ahead, for the prompt and the relaxed following distance, and its next speed camera. Crossings stay the
    map's (the owner's call)."""
    online = baidu is not None and baidu.online
    jams_online = baidu is not None and baidu.jams_online
    out = {"event": EVENT_NONE, "distance": 0., "source": SOURCE_NONE, "grade": math.tan(pitch), "gradeAhead": 0.,
           "mapMatched": False, "roadClass": 0, "jamLength": 0., "eventSpeedLimit": 0.,
           "baiduOnline": online or jams_online, "jamFollow": False, "jamFollowDistance": 0., "rampNear": False}

    sign, dist = 0, 0.
    if path is not None and v >= MIN_SPEED:
      grades = grades_ahead(*path, roll, pitch)
      sign, dist = slope_ahead(grades, SlopePrompt.THRESHOLD)
      finite = grades[np.isfinite(grades)]
      if len(finite):
        out["gradeAhead"] = float(finite[np.argmax(np.abs(finite))])
    slope = self.slope.update(t, out["grade"], sign, dist)

    roundabout = crossing = None
    if self.map is not None and position_ok and v >= MIN_SPEED:
      self.map.ensure(lat, lon)
      m = self.map.match(lat, lon, course, self.prev_way)
      self.prev_way = m.way if m is not None else None
      self.road_key = None
      if m is not None:
        out["mapMatched"] = True
        out["roadClass"] = int(self.map.way_class[m.way])
        self.road_key = (self.map.way_name[m.way] or self.map.way_ref[m.way], out["roadClass"])
        out["rampNear"] = out["roadClass"] in MAINLINE and self.map.ramp_near(m, *RAMP_NEAR)
        roundabout, crossing = self.map.features_ahead(m)
    else:
      self.prev_way = None   # slow or no position: the road key stays, the car is still on that road
    out["roadKey"] = self.road_key

    fast = self.road_key is not None and self.road_key[1] in FAST_ROADS
    jams = baidu.jams if jams_online else []
    jam_event, jam_distance, jam_length = EVENT_NONE, 0., 0.
    if v >= MIN_SPEED:   # crawling or stopped the car is in the queue already
      jam_event, jam_distance, jam_length = self.jam.update(t, jams, fast, v, gone=baidu is not None and not jams_online)
    out["jamFollow"], out["jamFollowDistance"] = self.jam_ahead.update(t, jams if baidu is not None else None, fast, v,
                                                                       v >= MIN_SPEED)
    camera_event, camera_distance, camera_limit = self.camera.update(t, v, baidu.camera_distance if online else 0.,
                                                                     baidu.camera_limit if online else 0., v >= MIN_SPEED,
                                                                     reported=baidu is not None)

    # one prompt at a time: a speed camera, a jam, the map's, then the model's
    for (event, d), source in (((camera_event, camera_distance), SOURCE_BAIDU), ((jam_event, jam_distance), SOURCE_BAIDU),
                               (self.roundabout.update(t, roundabout), SOURCE_MAP), (self.crossing.update(t, crossing), SOURCE_MAP),
                               (slope, SOURCE_MODEL)):
      if event != EVENT_NONE:
        out.update(event=event, distance=float(d), source=source)
        if event == EVENT_JAM:
          out["jamLength"] = float(jam_length)
        elif event == EVENT_SPEED_LIMIT_AHEAD:
          out["eventSpeedLimit"] = float(camera_limit)
        break
    return out

  def points_ahead(self, lat: float, lon: float, course: float, length: float) -> list[tuple[float, float]] | None:
    """The offline map's road ahead of the car as (lat, lon) points from where it is, up to length m along the straightest
    continuation; None off the map. Baidu's route planning asks along it."""
    if self.map is None:
      return None
    m = self.map.match(lat, lon, course, self.prev_way)
    if m is None:
      return None
    nodes, _ = self.map.path_ahead(m, length)
    xy = self.map.way_xy[m.way]
    start = xy[m.seg] + m.t * (xy[m.seg + 1] - xy[m.seg])
    return [self.map.to_latlon(*start)] + [self.map.to_latlon(*self.map.way_xy[w][j]) for _, w, j in nodes]
