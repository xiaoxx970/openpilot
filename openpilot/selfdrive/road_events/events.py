"""From the car's inputs to one cluster road prompt. Shared by roadeventd and the offline replay; it imports nothing
outside this package."""
import math

import numpy as np
from numpy.typing import ArrayLike

from openpilot.selfdrive.road_events.osm_map import OfflineMap
from openpilot.selfdrive.road_events.prompts import EVENT_CROSSING, EVENT_NONE, EVENT_ROUNDABOUT, MapPrompt, SlopePrompt
from openpilot.selfdrive.road_events.slope import grades_ahead, slope_ahead

MIN_SPEED = 5.          # m/s: below it the model path is short and the course unreliable
ROUNDABOUT_WINDOW = (30., 300.)   # m ahead where a roundabout is announced
CROSSING_WINDOW = (50., 250.)

SOURCE_NONE, SOURCE_MODEL, SOURCE_MAP = 0, 1, 2

ModelPath = tuple[ArrayLike, ArrayLike, ArrayLike]   # modelV2.position x, y, z


class RoadEvents:
  def __init__(self, map_root: str | None):
    self.map = OfflineMap(map_root) if map_root else None
    self.slope = SlopePrompt()
    self.roundabout = MapPrompt(EVENT_ROUNDABOUT, *ROUNDABOUT_WINDOW)
    self.crossing = MapPrompt(EVENT_CROSSING, *CROSSING_WINDOW)
    self.prev_way: int | None = None

  def update(self, t: float, v: float, roll: float, pitch: float, course: float, lat: float, lon: float, position_ok: bool,
             path: ModelPath | None) -> dict:
    """path: the model's (x, y, z) in the calibrated frame, or None. course: rad from north, clockwise."""
    out = {"event": EVENT_NONE, "distance": 0., "source": SOURCE_NONE, "grade": math.tan(pitch), "gradeAhead": 0.,
           "mapMatched": False, "roadClass": 0}

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
      if m is not None:
        out["mapMatched"] = True
        out["roadClass"] = int(self.map.way_class[m.way])
        roundabout, crossing = self.map.features_ahead(m)
    else:
      self.prev_way = None

    # one prompt at a time: the map's, then the model's
    for (event, d), source in ((self.roundabout.update(t, roundabout), SOURCE_MAP), (self.crossing.update(t, crossing), SOURCE_MAP),
                               (slope, SOURCE_MODEL)):
      if event != EVENT_NONE:
        out.update(event=event, distance=float(d), source=source)
        break
    return out
