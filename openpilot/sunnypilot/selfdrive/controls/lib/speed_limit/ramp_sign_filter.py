"""Fork: the car's sign recognition reads the ramps' signs from a motorway mainline.

On a motorway or trunk mainline the VW sign recognition (carStateSP.speedLimit) also reads the limits posted for the exit
and entry ramps beside it: 40 or 60 next to the 100 the car keeps (October 2026 logs: about 10 an hour of mainline).
Such a reading is held back from Speed Limit Assist and the cluster (which shows the reading in use, longitudinalPlanSP's
carSpeedLimit) when all of these hold:
- it is RATIO or less of the one in use (or of the offline map's limit before the first one);
- it comes near a ramp junction (roadeventd's rampNear, from the offline map);
- the car is not in the rightmost lane and the right blinker is off.
A held reading is taken once the map puts the car off the mainline (a ramp) or the right blinker comes on, within KEEP m of
where it was read; a new reading replaces it. In the rightmost lane or with the right blinker on, a reading goes through
at once: assist asks before using any limit, so a car staying on the mainline only ignores the ask. Without a fresh map
match nothing is held back.

The rightmost lane: the model's right road edge less than RIGHT_GAP beyond its right lane line, X_LANE m ahead, over
SMOOTH s (at exits the gap is about 1 m, with a lane to the right 5-8 m).
"""
import collections
import time

import numpy as np

from openpilot.common.realtime import DT_MDL

RATIO = 0.7
RATIO_TOL = 0.1       # m/s: readings are whole km/h, so 70 of 100 counts however the floats round
KEEP = 500.            # m
RIGHT_GAP = 3.5        # m, about a lane width
EDGE_STD = 2.0         # m: a vaguer road edge does not count
X_LANE = 15.           # m ahead
SMOOTH = 1.0           # s
WINDOW = int(SMOOTH / DT_MDL)
ROAD_MAX_AGE = 1.5     # s without roadEventsIC (2 Hz) before the map match is taken as lost
MAINLINE = (1, 3)      # OSM classes: motorway, trunk


class RampSignFilter:
  def __init__(self):
    self.raw = 0.        # m/s, the latest reading
    self.accepted = 0.   # m/s, the reading in use
    self.reference = 0.  # m/s, the latest nonzero one in use
    self.held = 0.       # m/s, the reading held back, 0 = none
    self.held_at = 0.    # m, the odometer when it was read
    self.odo = 0.
    self.gaps: collections.deque = collections.deque(maxlen=WINDOW)

  def update(self, sm, v_ego: float) -> float:
    """The car's speed limit (m/s) to use."""
    self.odo += v_ego * DT_MDL
    road = self._road(sm)
    mainline = road is not None and road.roadClass in MAINLINE
    if mainline:
      self._lane(sm['modelV2'])
    else:
      self.gaps.clear()

    reading = float(sm['carStateSP'].speedLimit)
    if reading != self.raw:
      self.raw = reading
      if mainline and road.rampNear and 0. < reading <= RATIO * self._reference(sm) + RATIO_TOL and not self._right(sm):
        self.held, self.held_at = reading, self.odo
      else:
        self._accept(reading)
    elif self.held:
      if self.odo - self.held_at > KEEP:
        self.held = 0.   # not for this car: it stayed on the mainline
      elif (road is not None and road.roadClass not in MAINLINE) or sm['carState'].rightBlinker:
        self._accept(self.held)
    return self.accepted

  def _reference(self, sm) -> float:
    if self.reference > 0.:
      return self.reference
    map_data = sm['liveMapDataSP']
    return float(map_data.speedLimit) if map_data.speedLimitValid else 0.

  def _accept(self, reading: float):
    self.accepted, self.held = reading, 0.
    if reading > 0.:
      self.reference = reading

  @staticmethod
  def _road(sm):
    """roadeventd's map match, None without a fresh one."""
    if 'roadEventsIC' not in getattr(sm, 'services', ()) or not sm.seen['roadEventsIC'] or \
       time.monotonic() - sm.recv_time['roadEventsIC'] > ROAD_MAX_AGE:
      return None
    road = sm['roadEventsIC']
    return road if road.mapMatched else None

  def _lane(self, model):
    """The gap from the right lane line to the right road edge, a large one when the edge is vague."""
    if len(model.laneLines) < 4 or len(model.roadEdges) < 2:
      return
    line, edge = model.laneLines[2], model.roadEdges[1]
    if model.roadEdgeStds[1] > EDGE_STD:
      self.gaps.append(np.inf)
      return
    self.gaps.append(float(np.interp(X_LANE, edge.x, edge.y) - np.interp(X_LANE, line.x, line.y)))

  def _right(self, sm) -> bool:
    """In the rightmost lane, or the right blinker on."""
    rightmost = len(self.gaps) >= WINDOW // 2 and float(np.median(self.gaps)) < RIGHT_GAP
    return rightmost or bool(sm['carState'].rightBlinker)
