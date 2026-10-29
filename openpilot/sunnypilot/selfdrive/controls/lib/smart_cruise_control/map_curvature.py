"""
Map curvature ahead of the car, for the curve speed driver profile (curve_speed.py).

mapd publishes MapCurvatures: the OSM nodes of the road it matched, in driving order (usually the whole road, behind the
car too), each with the unsigned curvature of the circle through it and its neighbours. This places the car on that
polyline from its GPS fix and returns the nodes ahead as distance along the road, which is what the planner plans its
braking against. Straight-line distances, as SCC-M uses, put the curves of a winding road too close.

In town mapd often publishes the road from a node ahead of the car (55-220 m ahead on 2026-10-28, Nanjing): the car is
then placed on the first segment's line, before its first node.

In hairpins mapd can publish the same road reversed for about a second while the car's bearing swings (4 times in
15 minutes of the 2026-09-20 mountain road, replayed through mapd itself). The bearing check alone accepted that table,
which would plan against the curves behind the car, so a new table is only used once the car is seen moving forward
along it. The last two tables are kept, so the original road is used again as soon as mapd switches back.

Pure numpy, no openpilot imports, so the offline replay runs the same code.
"""
import json
import math

import numpy as np

EARTH_RADIUS = 6371000.0


class _Table:
  NEW, CONFIRMED, REJECTED = 0, 1, 2

  def __init__(self, raw, points: list):
    self.raw = raw
    self.n = len(points) if len(points) >= 2 else 0
    self.last_seg = None
    self.state = self.NEW
    self.check_from = None     # (s_car, odometer) where the direction check started
    if not self.n:
      return
    pts = np.array(points, dtype=np.float64)
    self.lat0, self.lon0 = float(pts[0, 0]), float(pts[0, 1])
    self.cos0 = math.cos(math.radians(self.lat0))
    self.x = np.radians(pts[:, 1] - self.lon0) * self.cos0 * EARTH_RADIUS
    self.y = np.radians(pts[:, 0] - self.lat0) * EARTH_RADIUS
    self.dx, self.dy = np.diff(self.x), np.diff(self.y)
    self.seg_len2 = np.maximum(self.dx ** 2 + self.dy ** 2, 1e-6)
    self.seg_bearing = np.arctan2(self.dx, self.dy)        # clockwise from north, like the GPS bearing
    self.s = np.concatenate(([0.0], np.cumsum(np.sqrt(self.seg_len2))))
    self.kappa = np.abs(pts[:, 2])


class MapCurvatureAhead:
  def __init__(self, max_offset: float = 25.0, max_heading_error: float = 50.0, max_track_heading_error: float = 100.0,
               min_heading_speed: float = 5.0, horizon: float = 300.0, search_back: int = 3, search_ahead: int = 60,
               confirm_dist: float = 4.0, lead_in: float = 300.0):
    self.max_offset = max_offset                            # m from the road polyline: further and the car is on another road
    self.max_heading_error = math.radians(max_heading_error)
    # following the road from the last match: loose enough for the bearing lag through a hairpin, still rules out its
    # other leg, which runs the opposite way
    self.max_track_heading_error = math.radians(max_track_heading_error)
    self.min_heading_speed = min_heading_speed              # m/s; GPS bearing is noise below this
    self.horizon = horizon                                  # m along the road
    self.search_back = search_back                          # segments around the last match that are searched
    self.search_ahead = search_ahead
    self.confirm_dist = confirm_dist                        # m driven to confirm that a new table runs the car's way
    self.lead_in = lead_in                                  # m before a table's first node the car may still be placed
    self.table = _Table(None, [])
    self.previous = _Table(None, [])
    self.s_car = 0.0                                        # the car's place along the road at the last match (m)

  def set_points(self, raw) -> None:
    """raw: the MapCurvatures param as mapd writes it (JSON list of {latitude, longitude, curvature}), or None."""
    if raw == self.table.raw:
      return
    if raw == self.previous.raw:
      self.table, self.previous = self.previous, self.table
      return
    points = []
    if raw:
      try:
        points = [(float(p['latitude']), float(p['longitude']), float(p['curvature'])) for p in json.loads(raw)]
      except (ValueError, KeyError, TypeError):
        points = []
    self.table, self.previous = _Table(raw, points), self.table

  def _match(self, tb: _Table, px: float, py: float, bearing: float, max_error: float | None, segs: np.ndarray):
    """Best segment among `segs` for the car at (px, py): (segment, share along it), or None. `max_error`: largest
    heading difference to the segment (rad), None to skip the check."""
    # before the first node the car is placed on the first segment's line, up to lead_in metres back
    lo = np.where(segs == 0, -self.lead_in / np.sqrt(tb.seg_len2[segs]), 0.0)
    t = np.clip(((px - tb.x[segs]) * tb.dx[segs] + (py - tb.y[segs]) * tb.dy[segs]) / tb.seg_len2[segs], lo, 1.0)
    offset = np.hypot(px - (tb.x[segs] + t * tb.dx[segs]), py - (tb.y[segs] + t * tb.dy[segs]))
    ok = offset <= self.max_offset
    if max_error is not None:
      err = (tb.seg_bearing[segs] - bearing + math.pi) % (2 * math.pi) - math.pi
      ok &= np.abs(err) <= max_error
    if not ok.any():
      return None
    i = int(np.argmin(np.where(ok, offset, np.inf)))
    return int(segs[i]), float(t[i])

  def ahead(self, lat: float, lon: float, bearing_deg: float, speed: float, odometer: float, advance: float = 0.0):
    """Distance along the road (m) and |curvature| (1/m) of the map nodes ahead of the car, or None when the car is not
    on the mapped road. `odometer`: distance the car has driven in total (m), `advance`: metres since this GPS fix.

    The car is first placed with its heading checked against the road's (at speed), anywhere on the table; after that
    only near the last match, so slow driving keeps its place and a hairpin's other leg close by is not picked up."""
    tb = self.table
    if not tb.n or tb.state == _Table.REJECTED:
      return None
    px = math.radians(lon - tb.lon0) * tb.cos0 * EARTH_RADIUS
    py = math.radians(lat - tb.lat0) * EARTH_RADIUS
    bearing = math.radians(bearing_deg)
    check_heading = speed > self.min_heading_speed
    match = None
    if tb.last_seg is not None:
      lo = max(0, tb.last_seg - self.search_back)
      segs = np.arange(lo, min(tb.n - 1, tb.last_seg + self.search_ahead + 1))
      match = self._match(tb, px, py, bearing, self.max_track_heading_error if check_heading else None, segs)
    if match is None and check_heading:
      match = self._match(tb, px, py, bearing, self.max_heading_error, np.arange(tb.n - 1))
    if match is None:
      tb.last_seg = None
      tb.check_from = None
      return None
    seg, share = match
    tb.last_seg = seg
    s_car = tb.s[seg] + share * (tb.s[seg + 1] - tb.s[seg]) + max(advance, 0.0)
    self.s_car = s_car

    if tb.state == _Table.NEW:
      # the car must be seen moving forward along a new table before it is used
      if tb.check_from is None:
        tb.check_from = (s_car, odometer)
        return None
      driven = odometer - tb.check_from[1]
      if driven < self.confirm_dist:
        return None
      progress = s_car - tb.check_from[0]
      if progress > 0.5 * driven:
        tb.state = _Table.CONFIRMED
      elif progress < -0.5 * driven:
        tb.state = _Table.REJECTED        # runs against the car: mapd's reversed road in a hairpin
        return None
      else:
        tb.check_from = (s_car, odometer)
        return None

    i0, i1 = np.searchsorted(tb.s, [s_car, s_car + self.horizon], side='right')
    return tb.s[i0:i1] - s_car, tb.kappa[i0:i1]
