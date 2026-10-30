"""Baidu's driving route planning (驾车路线规划) as navd's route: Mapbox-shaped steps, the polyline and its traffic.

Baidu's steps are road by road with no maneuver of their own (has_guidance is false for this API): a step starts where the
road name changes, often straight on. The maneuvers are taken from the geometry instead: a turn of TURN_MIN or more where
two steps meet, the heading measured over HEADING_BEFORE before and HEADING_AFTER after (short: a ramp starts to curl at
once). A turn back the other way within MERGE_GAP of a maneuver is part of it, as onto a side road (辅路) by a short ramp.
Turns inside a step (a cloverleaf, a bend) are the road itself and no maneuver. Each maneuver ends one navd step, as
Mapbox's steps end at their banner's maneuver.

Pure python + road_events.baidu, no openpilot imports.
"""
import math

from openpilot.selfdrive.road_events import baidu

TURN_MIN = 20.        # deg of heading change where two of Baidu's steps meet that is a maneuver
HEADING_BEFORE = 30.  # m before a junction over which the heading in is measured
HEADING_AFTER = 15.   # m after it, for the heading out
MERGE_GAP = 80.       # m after a maneuver in which one the other way is part of it
UNNAMED = "无名路"


def modifier(turn: float) -> str:
  """Mapbox's maneuver modifier for a heading change in degrees, right positive."""
  side = "right" if turn > 0 else "left"
  a = abs(turn)
  if a >= 160.:
    return "uturn"
  if a >= 120.:
    return f"sharp {side}"
  if a >= 60.:
    return side
  return f"slight {side}"


def heading(a: tuple[float, float], b: tuple[float, float]) -> float:
  """Degrees from north, clockwise, from (lat, lon) a to b."""
  east, north = baidu.ground(a[0], a[1], b[0], b[1])
  return math.degrees(math.atan2(east, north)) % 360.


def point_at(path: baidu.Path, s: float) -> tuple[float, float]:
  s = min(max(s, path[0][0]), path[-1][0])
  for (s0, la0, lo0), (s1, la1, lo1) in zip(path, path[1:], strict=False):
    if s <= s1:
      u = (s - s0) / (s1 - s0) if s1 > s0 else 0.
      return la0 + u * (la1 - la0), lo0 + u * (lo1 - lo0)
  return path[-1][1], path[-1][2]


def turn_at(path: baidu.Path, s: float) -> float:
  """Heading change in degrees, right positive, at s m along the path."""
  before, at, after = (point_at(path, s + d) for d in (-HEADING_BEFORE, 0., HEADING_AFTER))
  if at in (before, after):
    return 0.
  return (heading(at, after) - heading(before, at) + 180.) % 360. - 180.


def parse(resp: dict) -> tuple[list[dict], list[tuple[float, float]], list[tuple[float, float, int]]] | None:
  """(navd steps, the route's WGS-84 (lat, lon) points, its jams (start, length, status) from its first point) of the first
  route of an answer, None without one."""
  routes = ((resp.get("result") or {}).get("routes") or []) if resp.get("status") == 0 else []
  if not routes:
    return None
  steps = routes[0].get("steps") or []
  path = baidu.parse_path(steps)
  if len(path) < 2:
    return None

  # where each of Baidu's steps starts along the polyline, its road and its time
  starts, roads, durations = [], [], []
  for step in steps:
    first = (step.get("path") or "").split(";")[0]
    if not first:
      continue
    on = baidu.project(path, *baidu.lnglat(first), lo=starts[-1] if starts else 0.)
    starts.append(on[0] if on is not None else 0.)
    roads.append("" if step.get("road_name") in (None, UNNAMED) else step["road_name"])
    durations.append(float(step.get("duration") or 0.))
  length = path[-1][0]
  ends = starts[1:] + [length]

  # maneuvers: (distance along, modifier, road after), the arrival last
  maneuvers = []
  for i in range(1, len(starts)):
    turn = turn_at(path, starts[i])
    if abs(turn) < TURN_MIN:
      continue
    if maneuvers and starts[i] - maneuvers[-1][0] < MERGE_GAP and ("left" in maneuvers[-1][1]) != (turn < 0):
      continue
    road = next((r for r in roads[i:] if r), "")
    maneuvers.append((starts[i], modifier(turn), road))
  maneuvers.append((length, "straight", "目的地"))

  def duration(a: float, b: float) -> float:
    """Baidu's time for a..b m along the route, each of its steps' time spread evenly over it."""
    return sum(dt * max(0., min(b, e) - max(a, s)) / (e - s) for s, e, dt in zip(starts, ends, durations, strict=True) if e > s)

  out = []
  s0 = 0.
  for i, (s1, mod, road) in enumerate(maneuvers):
    points = [(s0, *point_at(path, s0))] + [p for p in path if s0 < p[0] < s1] + [(s1, *point_at(path, s1))]
    arrive = i == len(maneuvers) - 1
    out.append({
      "distance": s1 - s0,
      "duration": duration(s0, s1),
      "geometry": {"coordinates": [[lon, lat] for _, lat, lon in points]},
      "bannerInstructions": [{"distanceAlongGeometry": s1 - s0,
                              "primary": {"text": road, "type": "arrive" if arrive else "turn", "modifier": mod}}],
    })
    s0 = s1
  return out, [(lat, lon) for _, lat, lon in path], baidu.traffic_runs(steps, 0.)
