"""Offline OSM roads from mapd's tiles, for the cluster's roundabout / crossing prompts.

mapd keeps 0.25 deg tiles of the OSM roads in <mapd root>/offline/<lat, 2 deg>/<lon, 2 deg>/<box>. Per road (way) they
keep the nodes, name, ref, lanes, oneway and the highway class, but no node tags (traffic signals) and no junction tag:
- roundabouts come from geometry: one-way ways (mapd makes junction=roundabout ways one-way) that chain into a small
  loop turning counter-clockwise, as in right-hand traffic, and rounder than a square;
- a crossing is a node the road ahead (trunk to tertiary) shares with another road of the same or a higher class that
  meets it at an angle, links (ramps), merges and short connectors left out, so the side streets of a city stay silent.

The car is placed on a road from its position and course, and that road is followed ahead, taking the straightest
continuation at every junction, for LOOKAHEAD m. Where roads are stacked (an elevated road over a street) the higher
class wins: then the street's junctions stay silent instead of firing under the elevated road.

numpy + pycapnp only, no openpilot imports, so the offline replay runs the same code.
"""
import math
import os
from typing import NamedTuple

import numpy as np

EARTH_RADIUS = 6371000.
M_PER_DEG = math.radians(1) * EARTH_RADIUS
TILE = 0.25                    # deg, mapd's tile size
LOAD_MARGIN = 0.006            # deg around the car whose tiles are kept loaded, more than LOOKAHEAD
LOOKAHEAD = 400.               # m of road followed ahead
MATCH_DIST = 25.               # m between the car and a road it can be on
MATCH_HEADING = math.radians(40)
AMBIGUOUS = 8.                 # m: roads this much farther than the nearest one count as just as near
CELL = 100.                    # m, grid of the segment index
MAX_TURN = math.radians(100)   # the road is not followed through sharper turns

# mapd's highway classes (offline.capnp). Rank: lower is more important.
LINKS = frozenset((2, 4, 6, 8, 10))
RAMPS = frozenset((2, 4))      # motorway and trunk links
RANK = {1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2, 7: 3, 8: 3, 9: 4, 10: 4, 11: 5, 12: 6, 13: 7}
CROSSING_ROADS = (1, 4)        # ranks of the car's road that get crossing prompts: trunk to tertiary. Motorways have
                               # none, and on minor roads every junction is one of equal class
MIN_CROSSING_LENGTH = 60.      # m: shorter ways are connectors (u-turn gaps, slip lanes), not crossing roads
MIN_CROSSING_ANGLE = math.radians(35)  # a road joining at a flatter angle merges or splits off, it does not cross

ROUNDABOUT_RADIUS = (8., 80.)  # m
ROUNDABOUT_ROUNDNESS = 0.25    # std / mean of the loop's distance to its center
ROUNDABOUT_MIN_Q = 0.85        # 4 pi area / perimeter^2 (circle 1, square 0.79): where two dual carriageways cross, the
                               # four one-way pieces between them close a small square loop, and a road split around an
                               # island a thin one. 83 of the 221 loops in the owner's tiles, the driven roundabouts 0.99+
ROUNDABOUT_MAX_WAYS = 10

_schema = None


def _offline_schema():
  global _schema
  if _schema is None:
    import capnp
    capnp.remove_import_hook()
    _schema = capnp.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "offline.capnp"))
  return _schema


def tile_key(lat: float, lon: float) -> tuple[float, float]:
  return math.floor(lat / TILE) * TILE, math.floor(lon / TILE) * TILE


def tile_path(root: str, key: tuple[float, float]) -> str:
  a, b = key
  return os.path.join(root, str(int(math.floor(a / 2) * 2)), str(int(math.floor(b / 2) * 2)),
                      f"{a:.6f}_{b:.6f}_{a + TILE:.6f}_{b + TILE:.6f}")


def node_keys(latlon: np.ndarray) -> np.ndarray:
  """One int64 per coordinate; OSM nodes shared by two ways have exactly the same coordinate in the tiles."""
  lat = np.round(latlon[:, 0] * 1e7).astype(np.int64) + 900_000_000
  lon = np.round(latlon[:, 1] * 1e7).astype(np.int64) + 1_800_000_000
  return lat * 3_600_000_001 + lon


def _wrap(a):
  return (a + math.pi) % (2 * math.pi) - math.pi


def _cell_key(cx, cy):
  return (np.asarray(cx, dtype=np.int64) + 2 ** 21) * 2 ** 22 + (np.asarray(cy, dtype=np.int64) + 2 ** 21)


def _grouped(keys: np.ndarray, *values: np.ndarray) -> tuple:
  """Sorted unique keys, slice bounds into the values sorted along with them (compact lookup tables)."""
  order = np.argsort(keys, kind="stable")
  keys = keys[order]
  first = np.flatnonzero(np.r_[True, keys[1:] != keys[:-1]]) if len(keys) else np.zeros(0, dtype=np.int64)
  return (keys[first], np.r_[first, len(keys)], *(v[order] for v in values))


class Match(NamedTuple):
  way: int
  seg: int          # segment i between nodes i and i + 1
  t: float          # position on it, 0..1
  direction: int    # +1 along the node order, -1 against it
  dist: float       # m from the car


class Feature(NamedTuple):
  distance: float   # m along the road ahead
  lat: float
  lon: float


class OfflineMap:
  def __init__(self, root: str):
    self.root = root
    self.keys: frozenset = frozenset()
    self._build([])

  # ---- loading ------------------------------------------------------------------------------------------------------
  def ensure(self, lat: float, lon: float) -> bool:
    """Load the tiles around the car if they changed; True when it (re)loaded."""
    m = LOAD_MARGIN
    need = {tile_key(lat + dy, lon + dx) for dy in (-m, m) for dx in (-m, m)}
    if need <= self.keys:
      return False
    keep = {tile_key(lat + dy, lon + dx) for dy in (-2 * m, 2 * m) for dx in (-2 * m, 2 * m)}
    self.load(frozenset(need | (self.keys & keep)))
    return True

  def load(self, keys: frozenset) -> None:
    schema = _offline_schema()
    ways: dict[int, tuple] = {}
    for key in keys:
      path = tile_path(self.root, key)
      if not os.path.isfile(path):
        continue
      with open(path, "rb") as f:
        tile = schema.Offline.from_bytes_packed(f.read(), traversal_limit_in_words=1 << 40, nesting_limit=64)
      for w in tile.ways:
        nodes = w.nodes
        n = len(nodes)
        if n < 2 or w.id in ways:
          continue
        latlon = np.empty((n, 2))
        latlon[:, 0] = np.fromiter((c.latitude for c in nodes), dtype=np.float64, count=n)
        latlon[:, 1] = np.fromiter((c.longitude for c in nodes), dtype=np.float64, count=n)
        ways[w.id] = (latlon, int(w.highwayClass.raw), bool(w.oneWay), str(w.name), str(w.ref))
      del tile
    self.keys = keys
    self._build(list(ways.values()))

  def _build(self, ways: list[tuple]) -> None:
    self.n = len(ways)
    self.lat0 = float(np.mean([k[0] for k in self.keys])) + TILE / 2 if self.keys else 0.
    self.lon0 = float(np.mean([k[1] for k in self.keys])) + TILE / 2 if self.keys else 0.
    self.kx = M_PER_DEG * math.cos(math.radians(self.lat0))
    self.way_class = np.array([w[1] for w in ways], dtype=np.int8)
    self.way_oneway = np.array([w[2] for w in ways], dtype=bool)
    self.way_name = [w[3] for w in ways]
    self.way_ref = [w[4] for w in ways]
    self.way_xy = [self.to_xy(w[0][:, 0], w[0][:, 1]) for w in ways]
    self.way_keys = [node_keys(w[0]) for w in ways]
    self.way_length = np.array([float(np.hypot(*np.diff(xy, axis=0).T).sum()) for xy in self.way_xy]) if ways else np.zeros(0)

    # junctions: the (way, node index) pairs of every node on more than one way, as sorted lookup tables
    keys = np.concatenate(self.way_keys) if ways else np.zeros(0, dtype=np.int64)
    owner = np.concatenate([np.full(len(k), i, dtype=np.int32) for i, k in enumerate(self.way_keys)]) if ways else np.zeros(0, dtype=np.int32)
    index = np.concatenate([np.arange(len(k), dtype=np.int32) for k in self.way_keys]) if ways else np.zeros(0, dtype=np.int32)
    ukeys, bounds, owner, index = _grouped(keys, owner, index)
    shared = np.flatnonzero(np.diff(bounds) > 1)
    keep = np.concatenate([np.arange(bounds[i], bounds[i + 1]) for i in shared]) if len(shared) else np.zeros(0, dtype=np.int64)
    self.j_keys, self.j_bounds, self.j_way, self.j_idx = _grouped(keys[np.argsort(keys, kind="stable")][keep], owner[keep], index[keep])

    # segment grid. numpy throughout: a city's four tiles hold ~160k segments, per-segment python objects cost ~25 MB more
    seg_count = np.array([len(xy) - 1 for xy in self.way_xy], dtype=np.int64)
    self.seg_way = np.repeat(np.arange(self.n, dtype=np.int32), seg_count)
    self.seg_idx = (np.arange(int(seg_count.sum())) - np.repeat(np.cumsum(seg_count) - seg_count, seg_count)).astype(np.int32)
    if len(self.seg_way):
      self.seg_a = np.concatenate([xy[:-1] for xy in self.way_xy])
      self.seg_b = np.concatenate([xy[1:] for xy in self.way_xy])
    else:
      self.seg_a = self.seg_b = np.zeros((0, 2))
    lo = np.floor(np.minimum(self.seg_a, self.seg_b) / CELL).astype(np.int64)
    hi = np.floor(np.maximum(self.seg_a, self.seg_b) / CELL).astype(np.int64)
    single = (lo == hi).all(axis=1)                                 # most segments sit in one cell
    one, multi = np.flatnonzero(single), np.flatnonzero(~single)
    # every cell of each longer segment's bounding box, x-major
    nx, ny = hi[multi, 0] - lo[multi, 0] + 1, hi[multi, 1] - lo[multi, 1] + 1
    cells = nx * ny
    k = np.arange(int(cells.sum())) - np.repeat(np.cumsum(cells) - cells, cells)
    ny_rep = np.repeat(ny, cells)
    pairs = np.concatenate([np.column_stack([lo[one], one]),
                            np.column_stack([np.repeat(lo[multi, 0], cells) + k // ny_rep, np.repeat(lo[multi, 1], cells) + k % ny_rep,
                                             np.repeat(multi, cells)])])
    self.cell_keys, self.cell_bounds, self.cell_segs = _grouped(_cell_key(pairs[:, 0], pairs[:, 1]), pairs[:, 2].astype(np.int32))

    self._find_roundabouts()

  def _find_roundabouts(self) -> None:
    """Way -> (center lat, lon, radius) for one-way ways that chain into a small round counter-clockwise loop."""
    self.roundabout: dict[int, tuple[float, float, float]] = {}
    max_len = 2 * math.pi * ROUNDABOUT_RADIUS[1]
    starts: dict[int, list[int]] = {}
    turns_left = np.zeros(self.n, dtype=bool)
    for w in range(self.n):
      if not self.way_oneway[w] or self.way_length[w] > max_len:
        continue
      xy = self.way_xy[w]
      d = np.diff(xy, axis=0)
      cross = d[:-1, 0] * d[1:, 1] - d[:-1, 1] * d[1:, 0]
      turns_left[w] = len(cross) == 0 or (cross >= -1e-6).mean() >= 0.8
      if turns_left[w]:
        starts.setdefault(int(self.way_keys[w][0]), []).append(w)
    for w0 in range(self.n):
      if not turns_left[w0] or w0 in self.roundabout:
        continue
      chain, cur = [w0], w0
      while len(chain) <= ROUNDABOUT_MAX_WAYS:
        end = int(self.way_keys[cur][-1])
        if end == int(self.way_keys[w0][0]):
          self._add_roundabout(chain)
          break
        nxt = [n for n in starts.get(end, ()) if n not in chain and n not in self.roundabout]
        if not nxt or sum(self.way_length[c] for c in chain) > max_len:
          break
        cur = nxt[0]
        chain.append(cur)

  def _add_roundabout(self, chain: list[int]) -> None:
    xy = np.concatenate([self.way_xy[w][:-1] for w in chain])
    if len(xy) < 4:
      return
    c = xy.mean(axis=0)
    r = np.hypot(*(xy - c).T)
    closed = np.vstack([xy, xy[:1]])
    area = 0.5 * float(np.sum(closed[:-1, 0] * closed[1:, 1] - closed[1:, 0] * closed[:-1, 1]))
    perimeter = float(np.hypot(*np.diff(closed, axis=0).T).sum())
    if area <= 0 or not (ROUNDABOUT_RADIUS[0] <= r.mean() <= ROUNDABOUT_RADIUS[1]) or r.std() > ROUNDABOUT_ROUNDNESS * r.mean() or \
       4 * math.pi * area < ROUNDABOUT_MIN_Q * perimeter ** 2:
      return
    lat, lon = self.to_latlon(c[0], c[1])
    for w in chain:
      self.roundabout[w] = (lat, lon, float(r.mean()))

  # ---- geometry -----------------------------------------------------------------------------------------------------
  def to_xy(self, lat, lon) -> np.ndarray:
    return np.stack([(np.asarray(lon) - self.lon0) * self.kx, (np.asarray(lat) - self.lat0) * M_PER_DEG], axis=-1)

  def to_latlon(self, x: float, y: float) -> tuple[float, float]:
    return float(self.lat0 + y / M_PER_DEG), float(self.lon0 + x / self.kx)

  def junction(self, key: int) -> list[tuple[int, int]]:
    """(way, node index) of every way through the node with this key, empty unless several ways share it."""
    i = int(np.searchsorted(self.j_keys, key))
    if i == len(self.j_keys) or self.j_keys[i] != key:
      return []
    a, b = self.j_bounds[i], self.j_bounds[i + 1]
    return list(zip(self.j_way[a:b].tolist(), self.j_idx[a:b].tolist(), strict=True))

  # ---- matching -----------------------------------------------------------------------------------------------------
  def match(self, lat: float, lon: float, course: float, prev_way: int | None = None) -> Match | None:
    """The road the car is on: course in rad from north, clockwise."""
    if self.n == 0:
      return None
    p = self.to_xy(lat, lon)
    cx, cy = int(math.floor(p[0] / CELL)), int(math.floor(p[1] / CELL))
    want = _cell_key(np.repeat(np.arange(cx - 1, cx + 2), 3), np.tile(np.arange(cy - 1, cy + 2), 3))
    i = np.searchsorted(self.cell_keys, want)
    hit = i < len(self.cell_keys)
    hit[hit] = self.cell_keys[i[hit]] == want[hit]
    if not hit.any():
      return None
    s = np.unique(np.concatenate([self.cell_segs[self.cell_bounds[k]:self.cell_bounds[k + 1]] for k in i[hit]]))
    a, b = self.seg_a[s], self.seg_b[s]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-9), 0., 1.)
    dist = np.hypot(*(a + t[:, None] * ab - p).T)
    head = np.arctan2(ab[:, 0], ab[:, 1])
    fwd = np.abs(_wrap(head - course)) < MATCH_HEADING
    back = (np.abs(_wrap(head + math.pi - course)) < MATCH_HEADING) & ~self.way_oneway[self.seg_way[s]]
    ok = (fwd | back) & (dist < MATCH_DIST)
    if not ok.any():
      return None
    near = ok & (dist <= dist[ok].min() + AMBIGUOUS)
    pool = np.flatnonzero(near)
    ways = self.seg_way[s[pool]]
    if prev_way is not None and (ways == prev_way).any():
      pick = pool[ways == prev_way]
    else:
      rank = np.array([RANK.get(int(self.way_class[w]), 8) for w in ways])
      pick = pool[rank == rank.min()]
    k = pick[np.argmin(dist[pick])]
    return Match(int(self.seg_way[s[k]]), int(self.seg_idx[s[k]]), float(t[k]), 1 if fwd[k] else -1, float(dist[k]))

  def path_ahead(self, m: Match, length: float = LOOKAHEAD) -> tuple[list[tuple[float, int, int]], list[int]]:
    """Nodes of the road ahead as (distance, way, node index), following the straightest continuation; and its ways."""
    w, d = m.way, m.direction
    xy = self.way_xy[w]
    prev = xy[m.seg] + m.t * (xy[m.seg + 1] - xy[m.seg])
    j = m.seg + 1 if d > 0 else m.seg
    dist, nodes, path = 0., [], [w]
    while True:
      dist += float(np.hypot(*(xy[j] - prev)))
      prev = xy[j]
      nodes.append((dist, w, j))
      if dist >= length:
        break
      if 0 <= j + d < len(xy):
        j += d
        continue
      nxt = self._continuation(w, j, d)
      if nxt is None or nxt[0] in path:
        break
      w, j, d = nxt
      xy = self.way_xy[w]
      path.append(w)
      nodes.append((dist, w, j))   # the junction node again, as the first node of the next way
      j += d
    return nodes, path

  def _continuation(self, w: int, j: int, d: int) -> tuple[int, int, int] | None:
    xy = self.way_xy[w]
    h_in = math.atan2(*(xy[j] - xy[j - d]))
    best = None
    for w2, j2 in self.junction(int(self.way_keys[w][j])):
      if w2 == w:
        continue
      xy2 = self.way_xy[w2]
      for d2 in (1, -1):
        if (d2 < 0 and self.way_oneway[w2]) or not 0 <= j2 + d2 < len(xy2):
          continue
        turn = abs(_wrap(math.atan2(*(xy2[j2 + d2] - xy2[j2])) - h_in))
        same = (self.way_name[w2] and self.way_name[w2] == self.way_name[w]) or (self.way_ref[w2] and self.way_ref[w2] == self.way_ref[w])
        score = turn - (0.2 if same else 0.)
        if best is None or score < best[0]:
          best = (score, turn, w2, j2, d2)
    if best is None or best[1] > MAX_TURN:
      return None
    return best[2], best[3], best[4]

  def ramp_near(self, m: Match, ahead: float, behind: float) -> bool:
    """Does a motorway or trunk link leave or join the car's road within ahead m in front of it or behind m after it?"""
    for match, length in ((m, ahead), (m._replace(direction=-m.direction), behind)):
      nodes, _ = self.path_ahead(match, length)
      for _, w, j in nodes:
        if any(w2 != w and int(self.way_class[w2]) in RAMPS for w2, _ in self.junction(int(self.way_keys[w][j]))):
          return True
    return False

  # ---- features ahead -----------------------------------------------------------------------------------------------
  def features_ahead(self, m: Match) -> tuple[Feature | None, Feature | None]:
    """(roundabout, crossing) nearest on the road ahead, None for none."""
    nodes, path = self.path_ahead(m)
    in_roundabout = m.way in self.roundabout
    on_path = set(path)
    roundabout = crossing = None
    for dist, w, j in nodes:
      if roundabout is None and not in_roundabout and w in self.roundabout:
        lat, lon, _ = self.roundabout[w]
        roundabout = Feature(dist, lat, lon)
      if crossing is None and not in_roundabout and w not in self.roundabout and \
         CROSSING_ROADS[0] <= RANK.get(int(self.way_class[w]), 8) <= CROSSING_ROADS[1] and self._is_crossing(w, j, on_path):
        crossing = Feature(dist, *self.to_latlon(*self.way_xy[w][j]))
      if roundabout is not None and crossing is not None:
        break
    return roundabout, crossing

  def _is_crossing(self, w: int, j: int, on_path: set) -> bool:
    rank = RANK.get(int(self.way_class[w]), 8)
    xy = self.way_xy[w]
    ours = math.atan2(*(xy[min(j + 1, len(xy) - 1)] - xy[max(j - 1, 0)]))
    for w2, j2 in self.junction(int(self.way_keys[w][j])):
      c2 = int(self.way_class[w2])
      if w2 in on_path or w2 in self.roundabout or c2 in LINKS or RANK.get(c2, 8) > rank:
        continue
      if self.way_length[w2] < MIN_CROSSING_LENGTH:
        continue
      if (self.way_name[w2] and self.way_name[w2] == self.way_name[w]) or (self.way_ref[w2] and self.way_ref[w2] == self.way_ref[w]):
        continue   # the other carriageway of the same road
      xy2 = self.way_xy[w2]
      theirs = math.atan2(*(xy2[min(j2 + 1, len(xy2) - 1)] - xy2[max(j2 - 1, 0)]))
      if abs(_wrap(2 * (theirs - ours))) / 2 < MIN_CROSSING_ANGLE:
        continue   # merges into it or splits off
      return True
    return False
