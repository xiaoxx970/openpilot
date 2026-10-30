"""
Curve speed planner with a driver profile: the "VW: Curve Speed from Driver Profile" option of Smart Cruise Control
(Vision). It replaces SCC-V's targets with ones measured from the owner's manual curves on a Golf 8 (2026-09,
739 qlog segments).

Pure numpy, no openpilot imports, so the same code runs in plannerd and in the offline closed-loop replay it was tuned
with.

Design goals (from the driver's own logs):
  * target speed depends only on the curve radius, never on the current speed
  * look ahead with the model path before the curve, but once inside the curve use the measured curvature
    (yaw rate / speed), which is flat for a constant-radius curve
  * the in-curve target is locked: it can only go down when the curve clearly tightens, never up until the exit
  * braking is planned backwards from the curve with a comfortable decel, escalating to a max decel only if needed
  * the exit re-accelerates at the driver's own rate
"""
from dataclasses import dataclass
import numpy as np

KPH = 1 / 3.6
NO_REQUIREMENT = 10.0   # m/s^2 reported for an input with nothing ahead to slow for


@dataclass
class CurveSpeedParams:
  # lateral acceleration the driver accepts, by speed (p50 of curve-limited manual curves)
  lat_bp: tuple = (20 * KPH, 35 * KPH, 45 * KPH, 55 * KPH, 68 * KPH, 90 * KPH)
  lat_v: tuple = (2.5, 2.3, 2.2, 2.1, 1.7, 1.4)
  lat_scale: float = 1.3
  # share of that comfortable lateral accel kept while the rain following distance is selected: (rain, heavy rain). Only
  # the comfort part: the reachable-steering cap and hard_lat are not grip limits. It moves at comfort_scale_rate (1/s), so
  # targets already being planned come down over a few seconds when rain mode starts (R40 m: 39 -> 37 / 36 km/h)
  rain_lat_scale: tuple = (0.9, 0.85)
  comfort_scale_rate: float = 0.05
  # what openpilot can actually make the car do: commanded curvature is capped at cmd_lat_max (m/s^2 at the EPS),
  # and the car only yaws a speed-dependent share of it (measured QFK_01 vs yaw rate, 277k samples, 2026-09-26)
  cmd_lat_max: float = 3.6
  cap_margin: float = 0.9
  hard_decel_bp: tuple = (50 * KPH, 80 * KPH, 100 * KPH)
  hard_decel_v: tuple = (1.3, 1.6, 2.0)   # decel allowed to meet the hard limit: only high speed needs real braking
  hard_lat: float = 3.0          # never plan above this real lateral accel, even when the comfort plan falls short
  yaw_ratio_bp: tuple = (35 * KPH, 45 * KPH, 55 * KPH, 65 * KPH, 75 * KPH, 85 * KPH, 95 * KPH, 105 * KPH, 115 * KPH, 125 * KPH, 135 * KPH)
  yaw_ratio_v: tuple = (1.0, 0.98, 0.93, 0.88, 0.81, 0.73, 0.67, 0.56, 0.48, 0.42, 0.35)
  min_target: float = 18 * KPH
  # braking plan
  decel_comfort: float = 1.1
  decel_max: float = 2.0
  jerk: float = 1.0               # m/s^3, both when building up and releasing braking
  reaction_time: float = 0.6      # s of travel assumed before braking bites
  brake_on: float = -0.25         # start when the plan needs at least this decel
  brake_off: float = -0.10
  approach_hold: float = 1.5      # s the preview must stay clear before an approach is dropped (it loses curves for a moment)
  # curve detection on measured curvature
  enter_lat: float = 1.0          # m/s^2 of lateral accel at the current speed to call it a curve
  exit_ratio: float = 0.90        # past the apex: curvature below this share of the peak seen in this curve
  exit_hold: float = 0.3          # s the curvature must stay below it (the driver re-accelerates 0.1-0.4 s after the apex)
  exit_margin: float = 0.5        # full exit accel once lateral accel is this share below the limit
  curve_decel: float = 1.1        # max decel used to bleed off speed inside a curve (unless something tighter is ahead)
  tighten_ratio: float = 1.10     # curve must tighten by 10 % ...
  tighten_hold: float = 0.5       # ... for this long before the locked target is lowered
  kappa_tau: float = 0.5          # s, low-pass on measured curvature
  target_rate: float = 2 * KPH    # m/s per s, max change rate of the published target
  # exit re-acceleration, by speed: the driver's accel once the curve has opened to < 70 % of its peak (90 manual exits,
  # 2026-10-26: p50 +0.39 < 30 km/h, +0.31 30-50, +0.16 > 50) ...
  exit_bp: tuple = (25 * KPH, 40 * KPH, 60 * KPH, 80 * KPH)
  exit_v: tuple = (0.40, 0.32, 0.18, 0.08)
  # ... scaled by how far it has opened (current / peak curvature): the driver is already at +0.13-0.18 above 90 %
  exit_share_bp: tuple = (0.6, 0.8, 0.95, 1.0)
  exit_share_v: tuple = (1.0, 0.75, 0.5, 0.0)
  apex_ratio: float = 0.97        # curvature this far below its peak: past the apex, the curve is opening
  # no speeding up in a curve until the measured curvature has stopped growing for this long (the apex): neither back up
  # to the locked speed nor from the near preview calling the apex early. In short city turns taken slower than the lock
  # the car was pushed +0.2-0.4 mid-turn (three times at one Nanjing junction, 2026-10-27/28). 0: as before
  apex_hold: float = 0.2
  peak_tol: float = 0.005         # growth below this share is the curvature levelling off, not still rising
  # the exit re-accelerates at the driver's rate while the road ahead can still be met by comfortable braking started
  # later, using up that room over this many seconds. 0: never faster than the constant accel that meets the next curve
  # exactly (before 2026-10-27), which held every exit back once the map showed the next curve (mountain road, mean
  # accel 3 s after the apex 0.09 vs the driver's 0.33)
  exit_room_time: float = 2.0
  # preview geometry
  lookahead_s: float = 7.0
  lookahead_m: float = 100.0
  # at highway speed 100 m is under 5 s: an exit ramp needs ~180 m to slow from 80 to 35 km/h at 1.1 m/s^2. The fast
  # preview reads 0.8-0.9 of the curvature there (mixed roads), so it may look further, blended in over the same speeds
  # as the gain tables
  lookahead_m_fast: float = 180.0
  # the model path under-reads curvature far ahead, and much more on winding roads (pred/actual p50 on road curves,
  # 2026-10-26: 50-75 m 0.85 mixed roads vs 0.55 mountain, 75-100 m 0.74 vs 0.37; the same from rlog and qlog paths).
  # Winding roads are driven slowly, so the slow table applies up to the first speed and the fast one from the second.
  preview_gain_bp: tuple = (30.0, 50.0, 62.0, 87.0, 100.0)
  preview_gain_v: tuple = (1.0, 1.105, 1.18, 1.43, 1.5)
  preview_gain_slow_v: tuple = (1.15, 1.3, 1.7, 2.5, 2.5)
  preview_gain_speed_bp: tuple = (60 * KPH, 90 * KPH)
  min_preview_dist: float = 4.0
  opening_time: float = 1.0       # s of preview ahead that must be less curved than now to call the apex early
  # map preview (optional): OSM node curvature ahead (circle through three nodes, as mapd computes it). On the 09-20
  # mountain roads it read curves of R < 150 m at 0.98-0.99 of the driven radius (p50), so it is used without a gain.
  map_gain: float = 1.0
  map_horizon_m: float = 250.0
  # with the map ahead the camera's far gain may raise its reading only up to what the map shows around that distance,
  # times this margin: the gain makes up for curves the model under-reads on winding roads, which the map sees too, but
  # on ordinary roads at 60-90 km/h it read a gentle R~300 curve as R~215 (80 -> 66 km/h vs the driver's 74, 2026-10-28)
  # while the map asked for no braking. Dropping the gain altogether with the map made mountain hairpins faster (worst
  # 5 % apex lateral accel 3.29 -> 3.60). 0: the gain is not gated
  map_gate_margin: float = 1.25
  map_gate_window: float = 20.0   # m around a camera point searched for map nodes: ~20 m node spacing plus GPS error
  # ... from these speeds on: winding roads are driven slower, and there the gate cost mountain exits (0.27 -> 0.20 on
  # the 09-20 hairpins, where the map draws some curves wider than they are)
  map_gate_speed_bp: tuple = (50 * KPH, 65 * KPH)
  map_min_dist: float = 4.0       # map points closer than this are left to the camera, which is accurate near the car


def curvature_along_path(x, y, yaw):
  """Return arc length s[i] and |curvature|[i] of a model path given as x, y, yaw arrays."""
  x = np.asarray(x, dtype=np.float64)
  y = np.asarray(y, dtype=np.float64)
  yaw = np.asarray(yaw, dtype=np.float64)
  ds = np.hypot(x[1:] - x[:-1], y[1:] - y[:-1])
  s = np.concatenate(([0.0], np.cumsum(ds)))
  if s[-1] < 5.0:
    return s, np.zeros_like(s)
  # yaw change per metre between consecutive points; the densely spaced first points are ignored
  seg = np.where(ds > 0.3, np.abs(yaw[1:] - yaw[:-1]) / np.maximum(ds, 0.3), 0.0)
  k = np.concatenate((seg[:1], seg))
  # 3-point median against single-point spikes: median(a,b,c) = max(min(a,b), min(max(a,b),c))
  a, b, c = k[:-2], k[1:-1], k[2:]
  mid = np.maximum(np.minimum(a, b), np.minimum(np.maximum(a, b), c))
  k[1:-1] = mid
  return s, k


class CurveSpeedPlanner:
  CRUISE, APPROACH, CURVE, EXIT = 'cruise', 'approach', 'curve', 'exit'

  def __init__(self, params: CurveSpeedParams | None = None, dt: float = 0.05):
    self.p = params or CurveSpeedParams()
    self.dt = dt
    self.comfort_scale = 1.0    # share of the comfortable lateral accel in use (rain mode lowers it)
    # curvature -> speed lookup, solved once (the lateral-accel table depends on speed, so it needs iterating), for dry and
    # for each rain scale; a scale between two of them blends their speeds
    self._lk = np.linspace(np.log(1e-5), np.log(0.3), 400)
    k = np.exp(self._lk)
    self._scales = np.array(sorted({1.0, *self.p.rain_lat_scale}))
    self._lv_by_scale = []
    for scale in self._scales:
      v = np.sqrt(2.2 / k)
      for _ in range(6):
        v = np.sqrt(self.lat_limit(v, scale) / k)
      self._lv_by_scale.append(v)
    self._lv = self._lv_by_scale[-1]
    vh = np.sqrt(2.2 / k)
    for _ in range(6):
      vh = np.sqrt(self.hard_limit(vh) / k)
    self._lvh = vh
    self.reset()

  def reset(self, comfort_scale=None):
    if comfort_scale is not None:
      self.comfort_scale = comfort_scale   # nothing is being commanded, so a new rain scale applies at once
    self.state = self.CRUISE
    self.kappa_f = 0.0
    self.kappa_lock = 0.0
    self.kappa_peak = 0.0
    self.peak_age = 0.0         # s since the measured curvature last grew in this curve
    self.v_lock = 0.0
    self.tighten_t = 0.0
    self.exit_t = 0.0
    self.clear_t = 0.0
    self.k_near = 0.0
    self.braking = False
    self.a_cmd = 0.0
    self.v_cap = 1e3            # speed cap handed to the planner (m/s)
    self.v_display = 0.0        # target shown to the driver, rate limited (m/s)
    self.a_req = 0.0
    self.a_req_hard = 0.0
    self.v_env = 1e3
    # what each input alone asks for, and whether the map's nodes beyond the camera set the braking now
    self.a_req_vision = NO_REQUIREMENT
    self.a_req_map = NO_REQUIREMENT
    self.map_limited = False

  # ---- helpers -------------------------------------------------------------------------------
  def lat_limit(self, v, comfort_scale=None):
    scale = self.comfort_scale if comfort_scale is None else comfort_scale
    comfort = scale * self.p.lat_scale * np.interp(v, self.p.lat_bp, self.p.lat_v)
    reachable = self.p.cap_margin * self.p.cmd_lat_max * np.interp(v, self.p.yaw_ratio_bp, self.p.yaw_ratio_v)
    return np.minimum(comfort, reachable)

  def hard_limit(self, v):
    reachable = self.p.cap_margin * self.p.cmd_lat_max * np.interp(v, self.p.yaw_ratio_bp, self.p.yaw_ratio_v)
    return np.minimum(self.p.hard_lat, reachable)

  def v_hard_for_kappa(self, kappa):
    k = np.maximum(np.asarray(kappa, dtype=np.float64), 1e-5)
    return np.maximum(np.interp(np.log(k), self._lk, self._lvh), self.p.min_target)

  def v_for_kappa(self, kappa):
    """Speed at which this curvature produces the driver's accepted lateral accel (speed-dependent table)."""
    k = np.maximum(np.asarray(kappa, dtype=np.float64), 1e-5)
    lk = np.log(k)
    if self.comfort_scale >= 1.0:
      v = np.interp(lk, self._lk, self._lv)
    else:
      s = float(np.clip(self.comfort_scale, self._scales[0], 1.0))
      i = min(int(np.searchsorted(self._scales, s, side='right')) - 1, len(self._scales) - 2)
      w = (s - self._scales[i]) / (self._scales[i + 1] - self._scales[i])
      v = (1.0 - w) * np.interp(lk, self._lk, self._lv_by_scale[i]) + w * np.interp(lk, self._lk, self._lv_by_scale[i + 1])
    return np.maximum(v, self.p.min_target)

  def exit_accel(self, v_ego):
    """The driver's re-acceleration out of the curve, by speed and by how far the curve has opened."""
    share = min(self.kappa_f, max(self.k_near, 0.5 * self.kappa_f)) / max(self.kappa_peak, 1e-6)
    return float(np.interp(v_ego, self.p.exit_bp, self.p.exit_v)) * float(np.interp(share, self.p.exit_share_bp, self.p.exit_share_v))

  def map_curvature_near(self, s, map_dist, map_kappa):
    """The map's tightest curvature within the gate window around each distance s: the nodes there and the profile
    interpolated between nodes (sparse nodes on a gentle curve, or the car between two nodes). No nodes = straight."""
    md = np.asarray(map_dist, dtype=np.float64)
    mk = np.asarray(map_kappa, dtype=np.float64)
    if not len(md):
      return np.zeros(len(s))
    w = self.p.map_gate_window
    k = np.maximum.reduce([np.interp(s - w, md, mk), np.interp(s, md, mk), np.interp(s + w, md, mk)])
    near = np.abs(md[None, :] - s[:, None]) <= w
    return np.maximum(k, np.max(np.where(near, mk[None, :], 0.0), axis=1))

  def exit_room(self, v_ego):
    """Accel the exit may use before the road ahead would need more than comfortable braking."""
    if self.p.exit_room_time <= 0.0:
      return self.a_req
    return max(0.0, (self.v_env - v_ego) / self.p.exit_room_time)

  def _rate_limit(self, cur, target, rate):
    step = rate * self.dt
    return cur + np.clip(target - cur, -step, step)

  # ---- main update ---------------------------------------------------------------------------
  def update(self, v_ego, a_ego, kappa_meas, path_x, path_y, path_yaw, path_t=None, lane_change=False, enabled=True,
             map_dist=None, map_kappa=None, comfort_scale=1.0):
    """comfort_scale: share of the comfortable lateral accel to plan with (rain mode); it is approached at
    comfort_scale_rate."""
    p = self.p
    dt = self.dt
    if not enabled or v_ego < 3.0:
      self.reset(comfort_scale)
      return self.v_cap, 0.0
    scale_prev = self.comfort_scale
    self.comfort_scale = float(self._rate_limit(scale_prev, comfort_scale, p.comfort_scale_rate))
    if self.state == self.CURVE and self.comfort_scale < scale_prev:
      # rain mode starting in a curve: the locked speed comes down with the lateral accel it was locked at
      self.v_lock *= float(np.sqrt(self.comfort_scale / scale_prev))
    # during a lane change the model path still follows the road, so keep planning from the preview;
    # only the measured curvature is polluted by the lane change yaw, so it must not start or reshape a curve

    # measured curvature, low-passed; this is what the car is actually driving right now
    alpha = dt / (p.kappa_tau + dt)
    self.kappa_f += alpha * (abs(kappa_meas) - self.kappa_f)

    # preview: curvature and required speed along the model path
    s, k = curvature_along_path(path_x, path_y, path_yaw)
    w_fast = float(np.interp(v_ego, p.preview_gain_speed_bp, (0.0, 1.0)))
    horizon = min(p.lookahead_m + w_fast * (p.lookahead_m_fast - p.lookahead_m), max(20.0, v_ego * p.lookahead_s))
    m = (s >= p.min_preview_dist) & (s <= horizon)
    w = float(np.interp(v_ego, p.preview_gain_speed_bp, (0.0, 1.0)))
    gain = w * np.interp(s[m], p.preview_gain_bp, p.preview_gain_v) + (1.0 - w) * np.interp(s[m], p.preview_gain_bp, p.preview_gain_slow_v)
    k_cam = k[m] * gain
    if map_dist is not None and map_kappa is not None and p.map_gate_margin > 0.0:
      g = float(np.interp(v_ego, p.map_gate_speed_bp, (0.0, 1.0)))
      if g > 0.0:
        gated = np.maximum(k[m], np.minimum(k_cam, p.map_gate_margin * p.map_gain * self.map_curvature_near(s[m], map_dist, map_kappa)))
        k_cam = k_cam + g * (gated - k_cam)
    s_ahead, k_ahead = s[m], k_cam
    n_vision = len(s_ahead)
    if map_dist is not None and map_kappa is not None and len(map_dist):
      # the map sees past what the camera can, so it is planned against just like the preview
      mm = (map_dist >= max(p.min_preview_dist, p.map_min_dist)) & (map_dist <= max(horizon, p.map_horizon_m))
      s_ahead = np.concatenate((s_ahead, map_dist[mm]))
      k_ahead = np.concatenate((k_ahead, p.map_gain * map_kappa[mm]))
    # the raw near preview (accurate to a few % within ~2 s) says whether the curve is opening ahead of the car
    near_raw = k[m][s[m] <= max(8.0, v_ego * p.opening_time)]
    self.k_near = float(np.max(near_raw)) if len(near_raw) else 0.0
    v_req = self.v_for_kappa(k_ahead) if len(k_ahead) else np.array([1e3])

    # required decel to meet every point ahead, after reaction distance and the jerk ramp
    s_eff = s_ahead - v_ego * (p.reaction_time + p.decel_comfort / (2 * p.jerk)) if len(k_ahead) else np.array([1e3])
    a_need = (v_req ** 2 - v_ego ** 2) / (2 * np.maximum(s_eff, 1.0))
    self.a_req = float(np.min(a_need)) if len(a_need) else 0.0
    self.a_req_vision = min(float(np.min(a_need[:n_vision])), NO_REQUIREMENT) if n_vision else NO_REQUIREMENT
    self.a_req_map = min(float(np.min(a_need[n_vision:])), NO_REQUIREMENT) if len(k_ahead) > n_vision else NO_REQUIREMENT
    # the fastest speed from which comfortable braking still meets every point ahead
    self.v_env = float(np.min(np.sqrt(v_req ** 2 + 2 * p.decel_comfort * np.maximum(s_eff, 0.0))))
    v_req_h = self.v_hard_for_kappa(k_ahead) if len(k_ahead) else np.array([1e3])
    a_need_h = (v_req_h ** 2 - v_ego ** 2) / (2 * np.maximum(s_eff, 1.0))
    self.a_req_hard = float(np.min(a_need_h)) if len(a_need_h) else 0.0

    lat_now = v_ego ** 2 * self.kappa_f

    # ---- state machine --------------------------------------------------------------------
    if self.state in (self.CRUISE, self.APPROACH, self.EXIT):
      if not lane_change and self.state != self.EXIT and lat_now >= p.enter_lat and self.kappa_f > 1 / 400.:
        self.state = self.CURVE
        self.kappa_lock = self.kappa_f
        # lock to the tighter of what we feel and what is right in front of us
        near = k_ahead[s_ahead <= max(15.0, v_ego * 1.5)] if len(k_ahead) else np.array([0.0])
        self.kappa_lock = max(self.kappa_lock, float(np.max(near)) if len(near) else 0.0)
        self.v_lock = float(self.v_for_kappa(self.kappa_lock))
        if self.v_display > 0.0:
          self.v_lock = min(self.v_lock, self.v_display)   # continue from the target already shown
        self.kappa_peak = self.kappa_f
        self.peak_age = 0.0
        self.tighten_t = self.exit_t = 0.0
      elif self.state == self.CRUISE and self.a_req < p.brake_on:
        self.state = self.APPROACH
        self.clear_t = 0.0
      elif self.state == self.APPROACH:
        # the preview drops a curve for a second at a time (crests, S-bends): hold off before giving up on it
        self.clear_t = self.clear_t + dt if self.a_req > p.brake_off else 0.0
        if self.clear_t >= p.approach_hold:
          self.state = self.CRUISE
      elif self.state == self.EXIT and self.v_cap >= 1e2:
        self.state = self.CRUISE

    elif self.state == self.CURVE:
      if self.kappa_f > self.kappa_peak * (1.0 + p.peak_tol):
        self.peak_age = 0.0
      else:
        self.peak_age += dt
      self.kappa_peak = max(self.kappa_peak, self.kappa_f)
      # tightening: only lower the lock when the curve is clearly and persistently tighter
      if not lane_change and self.kappa_f > p.tighten_ratio * self.kappa_lock:
        self.tighten_t += dt
        if self.tighten_t >= p.tighten_hold:
          self.kappa_lock = self.kappa_f
          self.v_lock = min(self.v_lock, float(self.v_for_kappa(self.kappa_lock)))
          self.tighten_t = 0.0
      else:
        self.tighten_t = 0.0
      # exit: curvature has dropped well below the locked value and nothing tighter is coming up
      tighter_ahead = self.a_req < p.brake_on
      if self.kappa_f < p.exit_ratio * self.kappa_peak and not tighter_ahead:
        self.exit_t += dt
        if self.exit_t >= p.exit_hold:
          self.state = self.EXIT
      else:
        self.exit_t = 0.0

    # ---- outputs -----------------------------------------------------------------------------
    if self.state == self.APPROACH:
      a_comfort = max(-p.decel_comfort, min(0.0, 1.1 * self.a_req))
      d_hard = min(p.decel_max, float(np.interp(v_ego, p.hard_decel_bp, p.hard_decel_v)))
      a_hard = max(-d_hard, min(0.0, 1.1 * self.a_req_hard))
      a_target = min(a_comfort, a_hard)
      v_cap_target = self.v_env
      self.v_cap = min(self.v_cap, max(v_cap_target, v_ego - 2.0)) if self.v_cap < 1e2 else max(v_cap_target, v_ego - 2.0)
      display = float(np.min(v_req))
    elif self.state == self.CURVE:
      self.kappa_peak = max(self.kappa_peak, self.kappa_f)
      # hold the locked speed; if we arrived too fast, bleed it off gently
      # hold the locked speed: bleed off gently if above it, come back up at the driver's exit rate if below it
      lim = self.lat_limit(v_ego)
      levelled = self.peak_age >= p.apex_hold
      opening = self.kappa_f < p.apex_ratio * self.kappa_peak or (levelled and self.k_near < p.apex_ratio * self.kappa_f)
      if opening and self.a_req >= 0.0:
        # past the apex with nothing tighter ahead: speed up at the driver's rate, never faster than the road ahead allows
        a_target = min(self.exit_accel(v_ego), self.exit_room(v_ego))
        self.v_lock = max(self.v_lock, v_ego)
      else:
        # below the locked speed it comes back up at the driver's rate, but only past the apex: the driver does not speed
        # up before it, and in city turns taken slower than the lock (traffic, the model) this was the mid-turn push
        up = float(np.interp(v_ego, p.exit_bp, p.exit_v)) * float(np.clip((lim - lat_now) / (p.exit_margin * lim), 0.0, 1.0))
        a_target = float(np.clip(0.5 * (self.v_lock - v_ego), -p.curve_decel, up if levelled else 0.0))
      if self.a_req < (a_target if p.exit_room_time <= 0.0 else min(a_target, 0.0)):   # something tighter still ahead
        d_hard = min(p.decel_max, float(np.interp(v_ego, p.hard_decel_bp, p.hard_decel_v)))
        a_target = min(max(-p.decel_comfort, 1.1 * self.a_req), max(-d_hard, 1.1 * self.a_req_hard), a_target)
      self.v_cap = self.v_lock
      display = self.v_lock
    elif self.state == self.EXIT:
      a_exit = min(self.exit_accel(v_ego), max(self.exit_room(v_ego), 0.0))
      a_target = a_exit
      if self.kappa_f > p.tighten_ratio * self.kappa_peak:   # S-bend / second apex in the same curve
        self.state = self.CURVE
        self.kappa_peak = self.kappa_f
        self.peak_age = 0.0
        self.v_lock = min(max(v_ego, self.v_lock), float(self.v_for_kappa(self.kappa_f)))
      self.v_cap = max(self.v_cap, v_ego) + a_exit * dt
      if self.a_req < p.brake_on:        # next curve already needs braking
        self.state = self.APPROACH
      if lat_now < 0.5 * p.enter_lat:
        self.v_cap = 1e3
      display = self.v_cap if self.v_cap < 1e2 else 0.0
    else:
      a_target = 0.0
      self.v_cap = 1e3
      display = 0.0

    self.map_limited = self.state != self.CRUISE and self.a_req_map < min(self.a_req_vision, p.brake_off)

    # jerk-limited braking command
    self.a_cmd = self._rate_limit(self.a_cmd, a_target, p.jerk)
    # published target: rate limited so the driver never sees it jump
    if display <= 0.0:
      self.v_display = 0.0
    elif self.v_display <= 0.0:
      self.v_display = display
    else:
      self.v_display = self._rate_limit(self.v_display, display, p.target_rate)
    return self.v_cap, self.a_cmd
