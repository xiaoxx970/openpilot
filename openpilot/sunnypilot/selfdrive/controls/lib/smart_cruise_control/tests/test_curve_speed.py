import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.curve_speed import CurveSpeedPlanner, CurveSpeedParams, \
  NO_REQUIREMENT
from openpilot.common.test import OpenpilotTestCase

KPH = 1 / 3.6


def road_curvature(s, radius, start, length):
  """|curvature| of a road with one constant-radius curve from `start` to `start + length` (m)."""
  return np.where((s >= start) & (s < start + length), 1 / radius, 0.)


def one_curve(radius, start, length):
  return lambda s: road_curvature(s, radius, start, length)


def straight(s):
  return np.zeros_like(s)


def short_turn(start, length, radius):
  """|curvature| of a short turn that tightens to `radius` at its middle and opens again, like a city junction."""
  def fn(s):
    x = 1.0 - np.abs(np.asarray(s, dtype=float) - (start + length / 2)) / (length / 2)
    return np.clip(x, 0.0, 1.0) / radius
  return fn


def model_path(s_car, v, curvature_fn):
  """A model path that follows the road exactly: positions and headings at the model's time samples."""
  s = s_car + max(v, 1.) * np.array(ModelConstants.T_IDXS)
  ds = np.diff(s, prepend=s[0])
  heading = np.cumsum(curvature_fn(s) * ds)
  x = np.cumsum(np.cos(heading) * ds)
  y = np.cumsum(np.sin(heading) * ds)
  return x, y, heading


def drive(curvature_fn, v0, duration, lane_change=lambda s: False, kappa_meas_fn=None, with_map=False, params=None,
          comfort_scale=lambda s: 1.):
  """Close the loop on a point-mass car: the planner's command when it acts, else hold the speed. `with_map`: the
  road's own curvature 5-245 m ahead as map input; `comfort_scale`: the rain scale asked for at each distance."""
  planner = CurveSpeedPlanner(params, dt=DT_MDL)
  v, a, s = v0, 0., 0.
  log = []
  map_dist = np.arange(5., 250., 10.)
  for _ in range(int(duration / DT_MDL)):
    kappa = float(kappa_meas_fn(s) if kappa_meas_fn else curvature_fn(np.array([s]))[0])
    mk = {'map_dist': map_dist, 'map_kappa': curvature_fn(s + map_dist)} if with_map else {}
    _, a_cmd = planner.update(v, a, kappa, *model_path(s, v, curvature_fn), lane_change=lane_change(s), comfort_scale=comfort_scale(s),
                              **mk)
    a_des = a_cmd if planner.state != CurveSpeedPlanner.CRUISE else 0.
    a += (a_des - a) * DT_MDL / 0.35
    v = max(0., v + a * DT_MDL)
    s += v * DT_MDL
    log.append((s, v, a, a_cmd, planner.state, planner.v_display, kappa))
  return planner, log


class TestCurveSpeedPlanner(OpenpilotTestCase):

  def test_brakes_before_the_curve_and_holds_a_steady_target(self):
    radius, start, length = 40., 250., 90.
    planner, log = drive(one_curve(radius, start, length), 60 * KPH, 25.)
    v_req = float(planner.v_for_kappa(1 / radius))

    entry = next(r for r in log if r[0] >= start)
    assert entry[1] < v_req * 1.1, f'entered at {entry[1] / KPH:.0f} km/h for a {v_req / KPH:.0f} km/h curve'
    first_brake = next(r for r in log if r[3] < -0.3)
    assert first_brake[0] < start, 'braking only started inside the curve'
    assert min(r[3] for r in log) >= -CurveSpeedParams().decel_max - 1e-6

    inside = [r for r in log if start + 10. <= r[0] <= start + length - 10.]
    targets = np.array([r[5] for r in inside])
    assert targets.max() - targets.min() < 3 * KPH, 'target moves inside a constant-radius curve'
    assert np.abs(np.diff(targets)).max() / DT_MDL <= CurveSpeedParams().target_rate + 1e-6

  def test_tight_curve_target_is_not_crawling(self):
    # the stock vision control asked 6-14 km/h of a 15 m radius the owner drives at ~26 km/h
    planner = CurveSpeedPlanner(dt=DT_MDL)
    assert planner.v_for_kappa(1 / 15.) > 20 * KPH

  def test_speeds_up_out_of_the_curve(self):
    radius, start, length = 40., 150., 60.
    _, log = drive(one_curve(radius, start, length), 45 * KPH, 25.)
    after = [r for r in log if start + length <= r[0] <= start + length + 15.]
    assert any(r[3] > 0.1 for r in after), 'no acceleration once the curve opens'

  def test_lane_change_yaw_does_not_start_a_curve(self):
    # a lane change at 80 km/h: measured curvature of R 150 m for 2 s while the road and the model path stay straight
    def kappa_meas(s):
      return 1 / 150. if 100. <= s < 145. else 0.

    def lane_change(s):
      return 90. <= s < 160.
    _, log = drive(straight, 80 * KPH, 10., lane_change=lane_change, kappa_meas_fn=kappa_meas)
    assert all(r[4] != CurveSpeedPlanner.CURVE for r in log)
    assert min(r[3] for r in log) > -0.1

  def test_disabled_resets(self):
    planner = CurveSpeedPlanner(dt=DT_MDL)
    curvature = one_curve(40., 60., 60.)
    for _ in range(40):
      planner.update(15., 0., 0., *model_path(0., 15., curvature))
    assert planner.state != CurveSpeedPlanner.CRUISE
    v_cap, a_cmd = planner.update(15., 0., 0., *model_path(0., 15., curvature), enabled=False)
    assert planner.state == CurveSpeedPlanner.CRUISE
    assert a_cmd == 0. and v_cap >= 100.

  def test_map_brakes_for_a_curve_the_camera_cannot_see(self):
    # straight model path, a 30 m radius on the map 120-160 m ahead at 60 km/h
    v = 60 * KPH
    map_dist = np.arange(5., 250., 10.)
    map_kappa = np.where((map_dist >= 120.) & (map_dist <= 160.), 1 / 30., 0.)
    planner = CurveSpeedPlanner(dt=DT_MDL)
    for _ in range(10):
      planner.update(v, 0., 0., *model_path(0., v, straight), map_dist=map_dist, map_kappa=map_kappa)
    assert planner.state == CurveSpeedPlanner.APPROACH and planner.a_cmd < 0.
    assert planner.map_limited
    assert planner.a_req_map < -0.25 and planner.a_req_vision == NO_REQUIREMENT

    camera_only = CurveSpeedPlanner(dt=DT_MDL)
    for _ in range(10):
      camera_only.update(v, 0., 0., *model_path(0., v, straight))
    assert camera_only.state == CurveSpeedPlanner.CRUISE and not camera_only.map_limited
    assert camera_only.a_req_map == NO_REQUIREMENT

  def test_camera_sets_the_braking_when_it_asks_for_more(self):
    # the camera sees a 40 m radius 80 m ahead; the map has only a gentle curve there
    v = 60 * KPH
    map_dist = np.arange(5., 250., 10.)
    map_kappa = np.where((map_dist >= 80.) & (map_dist <= 140.), 1 / 300., 0.)
    planner = CurveSpeedPlanner(dt=DT_MDL)
    for _ in range(10):
      planner.update(v, 0., 0., *model_path(0., v, one_curve(40., 80., 60.)), map_dist=map_dist, map_kappa=map_kappa)
    assert planner.state == CurveSpeedPlanner.APPROACH
    assert planner.a_req_vision < planner.a_req_map and not planner.map_limited

  def test_exit_speeds_up_although_the_map_shows_the_next_curve(self):
    # two 40 m radius curves 120 m apart: out of the first at the driver's rate, then slow down again for the second
    def two_curves(s):
      return road_curvature(s, 40., 150., 60.) + road_curvature(s, 40., 330., 60.)
    for exit_room_time, accelerates in ((2.0, True), (0.0, False)):
      planner, log = drive(two_curves, 45 * KPH, 45., with_map=True, params=CurveSpeedParams(exit_room_time=exit_room_time))
      # the planner's command while it still runs the exit (this harness holds the speed once it hands back)
      exit_cmd = max(r[3] for r in log if 200. <= r[0] <= 225.)
      assert (exit_cmd > 0.25) == accelerates, f'exit command {exit_cmd:.2f} out of the first curve, exit_room_time {exit_room_time}'
      entry = next(r for r in log if r[0] >= 330.)
      assert entry[1] < float(planner.v_for_kappa(1 / 40.)) * 1.1, 'too fast into the second curve'

  def test_no_speeding_up_before_the_apex_of_a_short_turn(self):
    # a junction turn tightening to R15 at its middle, entered at 16 km/h, slower than the planner's own target there:
    # it used to come back up to that target from the moment the turn began
    start, length = 40., 30.
    apex = start + length / 2
    for apex_hold, pushes in ((CurveSpeedParams().apex_hold, False), (0.0, True)):
      _, log = drive(short_turn(start, length, 15.), 16 * KPH, 25., params=CurveSpeedParams(apex_hold=apex_hold))
      before_apex = max(r[3] for r in log if start <= r[0] < apex)
      assert (before_apex > 0.1) == pushes, f'command {before_apex:.2f} before the apex, apex_hold {apex_hold}'
      after = max(r[3] for r in log if apex + 5. <= r[0] <= start + length + 10.)
      assert after > 0.1, 'no speeding up out of the turn'

  def test_map_keeps_the_camera_from_over_reading_a_gentle_curve(self):
    # 80 km/h, R300 from 80 m ahead: the camera's far gain reads it as ~R170 and brakes; the map shows the real radius
    v = 80 * KPH
    curve = one_curve(300., 80., 150.)
    map_dist = np.arange(5., 300., 20.)
    camera_only, with_map = CurveSpeedPlanner(dt=DT_MDL), CurveSpeedPlanner(dt=DT_MDL)
    for _ in range(10):
      camera_only.update(v, 0., 0., *model_path(0., v, curve))
      with_map.update(v, 0., 0., *model_path(0., v, curve), map_dist=map_dist, map_kappa=curve(map_dist))
    assert camera_only.state == CurveSpeedPlanner.APPROACH and camera_only.a_cmd < -0.2
    assert with_map.state == CurveSpeedPlanner.CRUISE and with_map.a_cmd == 0.

  def test_map_gate_keeps_tight_curves(self):
    # the same at R60, which the map shows too: the camera's gained reading stays and it brakes
    v = 80 * KPH
    curve = one_curve(60., 80., 60.)
    map_dist = np.arange(5., 300., 20.)
    planner = CurveSpeedPlanner(dt=DT_MDL)
    for _ in range(10):
      planner.update(v, 0., 0., *model_path(0., v, curve), map_dist=map_dist, map_kappa=curve(map_dist))
    assert planner.state == CurveSpeedPlanner.APPROACH and planner.a_cmd < -0.2

  def test_high_speed_respects_the_steering_cap(self):
    p = CurveSpeedParams()
    planner = CurveSpeedPlanner(p, dt=DT_MDL)
    v = 120 * KPH
    reachable = p.cap_margin * p.cmd_lat_max * float(np.interp(v, p.yaw_ratio_bp, p.yaw_ratio_v))
    assert float(planner.lat_limit(v)) <= reachable + 1e-9


def solve_v(planner, kappa, comfort_scale):
  # the speed whose lateral accel on this curvature is the limit, solved directly
  v = np.sqrt(2.2 / kappa)
  for _ in range(50):
    v = np.sqrt(planner.lat_limit(v, comfort_scale) / kappa)
  return max(float(v), planner.p.min_target)


class TestCurveSpeedRain(OpenpilotTestCase):
  RAIN, HEAVY = CurveSpeedParams().rain_lat_scale

  def test_only_the_comfort_part_is_scaled(self):
    p = CurveSpeedParams()
    planner = CurveSpeedPlanner(p, dt=DT_MDL)
    for v in (25 * KPH, 50 * KPH, 80 * KPH, 120 * KPH):
      comfort = p.lat_scale * float(np.interp(v, p.lat_bp, p.lat_v))
      reachable = p.cap_margin * p.cmd_lat_max * float(np.interp(v, p.yaw_ratio_bp, p.yaw_ratio_v))
      for scale in (1., self.RAIN, self.HEAVY):
        np.testing.assert_allclose(planner.lat_limit(v, scale), min(scale * comfort, reachable))
      np.testing.assert_allclose(planner.hard_limit(v), min(p.hard_lat, reachable))

  def test_rain_targets(self):
    # R40: 39 km/h dry, 37 in rain, 36 in heavy rain; scales between the tables blend, within 0.2 km/h of a direct solve
    planner = CurveSpeedPlanner(dt=DT_MDL)
    for radius in (15., 40., 120., 400.):
      dry = float(planner.v_for_kappa(1 / radius))
      for scale in (self.RAIN, self.HEAVY, (self.RAIN + self.HEAVY) / 2, (1. + self.RAIN) / 2):
        planner.comfort_scale = scale
        v = float(planner.v_for_kappa(1 / radius))
        assert abs(v - solve_v(planner, 1 / radius, scale)) < 0.2 * KPH, f'R{radius:.0f} at x{scale}'
        assert v <= dry
      planner.comfort_scale = 1.
    planner.comfort_scale = self.RAIN
    assert round(float(planner.v_for_kappa(1 / 40.)) / KPH) == 37
    planner.comfort_scale = self.HEAVY
    assert round(float(planner.v_for_kappa(1 / 40.)) / KPH) == 36

  def test_slower_curves_in_rain(self):
    # closed loop the car enters below the target (camera far gain, braking plan), so the speed drops by a little less than
    # sqrt(scale): R40 from 60 km/h 30.9 -> 30.1 -> 29.6 km/h
    radius, start, length = 40., 250., 90.
    apex = {}
    for scale in (1., self.RAIN, self.HEAVY):
      _, log = drive(one_curve(radius, start, length), 60 * KPH, 25., comfort_scale=lambda s, scale=scale: scale)
      apex[scale] = np.median([r[1] for r in log if start + 20. <= r[0] <= start + length - 20.])
    for scale in (self.RAIN, self.HEAVY):
      assert np.sqrt(scale) - 0.03 <= apex[scale] / apex[1.] <= 0.985, f'x{scale}: {apex[scale] / KPH:.1f} vs {apex[1.] / KPH:.1f} km/h dry'
    assert apex[self.HEAVY] < apex[self.RAIN] - 0.3 * KPH

  def test_rain_starting_mid_curve_lowers_the_target_gradually(self):
    # rain mode starts 30 m into a long R40 curve: the locked speed and the command come down without a step
    radius, start, length = 40., 150., 200.
    planner, log = drive(one_curve(radius, start, length), 45 * KPH, 30.,
                         comfort_scale=lambda s: self.HEAVY if s >= start + 30. else 1.)
    inside = [r for r in log if start + 10. <= r[0] <= start + length - 10.]
    targets = np.array([r[5] for r in inside])
    assert targets[-1] < targets[0] - 2 * KPH
    assert np.abs(np.diff(targets)).max() / DT_MDL <= CurveSpeedParams().target_rate + 1e-6
    cmds = np.array([r[3] for r in inside])
    assert np.abs(np.diff(cmds)).max() / DT_MDL <= CurveSpeedParams().jerk + 1e-6
    np.testing.assert_allclose(planner.comfort_scale, self.HEAVY)

  def test_scale_ramps_and_snaps_when_idle(self):
    p = CurveSpeedParams()
    planner = CurveSpeedPlanner(p, dt=DT_MDL)
    v = 60 * KPH
    scales = []
    for _ in range(int(5. / DT_MDL)):
      planner.update(v, 0., 0., *model_path(0., v, straight), comfort_scale=self.RAIN)
      scales.append(planner.comfort_scale)
    assert np.abs(np.diff([1.] + scales)).max() <= p.comfort_scale_rate * DT_MDL + 1e-9
    np.testing.assert_allclose(scales[-1], self.RAIN)
    planner.update(v, 0., 0., *model_path(0., v, straight), enabled=False, comfort_scale=1.)
    assert planner.comfort_scale == 1.
