import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.curve_speed import CurveSpeedPlanner, CurveSpeedParams
from openpilot.common.test import OpenpilotTestCase

KPH = 1 / 3.6


def road_curvature(s, radius, start, length):
  """|curvature| of a road with one constant-radius curve from `start` to `start + length` (m)."""
  return np.where((s >= start) & (s < start + length), 1 / radius, 0.)


def one_curve(radius, start, length):
  return lambda s: road_curvature(s, radius, start, length)


def straight(s):
  return np.zeros_like(s)


def model_path(s_car, v, curvature_fn):
  """A model path that follows the road exactly: positions and headings at the model's time samples."""
  s = s_car + max(v, 1.) * np.array(ModelConstants.T_IDXS)
  ds = np.diff(s, prepend=s[0])
  heading = np.cumsum(curvature_fn(s) * ds)
  x = np.cumsum(np.cos(heading) * ds)
  y = np.cumsum(np.sin(heading) * ds)
  return x, y, heading


def drive(curvature_fn, v0, duration, lane_change=lambda s: False, kappa_meas_fn=None):
  """Close the loop on a point-mass car: the planner's command when it acts, else hold the speed."""
  planner = CurveSpeedPlanner(dt=DT_MDL)
  v, a, s = v0, 0., 0.
  log = []
  for _ in range(int(duration / DT_MDL)):
    kappa = float(kappa_meas_fn(s) if kappa_meas_fn else curvature_fn(np.array([s]))[0])
    _, a_cmd = planner.update(v, a, kappa, *model_path(s, v, curvature_fn), lane_change=lane_change(s))
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

  def test_high_speed_respects_the_steering_cap(self):
    p = CurveSpeedParams()
    planner = CurveSpeedPlanner(p, dt=DT_MDL)
    v = 120 * KPH
    reachable = p.cap_margin * p.cmd_lat_max * float(np.interp(v, p.yaw_ratio_bp, p.yaw_ratio_v))
    assert float(planner.lat_limit(v)) <= reachable + 1e-9
