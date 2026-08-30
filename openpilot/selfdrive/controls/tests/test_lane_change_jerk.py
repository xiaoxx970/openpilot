import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import (LANE_CHANGE_JERK_RAMP_T, LANE_CHANGE_START_JERK,
                                                            MAX_LATERAL_JERK, clip_curvature,
                                                            lane_change_lateral_jerk)


class TestLaneChangeLateralJerk:
  def test_ramps_from_the_start_value_to_the_normal_limit(self):
    assert lane_change_lateral_jerk(0.0) == LANE_CHANGE_START_JERK
    assert lane_change_lateral_jerk(LANE_CHANGE_JERK_RAMP_T) == MAX_LATERAL_JERK
    assert lane_change_lateral_jerk(LANE_CHANGE_JERK_RAMP_T * 10) == MAX_LATERAL_JERK
    # never exceeds the normal limit, and never runs backwards
    ts = np.arange(0.0, LANE_CHANGE_JERK_RAMP_T * 2, DT_CTRL)
    jerks = [lane_change_lateral_jerk(t) for t in ts]
    assert max(jerks) <= MAX_LATERAL_JERK
    assert all(b >= a for a, b in zip(jerks, jerks[1:], strict=False))

  def test_softened_start_is_slower_but_reaches_the_same_curvature(self):
    # a step demand at highway speed, the shape the model produces when a lane change starts
    v_ego, target = 36.0, 0.0011

    def run(soften):
      curvature, t = 0.0, 0.0
      trace = []
      for _ in range(int(2.0 / DT_CTRL)):
        jerk = lane_change_lateral_jerk(t) if soften else MAX_LATERAL_JERK
        curvature, _limited = clip_curvature(v_ego, curvature, target, 0.0, jerk)
        t += DT_CTRL
        trace.append(curvature * v_ego ** 2)
      return trace

    def peak_jerk(trace):
      return max(abs(b - a) / DT_CTRL for a, b in zip(trace, trace[1:], strict=False))

    hard, soft = run(False), run(True)

    assert peak_jerk(soft) < peak_jerk(hard)
    assert peak_jerk(soft) <= MAX_LATERAL_JERK + 1e-6
    # softening only delays the entry, it does not give up any of the manoeuvre
    assert soft[-1] == hard[-1]
    def reach(trace):
      return next(i for i, a in enumerate(trace) if abs(a) >= 0.9 * abs(trace[-1]))

    assert reach(soft) > reach(hard)

  def test_default_argument_leaves_normal_driving_unchanged(self):
    v_ego = 30.0
    for target in (-0.01, -0.001, 0.0, 0.001, 0.01):
      explicit, _a = clip_curvature(v_ego, 0.0, target, 0.0, MAX_LATERAL_JERK)
      default, _b = clip_curvature(v_ego, 0.0, target, 0.0)
      assert explicit == default
