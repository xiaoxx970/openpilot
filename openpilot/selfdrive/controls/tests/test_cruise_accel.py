from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, get_max_accel, LeadClosingLimit, \
                                                                  LEAD_CLOSE_ACCEL_TIME, LEAD_CLOSE_MIN_SPEED, \
                                                                  LEAD_CLOSE_LOST_TIME, LEAD_CLOSE_MAX_SPEED, LEAD_CLOSE_MIN_ACCEL

Personality = log.LongitudinalPersonality
CP = SimpleNamespace(steerRatio=15., wheelbase=2.6)
DT = 0.05


def settled_cruise_accel(v_cruise, v_ego, personality, lead_capped=False):
  a = 0.
  for _ in range(200):
    a = get_cruise_accel(False, v_cruise, v_ego, a, 0., CP, DT, 2.0, True, personality, lead_capped)
  return a


def lead(d_rel, v_lead, present=True):
  return SimpleNamespace(present=present, dRel=d_rel, vLead=v_lead)


def desired_distance(v, personality):
  return get_safe_obstacle_distance(v, get_T_FOLLOW(personality)) - get_stopped_equivalence_factor(v)


def run(limit, v_cruise, v_ego, leads, personality=Personality.relaxed):
  return [limit.update(v_cruise, v_ego, ld, personality) for ld in leads]


class TestCruiseAccelLimits:
  def test_accel_scaled_by_personality(self):
    v_ego = 25.
    base = get_max_accel(v_ego)
    for personality, scale in ((Personality.relaxed, 0.75), (Personality.standard, 1.0), (Personality.aggressive, 1.0)):
      np.testing.assert_allclose(settled_cruise_accel(v_ego + 10., v_ego, personality), base * scale, atol=1e-3)

  def test_decel_scaled_by_personality(self):
    v_ego = 25.
    base = get_max_accel(v_ego)
    for personality, scale in ((Personality.relaxed, 0.75), (Personality.standard, 1.0), (Personality.aggressive, 1.25)):
      np.testing.assert_allclose(settled_cruise_accel(v_ego - 10., v_ego, personality), -base * scale, atol=1e-3)

  def test_lead_cap_only_eases_off(self):
    # slowing toward the lead cap is gentle, actual braking for the lead is the MPC's job
    for personality in (Personality.relaxed, Personality.standard, Personality.aggressive):
      np.testing.assert_allclose(settled_cruise_accel(15., 25., personality, lead_capped=True), LEAD_CLOSE_MIN_ACCEL, atol=1e-3)

  def test_lead_cap_tracks_softly(self):
    # a small speed error toward the lead cap gives a proportionally small command, not cruise's 1 s gain
    np.testing.assert_allclose(settled_cruise_accel(25.4, 25., Personality.standard, lead_capped=True), 0.4 / LEAD_CLOSE_ACCEL_TIME, atol=1e-3)


class TestLeadClosingLimit:
  def test_no_lead_keeps_set_speed(self):
    assert LeadClosingLimit(DT).update(33., 24., lead(110., 24., present=False), Personality.relaxed) == (33., False)

  def test_far_lead_caps_closing_speed(self):
    # route 000001a2 segment 35: lead 110 m ahead at ~87 km/h, set speed well above
    v_lead = 87 * CV.KPH_TO_MS
    v_target, capped = run(LeadClosingLimit(DT), 120 * CV.KPH_TO_MS, v_lead, [lead(110., v_lead)] * 200)[-1]
    assert capped
    np.testing.assert_allclose(v_target, v_lead + LEAD_CLOSE_MAX_SPEED, atol=1e-3)

  def test_closing_speed_shrinks_with_gap(self):
    v = 25.
    targets = [run(LeadClosingLimit(DT), 33., v, [lead(d, v)])[-1][0] for d in (75., 65., 58.)]
    assert targets[0] > targets[1] > targets[2] > v + LEAD_CLOSE_MIN_SPEED

  def test_steady_following_left_to_mpc(self):
    # at or inside the desired distance cruise still allows a little closing speed, so the MPC is what binds
    v = 25.
    for d in (desired_distance(v, Personality.relaxed), 20.):
      np.testing.assert_allclose(run(LeadClosingLimit(DT), 33., v, [lead(d, v)])[-1][0], v + LEAD_CLOSE_MIN_SPEED)

  def test_lead_speed_noise_filtered(self):
    v = 25.
    leads = [lead(120., v + (0.5 if k % 2 else -0.5)) for k in range(400)]
    targets = np.array([t for t, _ in run(LeadClosingLimit(DT), 33., v, leads)])
    assert np.std(targets[200:]) < 0.05

  def test_brief_lead_dropout_keeps_cap(self):
    v = 25.
    limit = LeadClosingLimit(DT)
    run(limit, 33., v, [lead(120., v)] * 100)
    dropout = int(LEAD_CLOSE_LOST_TIME / DT) - 2
    assert all(t == 33. for t, _ in run(limit, 33., v, [lead(0., 0., present=False)] * dropout))
    assert limit.active
    run(limit, 33., v, [lead(0., 0., present=False)] * 5)
    assert not limit.active

  def test_lower_set_speed_still_wins(self):
    assert run(LeadClosingLimit(DT), 20., 25., [lead(110., 24.)])[-1] == (20., False)


import unittest

from openpilot.selfdrive.controls.lib.longitudinal_planner import CoastBandHold, COAST_BAND_HOLD_TIME


class TestCoastBandHold(unittest.TestCase):
  """Flat-road prior: coast estimate -0.3, so throttle-off is -0.3 and brake-on -0.6."""
  COAST = -0.3

  def feed(self, hold, cmds, active=True, idle=False, brake=False):
    return [hold.update(c, self.COAST, active, idle, brake) for c in cmds]

  def test_throttle_side_is_untouched(self):
    # holding a set speed on the flat: the command moves between two throttle levels and never leaves that state
    hold = CoastBandHold(DT)
    cmds = ([0.24] * 100 + [-0.18] * 100) * 3
    self.assertEqual(self.feed(hold, cmds), cmds)

  def test_throttle_coast_flip_is_held(self):
    # downhill: the loop lifts the throttle and reapplies it every second; the second change rests on the edge
    hold = CoastBandHold(DT)
    self.feed(hold, [-0.1] * 20)
    self.assertEqual(self.feed(hold, [-0.35] * 20)[-1], -0.35)   # lifting passes
    out = self.feed(hold, [-0.1] * 40)                            # reapplying 1 s later: blocked, stays coasting
    self.assertAlmostEqual(out[-1], self.COAST - 0.02)
    self.assertTrue(all(o < -0.1 for o in out))
    self.assertEqual(self.feed(hold, [-0.35] * 20)[-1], -0.35)   # coasting again is no change of state

  def test_throttle_to_brake_and_back(self):
    hold = CoastBandHold(DT)
    self.feed(hold, [0.0] * 20)
    self.assertEqual(self.feed(hold, [-0.65] * 20)[-1], -0.65)   # first change, straight into braking, passes
    out = self.feed(hold, [-0.05] * 40)                           # back toward throttle 1 s later: released to coasting only
    self.assertAlmostEqual(out[-1], self.COAST - 0.02)
    out = self.feed(hold, [-0.65] * 20)                           # braking again 2 s on is another change: held at the brake edge
    self.assertAlmostEqual(out[-1], self.COAST - 0.3 + 0.02)
    self.feed(hold, [-0.65] * int(COAST_BAND_HOLD_TIME / DT))
    self.assertEqual(self.feed(hold, [-0.65] * 5)[-1], -0.65)    # and passes once the hold time is over

  def test_change_passes_after_the_hold_time(self):
    hold = CoastBandHold(DT)
    self.feed(hold, [-0.1] * 20)
    self.feed(hold, [-0.35] * int(COAST_BAND_HOLD_TIME / DT + 2))
    self.assertEqual(self.feed(hold, [-0.1] * 40)[-1], -0.1)

  def test_real_demand_overrides_a_hold(self):
    hold = CoastBandHold(DT)
    self.feed(hold, [0.0] * 20)
    self.feed(hold, [-0.65] * 20)
    self.feed(hold, [-0.05] * 40)                                 # held in coasting
    self.assertEqual(self.feed(hold, [1.0] * 20)[-1], 1.0)        # far past the edge: through within 1 s
    hold2 = CoastBandHold(DT)
    self.feed(hold2, [-0.35] * 20)
    self.feed(hold2, [-0.1] * 20)
    self.assertEqual(self.feed(hold2, [-1.5] * 20)[-1], -1.5)     # a hard brake request is never held

  def test_lead_and_e2e_requests_bypass(self):
    hold = CoastBandHold(DT)
    cmds = ([0.1] * 10 + [-0.7] * 10) * 5
    self.assertEqual(self.feed(hold, cmds, active=False), cmds)

  def test_edges_learn_from_the_car(self):
    hold = CoastBandHold(DT)
    self.feed(hold, [-0.45] * 20)                                 # throttle still on at -0.45 for 1 s
    self.feed(hold, [-0.45] * 2, idle=True)                       # then the engine goes idle: throttle-off is near -0.45
    lo, hi = hold.edges(self.COAST)
    self.assertAlmostEqual(hi, -0.33)                             # one observation moves the edge a fifth of the way
    self.feed(hold, [-0.75] * 20, idle=True)
    self.feed(hold, [-0.75] * 2, idle=True, brake=True)           # ACC starts braking at -0.75
    lo2, hi2 = hold.edges(self.COAST)
    self.assertAlmostEqual(hi2, hi)
    self.assertLess(lo2, lo)                                      # the band widened toward the observed brake onset

  def test_outlier_edges_are_ignored(self):
    hold = CoastBandHold(DT)
    self.feed(hold, [0.6] * 20)
    self.feed(hold, [0.6] * 2, idle=True)                         # idle while accelerating hard: a gear shift, not the coast line
    self.assertEqual(hold.edges(self.COAST), (-0.6, -0.3))


class TestCoastBandCalibration(unittest.TestCase):
  COAST = -0.3

  def learn(self, hold, at):
    hold.update(at, self.COAST, True, False, False)
    for _ in range(20):
      hold.update(at, self.COAST, True, False, False)
    hold.update(at, self.COAST, True, True, False)
    for _ in range(20):
      hold.update(at, self.COAST, True, True, False)
    for _ in range(20):
      hold.update(0.2, self.COAST, True, False, False)

  def test_starts_from_half_of_the_saved_band(self):
    hold = CoastBandHold(DT, {"offset": -0.2, "width": 0.5, "edges": 30})
    self.assertAlmostEqual(hold.coast_offset, -0.1)
    self.assertAlmostEqual(hold.band_width, 0.4)
    self.assertEqual(hold.learn_count, 0)

  def test_bad_saved_values_are_ignored(self):
    for saved in (None, {}, {"offset": "x", "width": 0.3}, {"offset": 1.5, "width": 0.3}, {"offset": 0., "width": 3.}):
      hold = CoastBandHold(DT, saved)
      self.assertEqual((hold.coast_offset, hold.band_width), (0., 0.3))

  def test_calibration_counts_accepted_edges(self):
    hold = CoastBandHold(DT)
    self.learn(hold, -0.45)
    self.learn(hold, -0.45)
    cal = hold.calibration()
    self.assertEqual(cal["edges"], 2)
    self.assertLess(cal["offset"], 0.)
    again = CoastBandHold(DT, cal)
    self.assertAlmostEqual(again.coast_offset, 0.5 * cal["offset"], places=4)

  def test_reset_keeps_the_calibration(self):
    hold = CoastBandHold(DT)
    self.learn(hold, -0.45)
    before = hold.calibration()
    hold.reset()
    self.assertEqual(hold.calibration(), before)
