from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, get_max_accel, LeadClosingLimit, \
                                                                  AccelDirectionHold, is_calm, ACCEL_HOLD_TIME, ACCEL_HOLD_OVERRIDE, ACCEL_HOLD_OVERRIDE_JERK, \
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
  return SimpleNamespace(present=present, dRel=d_rel, vLead=v_lead, aLeadK=0.)


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


def hold_run(hold, accels, calm=True):
  return [hold.update(a, calm) for a in accels]


class TestAccelDirectionHold:
  def test_small_flips_are_held(self):
    # the downhill set-speed case: the request wobbles +-0.05 around zero every second
    accels = [0.05 * np.sin(2 * np.pi * k * DT / 1.5) for k in range(int(4 / DT))]
    out = np.array(hold_run(AccelDirectionHold(DT), accels))
    turns = np.sum(np.diff(np.sign(np.diff(out[np.abs(np.diff(out, prepend=out[0])) > 1e-6]))) != 0)
    assert turns <= 1

  def test_reverses_after_hold_time(self):
    hold = AccelDirectionHold(DT)
    # up, then a turn down (the first turn after a reset is free) ...
    hold_run(hold, [0., 0.1, 0.05])
    # ... so turning back up is held for ACCEL_HOLD_TIME
    out = hold_run(hold, [0.1] * int((ACCEL_HOLD_TIME + 1) / DT))
    assert out[int((ACCEL_HOLD_TIME - 1) / DT)] == 0.05
    np.testing.assert_allclose(out[-1], 0.1)

  def test_large_change_passes_at_once_rate_limited(self):
    hold = AccelDirectionHold(DT)
    hold_run(hold, [0., 0.1])
    out = hold_run(hold, [0.1 - ACCEL_HOLD_OVERRIDE - 0.1] * int(2 / DT))
    assert out[1] < out[0] < 0.1
    assert np.max(np.abs(np.diff(out))) <= ACCEL_HOLD_OVERRIDE_JERK * DT + 1e-9
    np.testing.assert_allclose(out[-1], 0.1 - ACCEL_HOLD_OVERRIDE - 0.1)

  def test_not_calm_passes_through(self):
    hold = AccelDirectionHold(DT)
    hold_run(hold, [0., 0.1])
    assert hold.update(-1.0, calm=False) == -1.0


class TestIsCalm:
  def test_cruising_without_lead(self):
    assert is_calm(25., 0., lead(0., 0., present=False), Personality.standard, False, False)

  def test_not_calm(self):
    v = 25.
    d = desired_distance(v, Personality.standard)
    cases = [
      (3., lead(0., 0., present=False), False, False),                          # slow
      (v, lead(0., 0., present=False), True, False),                            # stopping
      (v, lead(0., 0., present=False), False, True),                            # fcw
      (v, lead(0.7 * d, v), False, False),                                      # too close
      (v, SimpleNamespace(present=True, dRel=d, vLead=v, aLeadK=-1.), False, False),  # lead braking
      (v, lead(30., v - 6.), False, False),                                     # 5 s to collision
    ]
    for v_ego, ld, stop, fcw in cases:
      assert not is_calm(v_ego, 0., ld, Personality.standard, stop, fcw)
    # a real braking request, e.g. the model stopping for a red light with no lead
    assert not is_calm(v, -0.8, lead(0., 0., present=False), Personality.standard, False, False)

  def test_steady_follow_is_calm(self):
    v = 25.
    assert is_calm(v, 0., lead(desired_distance(v, Personality.standard), v), Personality.standard, False, False)
