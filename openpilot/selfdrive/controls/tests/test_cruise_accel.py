import unittest
from types import SimpleNamespace

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX

from openpilot.cereal import log, messaging
from openpilot.common.constants import CV
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor, \
                                                                       get_rain_T_FOLLOW, STOP_DISTANCE
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, get_max_decel, LeadClosingLimit, J_CRUISE_APPROACH, \
                                                                  CRUISE_APPROACH_LOOKAHEAD, \
                                                                  CoastBandHold, COAST_BAND_HOLD_TIME, PullAwayAssist, PULL_AWAY_JERK, \
                                                                  PULL_AWAY_RELEASE_JERK, PULL_AWAY_MAX_ACCEL, \
                                                                  LEAD_CLOSE_ACCEL_TIME, LEAD_CLOSE_MIN_SPEED, \
                                                                  LEAD_CLOSE_LOST_TIME, LEAD_CLOSE_MAX_SPEED, LEAD_CLOSE_MIN_ACCEL, \
                                                                  LEAD_CLOSE_STOP_DECEL, LEAD_CLOSE_TIME, LEAD_CLOSE_RISE_TAU, \
                                                                  LEAD_CLOSE_FILTER_TAU, E2EDeadband, E2E_DEADBAND_MARGIN, \
                                                                  RainAccelCap, RAIN_ACCEL_CAP, RAIN_ACCEL_CAP_RATE

Personality = log.LongitudinalPersonality
DT = 0.05


def settled_cruise_accel(v_cruise, v_ego, personality, lead_capped=False):
  a = 0.
  for _ in range(200):
    a = get_cruise_accel(v_cruise, v_ego, a, DT, personality, lead_capped)
  return a


def lead(d_rel, v_lead, present=True, a_lead=0.):
  return SimpleNamespace(present=present, dRel=d_rel, vLead=v_lead, aLeadK=a_lead)


def desired_distance(v, personality):
  return get_safe_obstacle_distance(v, get_T_FOLLOW(personality)) - get_stopped_equivalence_factor(v)


def run(limit, v_cruise, v_ego, leads, personality=Personality.relaxed):
  return [limit.update(v_cruise, v_ego, ld, personality) for ld in leads]


class TestCruiseAccelLimits(unittest.TestCase):
  def test_accel_left_to_e2e(self):
    # far below the set speed, speeding up is shaped by the always-present e2e candidate, cruise only caps it at ACCEL_MAX
    for personality in (Personality.relaxed, Personality.standard, Personality.aggressive):
      np.testing.assert_allclose(settled_cruise_accel(55., 25., personality), ACCEL_MAX, atol=1e-3)

  def test_approach_tapers_by_personality(self):
    # 3 km/h below the set speed: accel already easing off, gentlest for relaxed
    v_error = 3. * CV.KPH_TO_MS
    accels = [settled_cruise_accel(25. + v_error, 25., p) for p in (Personality.relaxed, Personality.standard, Personality.aggressive)]
    # a = sqrt(2 * J * (v_error - a * lookahead)): the speed still gained through the car's lag counts as gained
    j_lookahead = np.array(J_CRUISE_APPROACH) * CRUISE_APPROACH_LOOKAHEAD
    np.testing.assert_allclose(accels, -j_lookahead + np.sqrt(j_lookahead ** 2 + 2. * np.array(J_CRUISE_APPROACH) * v_error), atol=1e-3)
    assert accels[0] < accels[1] < accels[2] < v_error

  def test_approach_through_the_car_lag(self):
    # 2026-10-28 route 228: aggressive, set 40 km/h, the model asking ~0.9 m/s^2 all the way up. Through the car's 0.15 s
    # delay and 0.3 s lag the car itself must already be easing off 2 km/h below the set speed, without a sharp drop or overshoot
    dt, delay, lag, a_model = 0.01, 0.15, 0.3, 0.9
    v_set = 40. * CV.KPH_TO_MS
    v, a_car, a_cruise, a_cmd = 25. * CV.KPH_TO_MS, 0.85, 0.85, 0.85
    sent = [a_cmd] * int(delay / dt + 1)
    trace = []
    for k in range(int(20. / dt)):
      if k % int(DT / dt) == 0:
        a_cruise = get_cruise_accel(v_set, v, a_cruise, DT, Personality.aggressive)
        a_cmd = min(a_cruise, a_model)
      sent.append(a_cmd)
      a_car += (sent[-1 - int(delay / dt)] - a_car) * dt / lag
      v += a_car * dt
      trace.append((v, a_car))
    v_tr, a_tr = np.array(trace).T
    i_peak = int(np.argmax(v_tr))
    self.assertLess(np.interp(v_set - 2. * CV.KPH_TO_MS, v_tr[:i_peak + 1], a_tr[:i_peak + 1]), 0.55)
    self.assertLess(np.max(-np.diff(a_tr)) / dt, 0.35)
    self.assertLess((v_tr.max() - v_set) * CV.MS_TO_KPH, 0.3)

  def test_approach_ramp_jerk(self):
    # driving up to the set speed from 15 km/h below, the command winds down at the approach jerk; where the taper hands
    # over to the 1 s gain for the last fraction of a km/h the jerk briefly doubles, at a command of only 2 * J
    for personality, j_approach in zip((Personality.relaxed, Personality.standard, Personality.aggressive), J_CRUISE_APPROACH, strict=True):
      v, a, accels = 25., 0., []
      v_cruise = v + 15. * CV.KPH_TO_MS
      for _ in range(int(30. / DT)):
        a = get_cruise_accel(v_cruise, v, a, DT, personality)
        v += a * DT
        accels.append(a)
      falling = -np.diff(accels[np.argmax(accels):]) / DT
      assert falling.max() < 2. * j_approach * 1.05
      assert np.median(falling[falling > 1e-3]) < j_approach * 1.05
      assert v_cruise - v < 0.1 and v <= v_cruise

  def test_personality_from_a_message(self):
    # the planner gets the enum read back from selfdriveState, which must hit the same values as the int constants
    msg = messaging.new_message('selfdriveState')
    msg.selfdriveState.personality = Personality.relaxed
    personality = messaging.log_from_bytes(msg.to_bytes()).selfdriveState.personality
    np.testing.assert_allclose(settled_cruise_accel(15., 25., personality), -get_max_decel(25.) * 0.75, atol=1e-3)

  def test_decel_scaled_by_personality(self):
    v_ego = 25.
    base = get_max_decel(v_ego)
    for personality, scale in ((Personality.relaxed, 0.75), (Personality.standard, 1.0), (Personality.aggressive, 1.25)):
      np.testing.assert_allclose(settled_cruise_accel(v_ego - 10., v_ego, personality), -base * scale, atol=1e-3)

  def test_lead_cap_only_eases_off(self):
    # slowing toward the lead cap is gentle, actual braking for the lead is the MPC's job
    for personality in (Personality.relaxed, Personality.standard, Personality.aggressive):
      np.testing.assert_allclose(settled_cruise_accel(15., 25., personality, lead_capped=True), LEAD_CLOSE_MIN_ACCEL, atol=1e-3)

  def test_lead_cap_eased_toward_from_above(self):
    # a small excess over the lead cap gives a proportionally small slow-down, not cruise's 1 s gain
    np.testing.assert_allclose(settled_cruise_accel(24.6, 25., Personality.standard, lead_capped=True), -0.4 / LEAD_CLOSE_ACCEL_TIME, atol=1e-3)

  def test_lead_cap_approached_without_the_taper(self):
    # below the lead cap, which a lead pulling away keeps raising, cruise speeds up at its 1 s gain; the set-speed taper
    # held launches behind a lead driving off to 0.7-0.9 m/s^2 (2026-10-28 route 228)
    for personality in (Personality.relaxed, Personality.standard, Personality.aggressive):
      for v_cap in (25.4, 27., 30.):
        a = settled_cruise_accel(v_cap, 25., personality, lead_capped=True)
        np.testing.assert_allclose(a, min(v_cap - 25., ACCEL_MAX), atol=1e-3)
        assert a > settled_cruise_accel(v_cap, 25., personality)


class TestLeadClosingLimit(unittest.TestCase):
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
    # and not rectified upward by the faster rise
    assert abs(np.mean(targets[200:]) - (v + LEAD_CLOSE_MAX_SPEED)) < 0.05

  def test_cap_keeps_up_with_a_lead_driving_off(self):
    # 2026-10-28 route 228: through the 2 s low-pass the cap trailed a lead driving off from a stop by ~5 km/h
    a_lead, t = 1.2, 3.
    n = int(t / DT)
    for a_lead_k, max_lag in ((a_lead, 1.1 * a_lead * LEAD_CLOSE_RISE_TAU), (0., None)):
      leads = [lead(5., a_lead * k * DT, a_lead=a_lead_k) for k in range(n + 1)]
      v_target, capped = run(LeadClosingLimit(DT), 50 * CV.KPH_TO_MS, 0., leads)[-1]
      assert capped
      lag = a_lead * t + LEAD_CLOSE_MIN_SPEED - v_target
      if max_lag is not None:
        assert lag < max_lag
      else:
        # a lead the radar does not see speeding up keeps the slow low-pass
        assert lag > 0.7 * a_lead * LEAD_CLOSE_FILTER_TAU

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

  def test_stopped_lead_closed_on_at_a_stoppable_speed(self):
    # 2026-10-26 19:54: the gap law held cruise at 1-2 km/h for the last metres up to a car stopped at a red light
    v_ego, d = 1.5, 15.
    v_target, capped = run(LeadClosingLimit(DT), 50 * CV.KPH_TO_MS, v_ego, [lead(d, 0.)] * 200)[-1]
    assert capped
    np.testing.assert_allclose(v_target, np.sqrt(2 * LEAD_CLOSE_STOP_DECEL * (d - STOP_DISTANCE)), atol=1e-3)
    assert v_target > 4 * LEAD_CLOSE_MIN_SPEED

  def test_stopped_lead_at_the_stopping_point_keeps_the_floor(self):
    v_target, _ = run(LeadClosingLimit(DT), 50 * CV.KPH_TO_MS, 0.5, [lead(STOP_DISTANCE - 0.5, 0.)] * 200)[-1]
    np.testing.assert_allclose(v_target, LEAD_CLOSE_MIN_SPEED, atol=1e-3)

  def test_moving_lead_keeps_the_gap_law(self):
    v_ego, v_lead, d = 8., 5., 30.
    gap = d - desired_distance(v_ego, Personality.relaxed) - (get_stopped_equivalence_factor(v_ego) - get_stopped_equivalence_factor(v_lead))
    expected = v_lead + np.clip(gap / LEAD_CLOSE_TIME, LEAD_CLOSE_MIN_SPEED, LEAD_CLOSE_MAX_SPEED)
    v_target, _ = run(LeadClosingLimit(DT), 50 * CV.KPH_TO_MS, v_ego, [lead(d, v_lead)] * 200)[-1]
    np.testing.assert_allclose(v_target, expected, atol=1e-3)


def radar_lead(d_rel, v_lead, a_lead, radar=True):
  return SimpleNamespace(present=True, radar=radar, dRel=d_rel, vLead=v_lead, aLeadK=a_lead)


class TestPullAwayAssist(unittest.TestCase):
  V = 15.
  GAP = desired_distance(15., Personality.standard) + 10.  # behind the desired distance

  def feed(self, assist, n, a_e2e=0.1, a_mpc=1.5, ld=None, v=V, personality=Personality.standard):
    ld = ld or radar_lead(self.GAP, self.V + 0.5, 0.5)
    return [assist.update(a_e2e, a_mpc, ld, v, personality) for _ in range(n)]

  def test_follows_a_lead_pulling_away(self):
    # lead 0.5 m/s faster and speeding up at 0.5: the command may reach 0.5 + 0.5 * 0.5 above a model asking 0.1
    out = self.feed(PullAwayAssist(DT), int(4 / DT))
    self.assertAlmostEqual(out[-1], 0.75, places=2)
    self.assertLessEqual(max(np.diff(out)) / DT, PULL_AWAY_JERK + 1e-6)

  def test_never_above_the_mpc(self):
    self.assertAlmostEqual(self.feed(PullAwayAssist(DT), int(4 / DT), a_mpc=0.3)[-1], 0.3)

  def test_model_asking_more_is_untouched(self):
    self.assertAlmostEqual(self.feed(PullAwayAssist(DT), int(4 / DT), a_e2e=1.2)[-1], 1.2)

  def test_model_slowing_vetoes_at_once(self):
    assist = PullAwayAssist(DT)
    self.feed(assist, int(4 / DT))
    self.assertEqual(self.feed(assist, 1, a_e2e=-0.2)[-1], -0.2)
    self.assertLess(self.feed(assist, 1, a_e2e=-0.1)[-1], 0.35)    # halfway to the veto, half the lift
    self.assertGreater(self.feed(assist, 1, a_e2e=0.)[-1], 0.5)    # the allowance itself is kept for when the model lets go

  def test_releases_gently_when_the_lead_stops_pulling_away(self):
    assist = PullAwayAssist(DT)
    self.feed(assist, int(4 / DT))
    out = self.feed(assist, int(2 / DT), ld=radar_lead(self.GAP, self.V, 0.))
    self.assertAlmostEqual(out[-1], 0.1)
    self.assertLessEqual(max(-np.diff(out)) / DT, PULL_AWAY_RELEASE_JERK + 1e-6)

  def test_only_while_pulling_away(self):
    gap, v_lead = self.GAP, self.V + 0.5
    inside = get_safe_obstacle_distance(self.V, get_T_FOLLOW(Personality.standard)) - get_stopped_equivalence_factor(v_lead) - 1.
    cases = {
      'lead not speeding up': {'ld': radar_lead(gap, v_lead, 0.1)},
      'lead slower than us': {'ld': radar_lead(gap, self.V - 1., 0.5)},
      'inside the desired distance': {'ld': radar_lead(inside, v_lead, 0.5)},
      'vision-only lead': {'ld': radar_lead(gap, v_lead, 0.5, radar=False)},
      'launching from a stop': {'v': 2.},
    }
    for name, kw in cases.items():
      with self.subTest(name):
        self.assertAlmostEqual(self.feed(PullAwayAssist(DT), int(4 / DT), **kw)[-1], 0.1)

  def test_capped_by_personality(self):
    for personality, cap in zip((Personality.relaxed, Personality.standard, Personality.aggressive), PULL_AWAY_MAX_ACCEL, strict=True):
      out = self.feed(PullAwayAssist(DT), int(4 / DT), ld=radar_lead(self.GAP, self.V + 2., 2.), personality=personality)
      self.assertAlmostEqual(out[-1], cap, places=3)

  def test_rain_following_distance_moves_the_threshold(self):
    # a lead beyond the dry desired distance but inside the rain one is only followed without the rain time gap
    t_rain = get_rain_T_FOLLOW(self.V)
    v_lead = self.V + 0.5
    dry = get_safe_obstacle_distance(self.V, get_T_FOLLOW(Personality.standard)) - get_stopped_equivalence_factor(v_lead)
    rain = get_safe_obstacle_distance(self.V, t_rain) - get_stopped_equivalence_factor(v_lead)
    ld = radar_lead((dry + rain) / 2, v_lead, 0.5)
    self.assertGreater(self.feed(PullAwayAssist(DT), int(4 / DT), ld=ld)[-1], 0.5)
    assist = PullAwayAssist(DT)
    out = [assist.update(0.1, 1.5, ld, self.V, Personality.standard, t_rain) for _ in range(int(4 / DT))]
    self.assertAlmostEqual(out[-1], 0.1)


class TestE2EDeadband(unittest.TestCase):
  V = 22.  # m/s, 80 km/h
  A_PLAN = 0.3  # the lead-aware plan (lower of MPC and cruise)

  @classmethod
  def model(cls, should_stop=False, hard_brake=False, lead_x=None, lat_accel=0.):
    n = len(ModelConstants.T_IDXS)
    return SimpleNamespace(action=SimpleNamespace(shouldStop=should_stop), meta=SimpleNamespace(hardBrakePredicted=hard_brake),
                           leadsV3=[SimpleNamespace(prob=0.9, x=[lead_x])] if lead_x is not None else [],
                           orientationRate=SimpleNamespace(z=[lat_accel / cls.V] * n), velocity=SimpleNamespace(x=[cls.V] * n))

  @classmethod
  def radar_lead(cls, d_rel, a_lead=0., radar=True):
    return SimpleNamespace(present=True, radar=radar, dRel=d_rel, vLead=cls.V, aLeadK=a_lead)

  def far_lead(self, personality, a_lead=0.):
    return self.radar_lead(2. * desired_distance(self.V, personality), a_lead)

  def follow(self, a_e2e, lead, model, personality, seconds=5., deadband=None):
    deadband = deadband or E2EDeadband(DT)
    out = a_e2e
    for _ in range(int(seconds / DT)):
      out = deadband.update(a_e2e, self.A_PLAN, lead, model, self.V, personality, get_T_FOLLOW(personality))
    return out, deadband

  def test_far_lead_margin_by_personality(self):
    # the model asks 0.4 below the plan for a lead twice the following distance ahead (the 10-25 bookmark)
    for personality, margin in zip((Personality.relaxed, Personality.standard, Personality.aggressive), E2E_DEADBAND_MARGIN, strict=True):
      out, _ = self.follow(self.A_PLAN - 0.4, self.far_lead(personality), self.model(), personality)
      np.testing.assert_allclose(out, self.A_PLAN - max(0., 0.4 - margin), atol=1e-2)

  def test_strong_request_passes_beyond_the_margin(self):
    out, _ = self.follow(self.A_PLAN - 1.5, self.far_lead(Personality.aggressive), self.model(), Personality.aggressive)
    np.testing.assert_allclose(out, self.A_PLAN - 1.0, atol=1e-2)

  def test_model_above_the_plan_untouched(self):
    out, _ = self.follow(0.8, self.far_lead(Personality.aggressive), self.model(), Personality.aggressive)
    self.assertEqual(out, 0.8)

  def test_margin_fades_in(self):
    deadband = E2EDeadband(DT)
    lead = self.far_lead(Personality.aggressive)
    outs = [deadband.update(-0.1, self.A_PLAN, lead, self.model(), self.V, Personality.aggressive, get_T_FOLLOW(Personality.aggressive))
            for _ in range(int(2. / DT))]
    self.assertLess(outs[0], -0.05)  # barely touched on the first frame
    self.assertLess(np.abs(np.diff(outs)).max(), 0.03)  # never switched in one step

  def test_stop_and_hard_brake_pass_at_once(self):
    lead = self.far_lead(Personality.aggressive)
    for model in (self.model(should_stop=True), self.model(hard_brake=True)):
      _, deadband = self.follow(-0.1, lead, self.model(), Personality.aggressive)
      out, deadband = self.follow(-0.1, lead, model, Personality.aggressive, seconds=DT, deadband=deadband)
      self.assertEqual(out, -0.1)
      self.assertEqual(deadband.weight, 0.)

  def test_cut_in_and_curve_hand_back_to_the_model(self):
    lead = self.far_lead(Personality.aggressive)
    for model in (self.model(lead_x=lead.dRel / 3.), self.model(lat_accel=1.5)):
      _, deadband = self.follow(-0.1, lead, self.model(), Personality.aggressive)
      out, _ = self.follow(-0.1, lead, model, Personality.aggressive, seconds=1.5, deadband=deadband)
      np.testing.assert_allclose(out, -0.1, atol=1e-2)

  def test_lead_braking_near_the_following_distance(self):
    # at the following distance a lead braking at 1 m/s^2 gets the model's early reaction; far ahead the margin stays
    near = self.radar_lead(desired_distance(self.V, Personality.aggressive), a_lead=-1.)
    out, _ = self.follow(-0.6, near, self.model(), Personality.aggressive)
    np.testing.assert_allclose(out, -0.6, atol=1e-3)
    out, _ = self.follow(-0.6, self.far_lead(Personality.aggressive, a_lead=-1.), self.model(), Personality.aggressive)
    np.testing.assert_allclose(out, self.A_PLAN - 0.4, atol=1e-2)

  def test_vision_lead_and_no_lead_left_to_the_model(self):
    for lead in (self.radar_lead(60., radar=False), SimpleNamespace(present=False, radar=False, dRel=0., vLead=0., aLeadK=0.)):
      out, _ = self.follow(-0.1, lead, self.model(), Personality.aggressive)
      np.testing.assert_allclose(out, -0.1, atol=1e-9)


def rain_state(rain_mode, rain_level=1):
  return SimpleNamespace(rainMode=rain_mode, rainLevel=rain_level)


class TestRainAccelCap(unittest.TestCase):
  def feed(self, cap, seconds, rain_mode, rain_level=1, snap=False):
    return [cap.update(rain_state(rain_mode, rain_level), snap) for _ in range(int(round(seconds / DT)))]

  def started(self, rain_mode=False, rain_level=1):
    cap = RainAccelCap(DT)
    self.feed(cap, 1., rain_mode, rain_level)
    return cap

  def test_dry_is_uncapped(self):
    self.assertEqual(self.feed(RainAccelCap(DT), 5., False)[-1], ACCEL_MAX)

  def test_rain_and_heavy_rain(self):
    for rain_level, expected in ((1, RAIN_ACCEL_CAP[0]), (2, RAIN_ACCEL_CAP[1])):
      self.assertAlmostEqual(self.feed(self.started(), 5., True, rain_level)[-1], expected)

  def test_rain_level_without_rain_mode(self):
    # stepped off the rain following distance during rain: no cap, as no rain distance
    self.assertEqual(self.feed(RainAccelCap(DT), 5., False, rain_level=2)[-1], ACCEL_MAX)

  def test_ramps_in_and_out(self):
    cap = self.started()
    caps = [cap.cap] + self.feed(cap, 5., True) + self.feed(cap, 1., True, rain_level=2) + self.feed(cap, 5., False)
    self.assertLessEqual(np.abs(np.diff(caps)).max(), RAIN_ACCEL_CAP_RATE * DT + 1e-9)
    ramp = int(np.ceil((ACCEL_MAX - RAIN_ACCEL_CAP[0]) / (RAIN_ACCEL_CAP_RATE * DT)))
    self.assertAlmostEqual(caps[ramp], RAIN_ACCEL_CAP[0])
    self.assertGreater(caps[ramp - 1], RAIN_ACCEL_CAP[0])
    self.assertEqual(caps[-1], ACCEL_MAX)

  def test_follows_at_once_when_nothing_is_commanded(self):
    # first update, or openpilot not in control
    self.assertEqual(self.feed(RainAccelCap(DT), DT, True, rain_level=2)[-1], RAIN_ACCEL_CAP[1])
    self.assertEqual(self.feed(self.started(), DT, True, snap=True)[-1], RAIN_ACCEL_CAP[0])


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
