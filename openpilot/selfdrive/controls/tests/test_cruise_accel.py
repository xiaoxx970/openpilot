from types import SimpleNamespace

import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, get_max_accel, limit_cruise_to_lead, \
                                                                  LEAD_CLOSE_MAX_SPEED, LEAD_CLOSE_MIN_ACCEL

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


class TestLimitCruiseToLead:
  def test_no_lead_keeps_set_speed(self):
    assert limit_cruise_to_lead(33., 24., lead(110., 24., present=False), Personality.relaxed) == (33., False)

  def test_far_lead_caps_closing_speed(self):
    # route 000001a2 segment 35: lead 110 m ahead at ~87 km/h, set speed well above
    v_lead = 87 * CV.KPH_TO_MS
    v_target, capped = limit_cruise_to_lead(120 * CV.KPH_TO_MS, v_lead, lead(110., v_lead), Personality.relaxed)
    assert capped
    np.testing.assert_allclose(v_target, v_lead + LEAD_CLOSE_MAX_SPEED)

  def test_closing_speed_shrinks_with_gap(self):
    v_lead = 24.
    targets = [limit_cruise_to_lead(33., v_lead, lead(d, v_lead), Personality.relaxed)[0] for d in (110., 70., 55., 40.)]
    assert targets[0] > targets[1] > targets[2] > targets[3] == v_lead

  def test_ego_above_cap_targets_lead_speed(self):
    # at or inside the desired distance there is no closing speed left
    assert limit_cruise_to_lead(33., 26., lead(30., 24.), Personality.standard) == (24., True)

  def test_lower_set_speed_still_wins(self):
    assert limit_cruise_to_lead(20., 25., lead(110., 24.), Personality.relaxed) == (20., False)
