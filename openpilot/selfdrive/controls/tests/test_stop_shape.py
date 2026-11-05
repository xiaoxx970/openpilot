from types import SimpleNamespace as NS

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS, model_lead_matches, blend_model_lead
from openpilot.selfdrive.controls.lib.longitudinal_planner import MODEL_STOP_MARGIN, model_stops_short
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA

LEAD_T = np.array([0., 2., 4., 6., 8., 10.])


def radar_lead(d, v, a=0., prob=0.99):
  return NS(present=True, dRel=d, vLead=v, aLeadK=a, aLeadTau=1.5, modelProb=prob)


def model_lead(d, v, decel):
  # a lead braking at a constant rate to a stop, as the model would predict it
  vs = np.maximum(v - decel * LEAD_T, 0.)
  xs = d + RADAR_TO_CAMERA + np.array([np.trapezoid(np.maximum(v - decel * np.linspace(0, t, 50), 0.), np.linspace(0, t, 50)) for t in LEAD_T])
  return NS(x=list(xs), v=list(vs))


def mpc_accel(lead, model=None, v_ego=12.):
  mpc = LongitudinalMpc()
  mpc.set_cur_state(v_ego, 0.)
  for _ in range(5):
    mpc.update(NS(leadOne=lead, leadTwo=NS(present=False, modelProb=0.)), model_leads=[model] if model is not None else None)
  return float(mpc.a_solution.min())  # the hardest braking it plans


class TestModelLeadTrajectory(OpenpilotTestCase):
  def test_match(self):
    lead = radar_lead(40., 8.)
    self.assertTrue(model_lead_matches(lead, model_lead(40., 8., 1.)))
    self.assertFalse(model_lead_matches(lead, model_lead(60., 8., 1.)))  # another car
    self.assertFalse(model_lead_matches(lead, model_lead(40., 11., 1.)))
    self.assertFalse(model_lead_matches(radar_lead(40., 8., prob=0.2), model_lead(40., 8., 1.)))
    self.assertFalse(model_lead_matches(lead, None))

  def test_radar_first_second_then_model(self):
    lead = radar_lead(40., 8.)  # the radar sees no braking yet
    radar = LongitudinalMpc.extrapolate_lead(40., 8., 0., 1.5)
    xv = blend_model_lead(radar, lead, model_lead(40., 8., 1.))
    early = T_IDXS <= 1.
    np.testing.assert_allclose(xv[early, 1], radar[early, 1])
    late = T_IDXS >= 3.
    np.testing.assert_allclose(xv[late, 1], np.maximum(8. - T_IDXS[late], 0.), atol=0.05)
    self.assertEqual(xv[0, 0], radar[0, 0])
    self.assertTrue(np.all(np.diff(xv[:, 0]) >= -1e-9))

  def test_never_more_optimistic_while_radar_sees_braking(self):
    # the radar measures hard braking, the model thinks the lead drives on: the radar's trajectory stays
    lead = radar_lead(30., 10., a=-3.)
    radar = LongitudinalMpc.extrapolate_lead(30., 10., -3., 1.5)
    xv = blend_model_lead(radar, lead, model_lead(30., 10., 0.))
    np.testing.assert_allclose(xv[:, 1], radar[:, 1], atol=1e-9)

  def test_braking_lead_brakes_earlier(self):
    # a lead 45 m ahead at 8 m/s that the model sees braking to a stop; the radar's aLeadK has not caught up
    lead = radar_lead(45., 8.)
    base = mpc_accel(lead)
    with_model = mpc_accel(lead, model_lead(45., 8., 1.5))
    self.assertLess(with_model, base - 0.2)
    # an unmatched model lead changes nothing
    self.assertAlmostEqual(mpc_accel(lead, model_lead(80., 8., 1.5)), base, places=3)


class TestModelStopsShort(OpenpilotTestCase):
  def test_rule(self):
    lead = radar_lead(10., 0.)
    v = 4.  # 14 km/h: the model's -1.0 stops in 8 m
    self.assertFalse(model_stops_short(-1.0, -1.3, lead, v))  # 8 m > 10 - 3.5
    self.assertTrue(model_stops_short(-1.0, -1.3, radar_lead(8. + MODEL_STOP_MARGIN, 0.), v))
    self.assertFalse(model_stops_short(-1.0, -0.8, radar_lead(20., 0.), v))   # the MPC is the softer one
    self.assertFalse(model_stops_short(0.2, -0.8, radar_lead(20., 0.), v))    # the model does not brake
    self.assertFalse(model_stops_short(-1.0, -1.3, radar_lead(20., 2.), v))   # the lead still moves
    self.assertFalse(model_stops_short(-1.0, -1.3, NS(present=False), v))
