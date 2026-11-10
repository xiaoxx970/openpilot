from types import SimpleNamespace as NS

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS, model_lead_matches, blend_model_lead
from openpilot.selfdrive.controls.lib.longitudinal_planner import MODEL_STOP_BLEND_TIME, MODEL_STOP_ENTER_MARGIN_ADD, MODEL_STOP_MARGIN, \
                                                                   MODEL_STOP_MIN_GAP, ModelStopBlend, model_stops_short
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


class TestModelStopBlend(OpenpilotTestCase):
  DT = 0.05
  A_MODEL, A_MPC = -0.55, -0.95

  def ramp(self, blend, lead, v, a_mpc=None, frames=None):
    frames = frames or int(MODEL_STOP_BLEND_TIME / self.DT) + 4
    return [blend.update(self.A_MODEL, self.A_MPC if a_mpc is None else a_mpc, lead, v) for _ in range(frames)]

  def test_entry_and_exit_have_no_step(self):
    blend = ModelStopBlend(self.DT)
    lead = radar_lead(20., 0.)
    entry = self.ramp(blend, lead, 3.)
    self.assertAlmostEqual(entry[-1], self.A_MODEL)
    self.assertLess(np.abs(np.diff([self.A_MPC] + entry)).max(), 0.1)
    # the model is no longer the softer request's safe stop: the lead is back inside the margin
    v = 8.
    exit_ = self.ramp(blend, radar_lead(10., 0.), v, a_mpc=-0.95)
    self.assertAlmostEqual(exit_[-1], -0.95, places=2)
    self.assertLess(np.abs(np.diff(exit_)).max(), 0.1)

  def test_hysteresis_holds_on_the_limit(self):
    v = 3.
    limit = v ** 2 / (2. * -self.A_MODEL) + MODEL_STOP_MARGIN
    blend = ModelStopBlend(self.DT)
    # inside the entry margin but outside the exit margin: does not start
    self.assertFalse(blend.update(self.A_MODEL, self.A_MPC, radar_lead(limit + MODEL_STOP_ENTER_MARGIN_ADD / 2, 0.), v) != self.A_MPC)
    self.assertFalse(blend.active)
    blend.update(self.A_MODEL, self.A_MPC, radar_lead(limit + MODEL_STOP_ENTER_MARGIN_ADD + 0.1, 0.), v)
    self.assertTrue(blend.active)
    # the same in-between distance keeps it going, a lead just below the entry speed limit too
    blend.update(self.A_MODEL, self.A_MPC, radar_lead(limit + MODEL_STOP_ENTER_MARGIN_ADD / 2, 0.9), v)
    self.assertTrue(blend.active)

  def test_much_harder_mpc_takes_over_at_once(self):
    blend = ModelStopBlend(self.DT)
    self.ramp(blend, radar_lead(20., 0.), 3.)
    self.assertAlmostEqual(blend.update(self.A_MODEL, -2.0, radar_lead(20., 3.), 3.), -2.0)  # the lead moved: rule off, MPC 1.45 harder

  def test_never_harder_than_the_mpc_nor_inside_the_min_gap(self):
    v = 4.
    d = MODEL_STOP_MIN_GAP + 8.
    need = -v ** 2 / (2. * (d - MODEL_STOP_MIN_GAP))  # -1.0
    blend = ModelStopBlend(self.DT)
    blend.active = True
    blend.weight = 1.
    out = blend.update(self.A_MODEL, -1.2, radar_lead(d, 2.), v)  # rule off, still mostly the model along the ramp
    self.assertAlmostEqual(out, need)
    blend.active = True
    blend.weight = 1.
    self.assertAlmostEqual(blend.update(self.A_MODEL, -0.8, radar_lead(MODEL_STOP_MIN_GAP + 1., 2.), v), -0.8)  # the limit is harder than the MPC

  def test_idle_passes_the_mpc_through(self):
    blend = ModelStopBlend(self.DT)
    self.assertEqual(blend.update(self.A_MODEL, -0.3, radar_lead(20., 0.), 3.), -0.3)  # the MPC is the softer one
    self.assertEqual(blend.update(0.2, self.A_MPC, radar_lead(20., 0.), 3.), self.A_MPC)  # the model does not brake
