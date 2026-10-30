import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_planner import RAIN_ACCEL_CAP, RAIN_ACCEL_CAP_RATE
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant


def launch_accels(rain_level):
  # from a stop with no lead toward a far set speed: every candidate lets the command climb
  man = Maneuver('', duration=8., initial_speed=0., rain_level=rain_level)
  valid, output = man.evaluate()
  assert valid
  return output[:, 5]


class TestRainAccelCap(OpenpilotTestCase):
  def test_launch_capped_in_rain(self):
    dry = launch_accels(0).max()
    self.assertGreater(dry, RAIN_ACCEL_CAP[0] + 0.3)
    for rain_level, cap in ((1, RAIN_ACCEL_CAP[0]), (2, RAIN_ACCEL_CAP[1])):
      accels = launch_accels(rain_level)
      self.assertLessEqual(accels.max(), cap + 1e-6)
      self.assertGreater(accels.max(), cap - 0.05)

  def test_no_step_as_rain_mode_starts_and_ends(self):
    # accelerating hard when rain mode starts and while it ends: the command follows the ramping cap, it never steps
    plant = Plant(speed=0.)
    t, accels = [], []
    while plant.current_time < 14.:
      plant.rain_level = 1 if 3. <= plant.current_time < 8. else 0
      t.append(plant.current_time)
      accels.append(plant.step(v_cruise=50.)['acceleration'])
    t, accels = np.array(t), np.array(accels)
    self.assertGreater(accels[(t > 2.) & (t < 3.)].min(), RAIN_ACCEL_CAP[0] + 0.5)
    # from the last dry frame on, the command comes down along the cap ramp and goes back up along it
    self.assertLessEqual(np.abs(np.diff(accels[t >= 2.95])).max(), RAIN_ACCEL_CAP_RATE * DT_MDL + 1e-6)
    settled = (t >= 3. + (ACCEL_MAX - RAIN_ACCEL_CAP[0]) / RAIN_ACCEL_CAP_RATE + 0.1) & (t < 8.)
    self.assertLessEqual(accels[settled].max(), RAIN_ACCEL_CAP[0] + 1e-6)
    self.assertGreater(accels[-1], RAIN_ACCEL_CAP[0] + 0.5)

  def test_logged_in_the_plan(self):
    plant = Plant(speed=10., rain_level=2)
    plant.step(v_cruise=50.)
    msg = messaging.new_message('longitudinalPlanIC')
    msg.longitudinalPlanIC.rainAccelCap = plant.planner.rain_accel_cap.cap
    msg.longitudinalPlanIC.rainCurveLatScale = plant.planner.scc.vision.profile_planner.comfort_scale
    plan_ic = messaging.log_from_bytes(msg.to_bytes()).longitudinalPlanIC
    self.assertAlmostEqual(plan_ic.rainAccelCap, RAIN_ACCEL_CAP[1], places=5)
    self.assertEqual(plan_ic.rainCurveLatScale, 1.)  # the curve profile only runs with the driver profile toggle on
