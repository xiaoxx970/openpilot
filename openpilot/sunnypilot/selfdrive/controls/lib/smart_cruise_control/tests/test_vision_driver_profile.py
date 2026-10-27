from typing import Any

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.tests.test_curve_speed import model_path, one_curve
from openpilot.common.test import OpenpilotTestCase

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
KPH = 1 / 3.6


def build_sm(v_ego, s_car, curvature_fn, lane_change=False):
  x, y, heading = model_path(s_car, v_ego, curvature_fn)
  t = ModelConstants.T_IDXS
  model = messaging.new_message('modelV2')
  model.modelV2.position = log.XYZTData.new_message(x=[float(i) for i in x], y=[float(i) for i in y],
                                                    z=[0.] * len(x), t=[0.] * len(x))
  model.modelV2.orientation = log.XYZTData.new_message(x=[0.] * len(x), y=[0.] * len(x), z=[float(i) for i in heading])
  model.modelV2.orientationRate = log.XYZTData.new_message(z=[float(v_ego * k) for k in curvature_fn(s_car + v_ego * np.array(t))])
  model.modelV2.velocity = log.XYZTData.new_message(x=[float(v_ego)] * len(x))
  if lane_change:
    model.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  car_state = messaging.new_message('carState')
  car_state.carState.vEgo = float(v_ego)
  car_state.carState.yawRate = float(v_ego * curvature_fn(np.array([s_car]))[0])
  controls_state = messaging.new_message('controlsState')
  return {'modelV2': model.modelV2, 'carState': car_state.carState, 'controlsState': controls_state.controlsState}


class TestVisionDriverProfile(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.params.put_bool("CurveSpeedDriverProfile", True, block=True)
    self.scc_v = SmartCruiseControlVision()
    self.curve = one_curve(40., 80., 80.)

  def teardown_method(self):
    self.params.put_bool("CurveSpeedDriverProfile", False, block=True)

  def run_for(self, seconds, v_ego=60 * KPH, long_enabled=True, long_override=False):
    # the car holds its place at s = 0, so the curve ahead stays where it is
    sm: Any = build_sm(v_ego, 0., self.curve)
    for _ in range(int(seconds / DT_MDL)):
      self.scc_v.update(sm, long_enabled, long_override, v_ego, 0., 120 * KPH)
    return sm

  def test_reads_the_toggle(self):
    assert self.scc_v.driver_profile

  def test_disabled_without_longitudinal(self):
    self.run_for(1., long_enabled=False)
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.profile_active and self.scc_v.profile_a_cmd == 0.

  def test_override_hands_back(self):
    self.run_for(2.)
    self.run_for(0.5, long_override=True)
    assert self.scc_v.state == VisionState.overriding
    assert not self.scc_v.profile_active

  def test_brakes_for_a_curve_ahead_without_a_v_target(self):
    # the car stands at s = 0 with a 40 m radius 80 m ahead at 60 km/h: approach, braking command, no cruise target
    self.run_for(2.)
    assert self.scc_v.state == VisionState.entering
    assert self.scc_v.profile_active and self.scc_v.is_active
    assert self.scc_v.profile_a_cmd < -0.3
    assert self.scc_v.output_v_target == V_CRUISE_UNSET, 'the profile must not take part in the cruise target min'
    assert 20 * KPH < self.scc_v.published_v_target < 60 * KPH
    assert self.scc_v.output_a_target == self.scc_v.profile_a_cmd

  def test_straight_road_is_idle(self):
    # a model path is never exactly straight; all-zero yaw rates divide by zero in the stock calculations
    self.curve = one_curve(1e5, -1e3, 1e4)
    self.run_for(2.)
    assert self.scc_v.state == VisionState.enabled
    assert not self.scc_v.profile_active
    assert self.scc_v.published_v_target == V_CRUISE_UNSET

  def test_toggle_off_is_stock(self):
    self.params.put_bool("CurveSpeedDriverProfile", False, block=True)
    stock = SmartCruiseControlVision()
    assert not stock.driver_profile
    sm = build_sm(60 * KPH, 0., self.curve)
    for _ in range(int(2. / DT_MDL)):
      stock.update(sm, True, False, 60 * KPH, 0., 120 * KPH)
    assert not stock.profile_active
    assert stock.published_v_target == stock.output_v_target
