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
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.tests.test_map_curvature import north_road, road_json, to_latlon
from openpilot.common.test import OpenpilotTestCase

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
KPH = 1 / 3.6
GPS = 'gpsLocation'
NOW = int(100e9)


class FakeSubMaster(dict):
  def __init__(self, msgs, mono_times):
    super().__init__(msgs)
    self.logMonoTime = mono_times


def build_sm(v_ego, s_car, curvature_fn, lane_change=False, gps_xy=None, gps_age=0.1, now=NOW, rain=None):
  """rain: (rainMode, rainLevel) for a selfdriveStateIC message, None leaves the service out."""
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
  gps = messaging.new_message(GPS)
  mono = {'modelV2': now, GPS: 0}
  if gps_xy is not None:
    # a fix on the test road (north_road), heading north at the car's speed
    lat, lon = to_latlon(*gps_xy)
    g = getattr(gps, GPS)
    g.latitude, g.longitude, g.bearingDeg, g.speed = float(lat), float(lon), 0., float(v_ego)
    g.hasFix, g.horizontalAccuracy = True, 2.
    mono[GPS] = now - int(gps_age * 1e9)
  msgs = {'modelV2': model.modelV2, 'carState': car_state.carState, 'controlsState': controls_state.controlsState, GPS: getattr(gps, GPS)}
  if rain is not None:
    ss_ic = messaging.new_message('selfdriveStateIC')
    ss_ic.selfdriveStateIC.rainMode, ss_ic.selfdriveStateIC.rainLevel = rain
    msgs['selfdriveStateIC'] = ss_ic.selfdriveStateIC
  return FakeSubMaster(msgs, mono)


def straight(s):
  # a model path is never exactly straight; all-zero yaw rates divide by zero in the stock calculations
  return np.full_like(s, 1e-5)


class TestVisionDriverProfile(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.params.put_bool("CurveSpeedDriverProfile", True, block=True)
    self.params.remove("MapCurvatures")
    self.scc_v = SmartCruiseControlVision()
    self.curve = one_curve(40., 80., 80.)

  def teardown_method(self):
    self.params.put_bool("CurveSpeedDriverProfile", False, block=True)
    self.params.remove("MapCurvatures")

  def run_for(self, seconds, v_ego=60 * KPH, long_enabled=True, long_override=False, rain=None):
    # the car holds its place at s = 0, so the curve ahead stays where it is
    sm: Any = build_sm(v_ego, 0., self.curve, rain=rain)
    for _ in range(int(seconds / DT_MDL)):
      self.scc_v.update(sm, long_enabled, long_override, v_ego, 0., 120 * KPH)
    return sm

  def drive_on_map(self, seconds, v_ego=60 * KPH, y0=0., gps_age=0.1):
    """Drive north along the test road with GPS fixes; the camera sees a straight road."""
    y = y0
    for i in range(int(seconds / DT_MDL)):
      sm: Any = build_sm(v_ego, y, straight, gps_xy=(1., y), gps_age=gps_age, now=NOW + int(i * DT_MDL * 1e9))
      self.scc_v.update(sm, True, False, v_ego, 0., 120 * KPH)
      y += v_ego * DT_MDL
    return y

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
    assert not self.scc_v.profile_map_valid and not self.scc_v.profile_map_limited

  def test_straight_road_is_idle(self):
    self.curve = straight
    self.run_for(2.)
    assert self.scc_v.state == VisionState.enabled
    assert not self.scc_v.profile_active
    assert self.scc_v.published_v_target == V_CRUISE_UNSET

  def test_map_curve_beyond_the_camera(self):
    # mapd's road has a 30 m radius 420-460 m north of the start; the camera sees nothing
    self.params.put("MapCurvatures", road_json(*north_road(length=800., curve=(420., 460.), kappa=1 / 30.)), block=True)
    y = self.drive_on_map(1.)
    assert self.scc_v.profile_map_valid
    assert self.scc_v.state == VisionState.enabled, 'the curve is still past the map horizon'
    self.drive_on_map(10., y0=y)
    assert self.scc_v.state == VisionState.entering and self.scc_v.profile_a_cmd < 0.
    assert self.scc_v.profile_map_limited
    assert self.scc_v.profile_planner.a_req_map < self.scc_v.profile_planner.a_req_vision

  def test_stale_gps_is_not_used(self):
    self.params.put("MapCurvatures", road_json(*north_road(curve=(260., 300.), kappa=1 / 30.)), block=True)
    self.drive_on_map(2., gps_age=3.)
    assert not self.scc_v.profile_map_valid and not self.scc_v.profile_map_limited

  def test_toggle_off_is_stock(self):
    self.params.put_bool("CurveSpeedDriverProfile", False, block=True)
    stock = SmartCruiseControlVision()
    assert not stock.driver_profile
    sm = build_sm(60 * KPH, 0., self.curve)
    for _ in range(int(2. / DT_MDL)):
      stock.update(sm, True, False, 60 * KPH, 0., 120 * KPH)
    assert not stock.profile_active
    assert stock.published_v_target == stock.output_v_target

  def test_rain_mode_lowers_the_comfortable_lateral_accel(self):
    rain, heavy = self.scc_v.profile_planner.p.rain_lat_scale
    self.run_for(2.)
    a_req_dry = self.scc_v.profile_planner.a_req
    self.run_for(6., rain=(True, 1))
    np.testing.assert_allclose(self.scc_v.profile_planner.comfort_scale, rain)
    a_req_rain = self.scc_v.profile_planner.a_req
    assert a_req_rain < a_req_dry - 0.02, 'a lower target speed for the same curve needs more braking'
    self.run_for(3., rain=(True, 2))
    np.testing.assert_allclose(self.scc_v.profile_planner.comfort_scale, heavy)
    assert self.scc_v.profile_planner.a_req < a_req_rain

  def test_rain_without_rain_mode_is_dry(self):
    # raining but the driver stepped off the rain following distance, or no selfdriveStateIC at all
    for rain in ((False, 2), None):
      self.run_for(6., rain=rain)
      assert self.scc_v.profile_planner.comfort_scale == 1.

  def test_rain_scale_applies_at_once_while_not_controlling(self):
    self.run_for(0.5, long_enabled=False, rain=(True, 2))
    assert self.scc_v.profile_planner.comfort_scale == self.scc_v.profile_planner.p.rain_lat_scale[1]
