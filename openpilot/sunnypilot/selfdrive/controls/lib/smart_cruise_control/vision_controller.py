"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import platform
from collections import deque

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.curve_speed import CurveSpeedPlanner
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_curvature import MapCurvatureAhead

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.3  # Predicted Lat Acc threshold to trigger entering turn state.
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # Predicted Lat Acc threshold to abort entering state if speed drops.

_TURNING_LAT_ACC_TH = 1.6  # Lat Acc threshold to trigger turning state.

_LEAVING_LAT_ACC_TH = 1.3  # Lat Acc threshold to trigger leaving turn state.
_FINISH_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger the end of the turn cycle.

_A_LAT_REG_MAX = 2.  # Maximum lateral acceleration

_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]  # min decel value allowed on ENTERING state
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]  # absolute value of lat acc ahead

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.5, 0., -0.4]  # acc value
_TURNING_ACC_BP = [1.5, 2.3, 3.]  # absolute value of current lat acc

_LEAVING_ACC = 0.5  # Conformable acceleration to regain speed while leaving a turn.

# Direction of the turn being slowed for, e.g. to pick a left/right curve icon on the car's cluster.
CURVE_NONE = 0
CURVE_LEFT = -1
CURVE_RIGHT = 1
CURVE_S_BEND = 2
_S_BEND_RATIO = 0.6  # both directions reach this share of the peak lateral accel ahead -> S-bend

# Driver profile (CurveSpeedDriverProfile): CurveSpeedPlanner states as SCC-V states, so the logs, the onroad icon and
# the cluster curve prompt keep working unchanged
_PROFILE_STATES = {
  CurveSpeedPlanner.CRUISE: VisionState.enabled,
  CurveSpeedPlanner.APPROACH: VisionState.entering,
  CurveSpeedPlanner.CURVE: VisionState.turning,
  CurveSpeedPlanner.EXIT: VisionState.leaving,
}
# The planner was tuned on yaw rate / speed smoothed over 0.5 s (logged carState), so it gets the same here
_PROFILE_KAPPA_SAMPLES = int(round(0.5 / DT_MDL))
# mapd's curvature ahead (MapCurvatures) only changes when mapd matches another road, so a few reads a second are enough;
# a GPS fix older or less accurate than this does not place the car on it
_MAP_READ_FRAMES = 5
_MAP_GPS_MAX_AGE = 1.5  # s
_MAP_GPS_MAX_ACCURACY = 20.  # m


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.driver_profile = self.params.get_bool("CurveSpeedDriverProfile")
    self.v_cruise_setpoint = 0.

    # driver profile: its braking command bypasses the cruise decel floor as its own longitudinal planner candidate
    self.profile_planner = CurveSpeedPlanner(dt=DT_MDL)
    self.profile_kappa = deque(maxlen=_PROFILE_KAPPA_SAMPLES)
    self.profile_path = None
    self.profile_a_cmd = 0.
    self.profile_active = False
    self.profile_display_v = 0.
    # map curvature ahead, placed on the road by GPS; whether it is in use and sets the braking goes to the log
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.gps_location_service = get_gps_location_service(self.params)
    self.map_ahead = MapCurvatureAhead()
    self.profile_odometer = 0.
    self.profile_map_valid = False
    self.profile_map_limited = False

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.pred_curve_direction = CURVE_NONE
    self.curve_direction = CURVE_NONE

  @staticmethod
  def get_curve_direction(signed_lat_accels: np.ndarray) -> int:
    # modelV2 orientation is in the device frame (z down): a positive yaw rate is a right turn
    right = max(float(np.max(signed_lat_accels)), 0.)
    left = max(-float(np.min(signed_lat_accels)), 0.)
    peak = max(left, right)
    if peak < _ABORT_ENTERING_PRED_LAT_ACC_TH:
      return CURVE_NONE
    if min(left, right) >= _S_BEND_RATIO * peak:
      return CURVE_S_BEND
    return CURVE_LEFT if left > right else CURVE_RIGHT

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.driver_profile:
      return V_CRUISE_UNSET  # it commands acceleration directly, see profile_a_cmd
    if self.is_active:
      return max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON

    return V_CRUISE_UNSET

  @property
  def published_v_target(self) -> float:
    """Target speed for the logs and the curve prompts: with the driver profile the rate-limited target it shows."""
    if self.driver_profile:
      return self.profile_display_v if self.profile_active else V_CRUISE_UNSET
    return self.output_v_target

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")
      driver_profile = self.params.get_bool("CurveSpeedDriverProfile")
      if driver_profile != self.driver_profile:
        self.profile_planner.reset()
      self.driver_profile = driver_profile

  def _update_map_ahead(self, sm: messaging.SubMaster, v_ego: float):
    """mapd's curvature ahead as (distance along the road, |curvature|) from the last GPS fix, or (None, None)."""
    if self.frame % _MAP_READ_FRAMES == 0:
      self.map_ahead.set_points(self.mem_params.get("MapCurvatures"))
    self.profile_odometer += v_ego * DT_MDL
    gps = sm[self.gps_location_service]
    gps_time = sm.logMonoTime[self.gps_location_service]
    age = max(0., (sm.logMonoTime['modelV2'] - gps_time) * 1e-9)
    if not gps_time or age > _MAP_GPS_MAX_AGE or not gps.hasFix or gps.horizontalAccuracy > _MAP_GPS_MAX_ACCURACY:
      return None, None
    ahead = self.map_ahead.ahead(gps.latitude, gps.longitude, gps.bearingDeg, v_ego, self.profile_odometer, advance=v_ego * age)
    return ahead if ahead is not None else (None, None)

  def _update_driver_profile(self, sm: messaging.SubMaster) -> None:
    cs = sm['carState']
    v_ego = cs.vEgo
    self.profile_kappa.append(abs(cs.yawRate) / max(v_ego, 3.) if v_ego > 3. else 0.)
    model = sm['modelV2']
    if len(model.position.x) == ModelConstants.IDX_N and len(model.orientation.z) == ModelConstants.IDX_N:
      self.profile_path = (np.array(model.position.x), np.array(model.position.y), np.array(model.orientation.z))
    lane_change = model.meta.laneChangeState != log.LaneChangeState.off
    map_dist, map_kappa = self._update_map_ahead(sm, v_ego)

    run = self.long_enabled and self.enabled and not self.long_override and self.profile_path is not None
    if run:
      _, self.profile_a_cmd = self.profile_planner.update(v_ego, cs.aEgo, sum(self.profile_kappa) / len(self.profile_kappa),
                                                         *self.profile_path, lane_change=lane_change,
                                                         map_dist=map_dist, map_kappa=map_kappa)
    else:
      self.profile_planner.reset()
      self.profile_a_cmd = 0.
    self.profile_active = run and self.profile_planner.state != CurveSpeedPlanner.CRUISE
    self.profile_display_v = self.profile_planner.v_display
    self.profile_map_valid = map_dist is not None
    self.profile_map_limited = self.profile_active and self.profile_planner.map_limited

    if not self.long_enabled or not self.enabled:
      self.state = VisionState.disabled
    elif self.long_override:
      self.state = VisionState.overriding
    else:
      self.state = _PROFILE_STATES[self.profile_planner.state]
    self.current_lat_acc = v_ego ** 2 * self.profile_planner.kappa_f

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      return
    else:
      signed_rate_plan = np.array(sm['modelV2'].orientationRate.z)
      rate_plan = np.abs(signed_rate_plan)
      vel_plan = np.array(sm['modelV2'].velocity.x)
      self.pred_curve_direction = self.get_curve_direction(signed_rate_plan * vel_plan)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # get the maximum lat accel from the model
      predicted_lat_accels = rate_plan * vel_plan
      self.max_pred_lat_acc = np.percentile(predicted_lat_accels, 97)

      # get the maximum curve based on the current velocity
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      max_curve = self.max_pred_lat_acc / (v_ego**2)

      # Get the target velocity for the maximum curve
      self.v_target = (_A_LAT_REG_MAX / max_curve) ** 0.5

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          elif self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Transition back to Turning if current lateral acceleration goes back over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          # Finish if current lateral acceleration goes below a threshold.
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      a_target = np.interp(self.max_pred_lat_acc, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    if self.driver_profile:
      self._update_driver_profile(sm)
      self.is_enabled, self.is_active = self.state in ENABLED_STATES, self.state in ACTIVE_STATES
      self.a_target = self.profile_a_cmd
    else:
      self.profile_active = self.profile_map_valid = self.profile_map_limited = False
      self.is_enabled, self.is_active = self._update_state_machine()
      self.a_target = self._update_solution()

    # Latch the direction for the whole slow-down so a cluster icon does not flip mid-curve
    if self.state in (VisionState.entering, VisionState.turning):
      if self.curve_direction == CURVE_NONE:
        self.curve_direction = self.pred_curve_direction
    else:
      self.curve_direction = CURVE_NONE

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
