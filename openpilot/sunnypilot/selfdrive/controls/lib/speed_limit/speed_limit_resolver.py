"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.road_events.prompts import CameraPrompt
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE, LIMIT_ADAPT_ACC
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy, OffsetType

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ALL_SOURCES = tuple(SpeedLimitSource.schema.enumerants.values())
BAIDU_MAX_AGE = 1.5  # s without a roadEventsIC message (2 Hz) before Baidu's limits are dropped


class SpeedLimitResolver:
  limit_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  distance_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: custom.LongitudinalPlanSP.SpeedLimit.Source
  speed_limit_offset: float

  def __init__(self):
    self.params = Params()
    self.frame = -1

    self._gps_location_service = get_gps_location_service(self.params)
    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
    self.policy = get_sanitize_int_param(
      "SpeedLimitPolicy",
      Policy.min().value,
      Policy.max().value,
      self.params
    )
    self._policy_to_sources_map = {
      Policy.car_state_only: [SpeedLimitSource.car],
      Policy.map_data_only: [SpeedLimitSource.map],
      Policy.car_state_priority: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.map_data_priority: [SpeedLimitSource.map, SpeedLimitSource.car],
      Policy.combined: [SpeedLimitSource.car, SpeedLimitSource.map],
    }
    self.source = SpeedLimitSource.none
    self.baidu_in_use = False
    self.baidu_camera = False   # Baidu's limit is the speed camera's (fork)
    self.camera = False         # the resolved limit is a Baidu speed camera's (fork)
    self._camera: tuple[float, float] | None = None   # (m to it, limit m/s) of the speed camera in use
    self._last_before_camera = (0., 0.)              # speed_limit_last, speed_limit_final_last from before it
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)

    self.is_metric = self.params.get_bool("IsMetric")
    self.offset_type = get_sanitize_int_param(
      "SpeedLimitOffsetType",
      OffsetType.min().value,
      OffsetType.max().value,
      self.params
    )
    self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

    self.speed_limit = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    if self.speed_limit > 0.:
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final

  @property
  def speed_limit_valid(self) -> bool:
    return self.speed_limit > 0.

  @property
  def speed_limit_last_valid(self) -> bool:
    return self.speed_limit_last > 0.

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
      self.is_metric = self.params.get_bool("IsMetric")
      self.offset_type = self.params.get("SpeedLimitOffsetType", return_default=True)
      self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def _get_speed_limit_offset(self, speed_limit: float | None = None) -> float:
    speed_limit = self.speed_limit if speed_limit is None else speed_limit
    if self.offset_type == OffsetType.off:
      return 0
    elif self.offset_type == OffsetType.fixed:
      return float(self.offset_value * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    elif self.offset_type == OffsetType.percentage:
      return float(self.offset_value * 0.01 * speed_limit)
    else:
      raise NotImplementedError("Offset not supported")

  def _reset_limit_sources(self, source: custom.LongitudinalPlanSP.SpeedLimit.Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    self.limit_solutions[SpeedLimitSource.car] = sm['carStateSP'].speedLimit
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_map_data(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._process_map_data(sm)

  def _process_map_data(self, sm: messaging.SubMaster) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    gps_fix_age = time.monotonic() - gps_data.unixTimestampMillis * 1e-3
    if gps_fix_age > LIMIT_MAX_MAP_DATA_AGE:
      return

    speed_limit = map_data.speedLimit if map_data.speedLimitValid else 0.
    next_speed_limit = map_data.speedLimitAhead if map_data.speedLimitAheadValid else 0.

    self._calculate_map_data_limits(sm, speed_limit, next_speed_limit)

  def _calculate_map_data_limits(self, sm: messaging.SubMaster, speed_limit: float, next_speed_limit: float) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    distance_since_fix = self.v_ego * (time.monotonic() - gps_data.unixTimestampMillis * 1e-3)
    distance_to_speed_limit_ahead = max(0., map_data.speedLimitAheadDistance - distance_since_fix)

    self.limit_solutions[SpeedLimitSource.map] = speed_limit
    self.distance_solutions[SpeedLimitSource.map] = 0.

    # FIXME-SP: this is not working as expected
    if 0. < next_speed_limit < self.v_ego:
      adapt_time = (next_speed_limit - self.v_ego) / LIMIT_ADAPT_ACC
      adapt_distance = self.v_ego * adapt_time + 0.5 * LIMIT_ADAPT_ACC * adapt_time ** 2

      if distance_to_speed_limit_ahead <= adapt_distance:
        self.limit_solutions[SpeedLimitSource.map] = next_speed_limit
        self.distance_solutions[SpeedLimitSource.map] = distance_to_speed_limit_ahead

  def _get_from_baidu(self, sm: messaging.SubMaster) -> bool:
    """Baidu Maps' limits from roadeventd (fork): True when they stand in for the offline map's, i.e. with a fresh limit for the
    road the car is on or a speed camera in use. Otherwise the offline map stays."""
    self._reset_limit_sources(SpeedLimitSource.baidu)
    self.baidu_camera = False
    if 'roadEventsIC' not in getattr(sm, 'services', ()) or not sm.seen['roadEventsIC'] or \
       time.monotonic() - sm.recv_time['roadEventsIC'] > BAIDU_MAX_AGE:
      self._camera = None
      return False
    road = sm['roadEventsIC']
    in_use = False
    if road.speedLimitValid:
      self.limit_solutions[SpeedLimitSource.baidu] = road.speedLimit
      in_use = True
    camera = self._update_camera(sm, road, time.monotonic() - sm.recv_time['roadEventsIC'])
    if camera is not None:
      distance, self.limit_solutions[SpeedLimitSource.baidu] = camera
      self.distance_solutions[SpeedLimitSource.baidu] = max(0., distance)
      self.baidu_camera = in_use = True
    return in_use

  def _update_camera(self, sm: messaging.SubMaster, road, age: float) -> tuple[float, float] | None:
    """A Baidu speed camera's limit, as a limit ahead: taken up LOOK_AHEAD s before the camera (as its cluster prompt) when the
    set speed is above it, so it never offers to speed up, and kept while that camera is reported, until 50 m past it
    (roadeventd), with the set speed at its limit or above. Every camera is taken as a point camera."""
    limit, distance = road.speedLimitAhead, road.speedLimitAheadDistance - self.v_ego * age
    if limit <= 0.:
      self._camera = None
      return None
    set_speed_kph = min(sm['carState'].vCruiseCluster, V_CRUISE_MAX)
    target_kph = round((limit + self._get_speed_limit_offset(limit)) * CV.MS_TO_KPH)
    same = self._camera is not None and abs(limit - self._camera[1]) < 0.1 and distance < self._camera[0] + CameraPrompt.SAME
    if same and target_kph <= set_speed_kph:
      self._camera = (distance, limit)
    elif not same and 0. <= distance <= CameraPrompt.window(self.v_ego) and target_kph < set_speed_kph:
      self._camera = (distance, limit)
    else:
      self._camera = None
    return self._camera

  def _get_source_solution_according_to_policy(self) -> custom.LongitudinalPlanSP.SpeedLimit.Source:
    # Baidu replaces the offline map wherever a policy uses it
    sources_for_policy = [SpeedLimitSource.baidu if s == SpeedLimitSource.map and self.baidu_in_use else s
                          for s in self._policy_to_sources_map[Policy(self.policy)]]

    if Policy(self.policy) != Policy.combined:
      # They are ordered in the order of preference, so we pick the first that's non-zero
      for source in sources_for_policy:
        if self.limit_solutions[source] > 0.:
          return source
      return SpeedLimitSource.none

    sources_with_limits = [(s, limit) for s, limit in [(s, self.limit_solutions[s]) for s in sources_for_policy] if limit > 0.]
    if sources_with_limits:
      return min(sources_with_limits, key=lambda x: x[1])[0]

    return SpeedLimitSource.none

  def _resolve_limit_sources(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Get limit solutions from each data source"""
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)
    self.baidu_in_use = self._get_from_baidu(sm)

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    return speed_limit, distance, source

  def update(self, v_ego: float, sm: messaging.SubMaster) -> None:
    self.v_ego = v_ego
    self.update_params()

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    camera = self.source == SpeedLimitSource.baidu and self.baidu_camera
    if camera and not self.camera:
      self._last_before_camera = (self.speed_limit_last, self.speed_limit_final_last)
    self.update_speed_limit_states()
    if self.camera and not camera and self.speed_limit == 0.:
      # past a speed camera with no other limit: the last limit is again the one from before it, not the camera's
      self.speed_limit_last, self.speed_limit_final_last = self._last_before_camera
    self.camera = camera

    self.frame += 1
