"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(0, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'unixTimestampMillis': time.monotonic() * 1e3,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  return sm_mock


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].unixTimestampMillis = (time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e3
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


def setup_baidu_sm_mock(mocker, map_limit, car_limit=0., road=None, age=0.2):
  """Fork: roadeventd's Baidu limits next to the car's and the offline map's."""
  sm_mock = setup_sm_mock(mocker)
  items = {key: sm_mock[key] for key in ('carState', 'liveMapDataSP', 'carStateSP', 'gpsLocation')}
  items['liveMapDataSP'].speedLimit = map_limit
  items['carStateSP'].speedLimit = car_limit
  if road is not None:
    items['roadEventsIC'] = create_mock({'speedLimit': 0., 'speedLimitValid': False, 'speedLimitAhead': 0., 'speedLimitAheadDistance': 0.,
                                         **road}, mocker)
  sm_mock.__getitem__.side_effect = lambda key: items[key]
  sm_mock.services = list(items)
  sm_mock.seen = dict.fromkeys(items, True)
  sm_mock.recv_time = dict.fromkeys(items, time.monotonic() - age)
  return sm_mock


class TestSpeedLimitResolverBaidu(OpenpilotTestCase):
  MAP, BAIDU = 80 / 3.6, 60 / 3.6

  @parameterized.expand([(Policy.map_data_only,), (Policy.map_data_priority,), (Policy.car_state_priority,)], names=["policy"])
  def test_baidu_replaces_the_map(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(20., setup_baidu_sm_mock(mocker, self.MAP, road={'speedLimit': self.BAIDU, 'speedLimitValid': True}))
    assert resolver.source == SpeedLimitSource.baidu
    assert resolver.speed_limit == self.BAIDU

  @parameterized.expand([({'speedLimitValid': False},), (None,)], names=["road"])
  def test_the_map_without_a_baidu_limit(self, resolver_class, road, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    resolver.update(20., setup_baidu_sm_mock(mocker, self.MAP, road=road))
    assert resolver.source == SpeedLimitSource.map
    assert resolver.speed_limit == self.MAP

  def test_the_map_when_baidu_is_stale(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    sm_mock = setup_baidu_sm_mock(mocker, self.MAP, road={'speedLimit': self.BAIDU, 'speedLimitValid': True}, age=2.)
    resolver.update(20., sm_mock)
    assert resolver.source == SpeedLimitSource.map

  def test_speed_camera_ahead(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    v_ego = 80 / 3.6   # adapting from 80 to 60 km/h at 1 m/s^2 takes 5.6 s and 108 m
    near = {'speedLimitAhead': self.BAIDU, 'speedLimitAheadDistance': 95.}
    resolver.update(v_ego, setup_baidu_sm_mock(mocker, 0., road=near, age=0.2))
    assert resolver.source == SpeedLimitSource.baidu
    assert resolver.speed_limit == self.BAIDU
    assert 0. < resolver.distance < 95.   # aged by the message's age
    resolver.update(v_ego, setup_baidu_sm_mock(mocker, self.MAP, road={**near, 'speedLimitAheadDistance': 400.}))
    assert resolver.source == SpeedLimitSource.map
    resolver.update(self.BAIDU, setup_baidu_sm_mock(mocker, self.MAP, road=near))   # not faster than the camera
    assert resolver.source == SpeedLimitSource.map

  def test_car_state_only_unchanged(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.car_state_only
    resolver.update(20., setup_baidu_sm_mock(mocker, self.MAP, car_limit=50 / 3.6, road={'speedLimit': self.BAIDU, 'speedLimitValid': True}))
    assert resolver.source == SpeedLimitSource.car
    assert resolver.speed_limit == 50 / 3.6

  def test_combined_takes_baidu_in_place_of_the_map(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    resolver.update(20., setup_baidu_sm_mock(mocker, 40 / 3.6, car_limit=90 / 3.6, road={'speedLimit': self.BAIDU, 'speedLimitValid': True}))
    assert resolver.source == SpeedLimitSource.baidu
    assert resolver.speed_limit == self.BAIDU
