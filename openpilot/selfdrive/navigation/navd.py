#!/usr/bin/env python3
import math
import os
import time

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.api import API_HOST, Api
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation import baidu_route
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import along_path, local_meters
from openpilot.selfdrive.road_events import baidu

REROUTE_DISTANCE = 25
ARRIVE_DISTANCE = 50  # m from the destination (or the route's end): arrived, and no rerouting. Baidu's last step runs from the last maneuver to
# the destination, often km: being on it is not being nearly there
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
ROUTE_RESEND_PERIOD = 10  # updates (seconds); a restarted mapsd gets the route within this
RECOMPUTE_HOLD = 10  # updates (seconds) between route requests
# Off the route, slow and hardly moving (a garage, a car park, off the road network), asking again finds no better route:
# a reroute wants some speed or some distance from where the last route was asked, and waits longer each time in a row.
REROUTE_MIN_SPEED = 3.  # m/s
REROUTE_MIN_MOVED = 30.  # m
REROUTE_BACKOFF = (10, 30, 60)  # updates (seconds) after the 1st, 2nd, 3rd and later reroutes in a row
# With Baidu's key file (road_events/baidu.py) routes come from Baidu's driving route planning, with China's roads and traffic:
# asked again every BAIDU_REFRESH while moving for fresh traffic and times, which roadeventd takes for its jams ahead.
BAIDU_REFRESH = 120.  # s
BAIDU_REFRESH_SPEED = 2.  # m/s
BAIDU_COUNT_FILE = "/data/baidu_nav_calls"
BAIDU_RETRY = 60  # updates (seconds) before asking again after a failed request
# NavDestination's waypoints [[lat, lon], ...]: the route picked on the phone among Baidu's alternatives, kept through every
# refresh and reroute; one is passed once the car comes this close to it
WAYPOINT_REACHED = 100.  # m
MAX_WAYPOINTS = 5
WAYPOINTS_GIVEN_UP = 2  # off-route reroutes since a waypoint was passed after which the rest are dropped


def as_waypoints(value) -> tuple[tuple[float, float], ...]:
  """NavDestination's waypoints as (lat, lon) pairs; a malformed list is none."""
  try:
    out = tuple((float(lat), float(lon)) for lat, lon in (value or [])[:MAX_WAYPOINTS])
  except (TypeError, ValueError):
    return ()
  return out if all(-90 <= la <= 90 and -180 <= lo <= 180 for la, lo in out) else ()


def set_banner(instruction, banners, distance_to_maneuver):
  """Fill the banner fields from the banner to show now: the last one still ahead of the ego, else the first."""
  if not banners:
    return
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  instruction.showFull = distance_to_maneuver < banner['distanceAlongGeometry']
  primary = banner['primary']
  instruction.maneuverPrimaryText = primary.get('text') or ''
  instruction.maneuverType = primary.get('type') or ''
  instruction.maneuverModifier = primary.get('modifier') or ''
  if banner.get('secondary') is not None:
    instruction.maneuverSecondaryText = banner['secondary']['text']


class RouteEngine:
  def __init__(self, sm, pm, params=None):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None

    self.nav_destination = None
    self.nav_waypoints = ()  # NavDestination's, as last routed to
    self.target = None  # NavDestination's (destination, waypoints) as last seen
    self.waypoints = []  # its waypoints still ahead, (lat, lon): kept when a request fails, so a passed one is not asked again
    self.waypoint_misses = 0  # off-route reroutes since the target was set or a waypoint passed
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step

    self.recompute_hold = 0
    self.reroute_counter = 0
    self.reroutes = 0  # off-route reroutes in a row, since the car was last on its route
    self.request_position = None  # where the last route was asked
    self.arrived = False

    self.last_speed = 0.
    self.route_time = None  # monotonic s when a Baidu route was asked, for its traffic
    self.traffic = []  # its jams: (start, length, status), m along the route from its start

    self.baidu_keys = baidu.load_keys()
    self.baidu_daily = baidu.DailyCount(BAIDU_COUNT_FILE if self.baidu_keys is not None else None)

    self.mapbox_token = os.environ.get("MAPBOX_TOKEN")
    self.api = Api(self.params.get("DongleId")) if self.mapbox_token is None else None
    self.mapbox_host = "https://api.mapbox.com" if self.mapbox_token is not None else API_HOST.rstrip('/') + '/v1/nav'

  def update(self):
    self.sm.update(0)

    source = latest_gps_source(self.sm)
    if source is not None:
      gps = self.sm[source]
      self.last_position, self.last_bearing, self.last_speed = (gps.longitude, gps.latitude), gps.bearingDeg, gps.speed
    route = self.route
    try:
      self.recompute_route(gps_ok=source is not None)
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
    # mapsd dedupes repeats, so a periodic resend is how a restarted mapsd gets the route
    if self.route is not route or self.sm.frame % ROUTE_RESEND_PERIOD == 0:
      self.send_route()

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get("NavDestination")
    if destination is None:
      self.target, self.waypoints = None, []
      self.clear_route()
      self.recompute_hold = 0
      return
    new_destination = (destination['longitude'], destination['latitude'])
    waypoints = as_waypoints(destination.get('waypoints'))
    if (new_destination, waypoints) != self.target:
      self.target, self.waypoints, self.waypoint_misses = (new_destination, waypoints), list(waypoints), 0
    self.pass_waypoints()

    # Don't recompute when GPS drifts in tunnels
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    refresh = self.route_time is not None and time.monotonic() - self.route_time >= BAIDU_REFRESH and self.last_speed >= BAIDU_REFRESH_SPEED
    new = new_destination != self.nav_destination or waypoints != self.nav_waypoints
    reroute = self.should_recompute() and (self.step_idx is None or self.reroute_worthwhile())
    if (reroute or new or refresh) and self.recompute_hold <= 0:
      if new or refresh or self.step_idx is None:
        self.recompute_hold = RECOMPUTE_HOLD
        self.reroutes = 0
      else:
        self.recompute_hold = REROUTE_BACKOFF[min(self.reroutes, len(REROUTE_BACKOFF) - 1)]
        self.reroutes += 1
        if self.waypoints:
          self.waypoint_misses += 1
          if self.waypoint_misses >= WAYPOINTS_GIVEN_UP:  # left the picked route again after a reroute back to it: another way
            self.waypoints = []
      self.request_position = self.last_position
      self.nav_waypoints = waypoints
      self.calculate_route(new_destination)
      self.reroute_counter = 0

  def pass_waypoints(self):
    """Drop the waypoints the car has come to, and those before them."""
    assert self.last_position is not None
    for i in range(len(self.waypoints) - 1, -1, -1):
      lat, lon = self.waypoints[i]
      if np.linalg.norm(local_meters(self.last_position, (lon, lat))) < WAYPOINT_REACHED:
        self.waypoints, self.waypoint_misses = self.waypoints[i + 1:], 0
        return

  def reroute_worthwhile(self) -> bool:
    if self.last_speed >= REROUTE_MIN_SPEED or self.request_position is None:
      return True
    return bool(np.linalg.norm(local_meters(self.last_position, self.request_position)) >= REROUTE_MIN_MOVED)

  def calculate_route(self, destination):
    assert self.last_position is not None
    cloudlog.info(f"Calculating route {self.last_position} -> {destination}")
    self.arrived = False
    self.nav_destination = destination
    if self.baidu_keys is not None:
      self.calculate_baidu_route(destination)
      return

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    token = self.mapbox_token
    if token is None:
      token = self.api.get_token()

    params = {
      'access_token': token,
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
    }
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90;"

    coords_str = ';'.join(f'{lon},{lat}' for lon, lat in (self.last_position, destination))
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      if len(r['routes']):
        self.route = r['routes'][0]['legs'][0]['steps']
        self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
        self.step_idx = 0
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      self.clear_route()

  def calculate_baidu_route(self, destination):
    assert self.last_position is not None and self.baidu_keys is not None
    now = baidu.unix_now()
    if self.baidu_daily.left("nav", now) <= 0:
      cloudlog.warning("navd: Baidu route budget used up today")
      self.clear_route()
      self.recompute_hold = BAIDU_RETRY
      return
    lon, lat = self.last_position
    course = math.radians(self.last_bearing or 0.)
    query = baidu.signed_get(baidu.ROUTE_PATH, baidu.route_params(lat, lon, course, self.last_speed, (destination[1], destination[0]),
                                                                       self.waypoints),
                             *self.baidu_keys)
    self.baidu_daily.add("nav", now)
    asked = time.monotonic()
    try:
      parsed = baidu_route.parse(baidu.http_get(query))
    except Exception as e:  # only the type: the message could hold the signed URL
      cloudlog.warning(f"navd: Baidu route failed ({type(e).__name__})")
      parsed = None
    if parsed is None:
      cloudlog.warning("Got empty route response")
      self.clear_route()
      self.recompute_hold = BAIDU_RETRY
      return
    self.route, _, self.traffic = parsed
    self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
    self.step_idx = 0
    self.route_time = asked

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)

    if self.step_idx is None:
      msg.valid = False
      self.pm.send('navInstruction', msg)
      return

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    step = self.route[self.step_idx]
    along_geometry, _ = along_path(self.route_geometry[self.step_idx], self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    set_banner(msg.navInstruction, banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)

    # Total remaining distance and time: the rest of this step, then every step after it
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    future = self.route[self.step_idx + 1:]
    msg.navInstruction.distanceRemaining = step['distance'] * remaining + sum(s['distance'] for s in future)
    msg.navInstruction.timeRemaining = step['duration'] * remaining + sum(s['duration'] for s in future)

    self.pm.send('navInstruction', msg)

    # Transition to next route segment, once the car is on it: off the route "along" keeps growing with the distance from it
    passed = distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD
    if self.step_idx + 1 < len(self.route):
      if passed and along_path(self.route_geometry[self.step_idx + 1], self.last_position)[1] <= REROUTE_DISTANCE:
        self.step_idx += 1
        self.recompute_hold = 0
      return
    to_destination = self.distance_to_destination()
    if passed and to_destination <= ARRIVE_DISTANCE and not self.arrived:
      cloudlog.info("Destination reached")
      self.arrived = True
    # Clear route if driving away from destination
    if self.arrived and to_destination > REROUTE_DISTANCE:
      self.params.remove("NavDestination")
      self.clear_route()

  def distance_to_destination(self) -> float:
    """From the destination or the route's end, the nearer: a destination off the road network is reached where the road ends."""
    assert self.route_geometry is not None
    ends = np.array([self.nav_destination, self.route_geometry[-1][-1]])
    return float(np.linalg.norm(local_meters(ends, self.last_position), axis=1).min())

  def send_route(self):
    coords = np.concatenate(self.route_geometry) if self.route_geometry is not None else np.empty((0, 2))
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in coords.tolist()]
    self.pm.send('navRoute', msg)
    if self.baidu_keys is not None:
      self.send_traffic(coords)

  def send_traffic(self, coords):
    """The Baidu route with its traffic for roadeventd, empty without one."""
    msg = messaging.new_message('navRouteIC', valid=True)
    if self.route_time is not None and len(coords):
      msg.navRouteIC.requestTime = int(self.route_time * 1e9)
      msg.navRouteIC.coordinates = [{'latitude': lat, 'longitude': lon} for lon, lat in coords.tolist()]
      msg.navRouteIC.jams = [{'start': start, 'length': length, 'status': status} for start, length, status in self.traffic]
    self.pm.send('navRouteIC', msg)

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None
    self.route_time = None
    self.traffic = []

  def should_recompute(self):
    if self.step_idx is None:
      return True

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    # Don't recompute at the destination
    if self.step_idx == len(self.route) - 1 and (self.arrived or self.distance_to_destination() <= ARRIVE_DISTANCE):
      return False

    # TODO: Check for going wrong way in segment
    if along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
      self.reroutes = 0  # back on the route
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'navRouteIC'])
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
