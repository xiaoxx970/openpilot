#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, get_speed_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
# Cruise accel and decel both follow the table above, scaled by longitudinal personality
A_CRUISE_MAX_SCALE = {
  log.LongitudinalPersonality.relaxed: 0.75,
  log.LongitudinalPersonality.standard: 1.0,
  log.LongitudinalPersonality.aggressive: 1.0,
}
A_CRUISE_MIN_SCALE = {
  log.LongitudinalPersonality.relaxed: 0.75,
  log.LongitudinalPersonality.standard: 1.0,
  log.LongitudinalPersonality.aggressive: 1.25,
}
# With a lead ahead, cruise may close the extra gap at most this much faster than the lead,
# and only as fast as would close that gap over LEAD_CLOSE_TIME
LEAD_CLOSE_MAX_SPEED = 5. * CV.KPH_TO_MS
LEAD_CLOSE_TIME = 20.
# When that cap is what limits cruise, only ease off toward it; real braking for the lead stays with the MPC
LEAD_CLOSE_MIN_ACCEL = -0.3
# Closing speed never drops below this, so in steady following cruise sits just above the MPC instead of fighting it
LEAD_CLOSE_MIN_SPEED = 0.3
# Low-pass on the capped target against radar noise, and how long a lead dropout keeps the filter state
LEAD_CLOSE_FILTER_TAU = 2.
LEAD_CLOSE_LOST_TIME = 1.
# Track the capped target with a 4 s time constant instead of cruise's 1 s, so lead noise barely reaches the command
LEAD_CLOSE_ACCEL_TIME = 4.
# In calm driving the accel command may reverse its trend at most once per ACCEL_HOLD_TIME; a request that
# moves further than ACCEL_HOLD_OVERRIDE from the held value always gets through
ACCEL_HOLD_TIME = 5.
ACCEL_HOLD_OVERRIDE = 0.3
# While calm the released command slews at most this fast, so letting go of a hold is not a step either;
# a change beyond ACCEL_HOLD_OVERRIDE is a real demand and slews at the faster rate
ACCEL_HOLD_JERK = 0.5
ACCEL_HOLD_OVERRIDE_JERK = 2.
# Calm means: not slow, not inside this share of the desired following distance, lead not braking, time to collision long
ACCEL_HOLD_MIN_SPEED = 5.
ACCEL_HOLD_MIN_DISTANCE_RATIO = 0.8
ACCEL_HOLD_LEAD_BRAKE = -0.5
ACCEL_HOLD_MIN_TTC = 6.
# Any planner request braking harder than this is never calm, e.g. stopping for a light or a cut-in
ACCEL_HOLD_MAX_BRAKE = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


class LeadClosingLimit:
  """Caps the cruise target while a lead is present.

  Cruise may run at most LEAD_CLOSE_MAX_SPEED faster than the lead, scaled down as the gap beyond the
  MPC's desired distance shrinks, but never below LEAD_CLOSE_MIN_SPEED. That floor keeps cruise just above
  the MPC in steady following, so the MPC alone shapes the follow there. The target is low-passed so radar
  noise on vLead does not reach the accel command.
  """
  def __init__(self, dt):
    self.dt = dt
    self.active = False
    self.lead_lost_time = 0.
    self.v_target = FirstOrderFilter(0., LEAD_CLOSE_FILTER_TAU, dt)

  def update(self, v_cruise, v_ego, lead, personality):
    # Returns the cruise target and whether the lead cap is what set it
    if not lead.present:
      self.lead_lost_time += self.dt
      if self.lead_lost_time > LEAD_CLOSE_LOST_TIME:
        self.active = False
      return v_cruise, False
    self.lead_lost_time = 0.

    desired_distance = get_safe_obstacle_distance(v_ego, get_T_FOLLOW(personality)) - get_stopped_equivalence_factor(lead.vLead)
    close_speed = float(np.clip((lead.dRel - desired_distance) / LEAD_CLOSE_TIME, LEAD_CLOSE_MIN_SPEED, LEAD_CLOSE_MAX_SPEED))
    if not self.active:
      self.active = True
      self.v_target.x = lead.vLead + close_speed
    v_lead_cap = self.v_target.update(lead.vLead + close_speed)
    return (v_lead_cap, True) if v_lead_cap < v_cruise else (v_cruise, False)


class AccelDirectionHold:
  """Stops small back-and-forth changes in the accel command.

  Holding a speed downhill, the cruise loop flipped the command around zero every 1-2 s and the
  powertrain amplified each flip into a felt throttle/drag change. While calm, once the command has
  turned, it may not turn back for ACCEL_HOLD_TIME unless the request moves ACCEL_HOLD_OVERRIDE away.
  """
  def __init__(self, dt):
    self.dt = dt
    self.reset()

  def reset(self):
    self.held = None
    self.output = None
    self.direction = 0
    self.since_reversal = ACCEL_HOLD_TIME

  def update(self, accel, calm):
    self.since_reversal += self.dt
    if self.held is None:
      self.held = self.output = accel
      return accel

    delta = accel - self.held
    if self.direction == 0 or delta * self.direction >= 0:
      if delta != 0. and self.direction == 0:
        self.direction = 1 if delta > 0 else -1
      self.held = accel
    elif not calm or abs(delta) > ACCEL_HOLD_OVERRIDE or self.since_reversal >= ACCEL_HOLD_TIME:
      # outside calm the request always passes, but the turn still counts so calm cannot allow another right after
      self.direction = -self.direction
      self.since_reversal = 0.
      self.held = accel

    if calm:
      step = (ACCEL_HOLD_OVERRIDE_JERK if abs(self.held - self.output) > ACCEL_HOLD_OVERRIDE else ACCEL_HOLD_JERK) * self.dt
      self.output = float(np.clip(self.held, self.output - step, self.output + step))
    else:
      self.output = self.held
    return self.output


def is_calm(v_ego, accel_request, lead, personality, should_stop, fcw):
  if v_ego < ACCEL_HOLD_MIN_SPEED or accel_request < ACCEL_HOLD_MAX_BRAKE or should_stop or fcw:
    return False
  if not lead.present:
    return True
  desired_distance = get_safe_obstacle_distance(v_ego, get_T_FOLLOW(personality)) - get_stopped_equivalence_factor(lead.vLead)
  closing_speed = v_ego - lead.vLead
  if lead.dRel < ACCEL_HOLD_MIN_DISTANCE_RATIO * desired_distance or lead.aLeadK < ACCEL_HOLD_LEAD_BRAKE:
    return False
  return closing_speed <= 0. or lead.dRel / closing_speed > ACCEL_HOLD_MIN_TTC


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_lead_distance(radarState):
  if radarState.leadOne.present and (not radarState.leadTwo.present or radarState.leadOne.dRel < radarState.leadTwo.dRel):
    return radarState.leadOne.dRel
  if radarState.leadTwo.present:
    return radarState.leadTwo.dRel
  return 0


def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                     personality=log.LongitudinalPersonality.standard, lead_capped=False):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego) * A_CRUISE_MAX_SCALE.get(personality, 1.0)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  min_accel = -get_max_accel(v_ego) * A_CRUISE_MIN_SCALE.get(personality, 1.0)
  if lead_capped:
    target_accel = np.clip((v_cruise - v_ego) / LEAD_CLOSE_ACCEL_TIME, max(min_accel, LEAD_CLOSE_MIN_ACCEL), max_accel)
  else:
    target_accel = np.clip(v_cruise - v_ego, min_accel, max_accel)
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.lead_closing_limit = LeadClosingLimit(self.dt)
    self.accel_hold = AccelDirectionHold(self.dt)
    self.output_a_target = init_a
    self.output_should_stop = False
    self.output_v_target = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    is_e2e = self.is_e2e(sm)

    v_cruise, lead_capped = self.lead_closing_limit.update(v_cruise, v_ego, sm['radarState'].leadOne, sm['selfdriveState'].personality)
    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, sm['selfdriveState'].personality, lead_capped)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)
    if reset_state:
      self.accel_hold.reset()
    calm = is_calm(v_ego, float(self.output_a_target), sm['radarState'].leadOne, sm['selfdriveState'].personality, self.output_should_stop, self.fcw)
    self.output_a_target = self.accel_hold.update(float(self.output_a_target), calm)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0
    self.output_v_target = get_speed_from_plan(self.v_desired_trajectory, CONTROL_N_T_IDX, action_t=action_t)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    # infiniteCable longitudinal plan extension
    plan_ic_send = messaging.new_message('longitudinalPlanIC')
    plan_ic_send.valid = sm.all_checks()
    plan_ic_send.longitudinalPlanIC.leadDistance = get_lead_distance(sm['radarState'])
    plan_ic_send.longitudinalPlanIC.vTarget = float(self.output_v_target)
    pm.send('longitudinalPlanIC', plan_ic_send)

    self.publish_longitudinal_plan_sp(sm, pm)
