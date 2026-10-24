#!/usr/bin/env python3
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, get_speed_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
# Set-speed decel follows this table at the current speed, scaled by longitudinal personality. Accel is
# left to ACCEL_MAX: the e2e candidate is always in the min() and shapes speeding up.
A_CRUISE_DECEL_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
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
# The car reports whether the engine delivers torque and whether ACC is braking; between the two it coasts.
# A cruise command that flips across that band every second or two (holding a speed downhill) is felt as a
# throttle/drag lurch each time; one that moves within the throttle side (holding a set speed on the flat) is not.
COAST_BAND_PRIOR_WIDTH = 0.3    # m/s^2 from throttle-off down to brake-on until the car has shown both edges
COAST_BAND_LEARN_RATE = 0.2     # blend of each observed flag edge into the band position
COAST_BAND_MAX_OFFSET = 0.3     # m/s^2 an observed edge may sit from the pitch-based coast estimate to count
COAST_BAND_HOLD_TIME = 5.       # s after a crossing before the opposite side may be entered again
COAST_BAND_OVERRIDE = 0.3       # m/s^2 past the edge a blocked request is still let through: a real demand
COAST_BAND_REST = 0.02          # m/s^2 back from the edge a blocked change rests at
COAST_BAND_JERK = 0.5           # m/s^3 slewing onto an edge and back off it
COAST_BAND_OVERRIDE_JERK = 2.   # m/s^3 when leaving a hold for a demand beyond COAST_BAND_OVERRIDE
COAST_BAND_MIN_SPEED = 5.       # m/s; slower than that stopping and creeping shape the command
COAST_BAND_FLAG_DEBOUNCE = 0.5  # s a car flag must have held its previous value for its edge to count
# The learnt band is the car's (drag, rolling, gearbox) plus this drive's load; carry half of it into the next drive
COAST_BAND_PARAM = "CoastBandCalibration"
COAST_BAND_KEEP = 0.5
COAST_BAND_SAVE_INTERVAL = 120.  # s between saves, and only after new edges; the write is non-blocking
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

def get_max_decel(v_ego):
  return np.interp(v_ego, J_CRUISE_BP, A_CRUISE_DECEL_VALS)


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


class CoastBandHold:
  """Lets the set-speed command change throttle/coast/brake state at most once per COAST_BAND_HOLD_TIME.

  The car coasts between its throttle-off and brake-on points. Each move between throttle, coasting and
  braking is a felt lurch; a command that stays on one side (holding a set speed on the flat lives on
  the throttle side) is passed through untouched. A blocked change rests the output just short of the
  edge it tried to cross, so the car stays in the state it is in; asked to go from throttle straight to
  braking it takes the step into coasting and holds the second. A request COAST_BAND_OVERRIDE past the
  edge is a real demand and always passes. The band sits at the pitch-based coast estimate and is
  corrected from the car's own flag edges, so load, grade and drag move it.
  """
  def __init__(self, dt, saved=None):
    self.dt = dt
    self.coast_offset = 0.                   # throttle-off edge relative to the coast estimate
    self.band_width = COAST_BAND_PRIOR_WIDTH  # throttle-off edge minus brake-on edge
    self.learn_count = 0
    self.last_edges = (-COAST_BAND_PRIOR_WIDTH, 0.)
    self.load(saved)
    self.reset()

  def load(self, saved):
    # start from part of the last drive's band; anything malformed or out of range is ignored
    try:
      offset, width = float(saved["offset"]), float(saved["width"])
    except (TypeError, KeyError, ValueError):
      return
    if abs(offset) <= COAST_BAND_MAX_OFFSET and 0.05 <= width <= 2. * COAST_BAND_PRIOR_WIDTH:
      self.coast_offset = COAST_BAND_KEEP * offset
      self.band_width = COAST_BAND_PRIOR_WIDTH + COAST_BAND_KEEP * (width - COAST_BAND_PRIOR_WIDTH)

  def calibration(self):
    return {"offset": round(self.coast_offset, 4), "width": round(self.band_width, 4), "edges": self.learn_count}

  def reset(self):
    self.state = None                        # 1 throttle, 0 coasting, -1 braking
    self.since_change = COAST_BAND_HOLD_TIME
    self.output = None
    self.holding = False
    self.flag_prev = {'idle': None, 'brake': None}
    self.flag_age = {'idle': 0., 'brake': 0.}

  def edges(self, accel_coast):
    hi = accel_coast + self.coast_offset
    return hi - self.band_width, hi

  def _learn(self, engine_idle, brake_requested, accel_coast):
    # the command we had sent when a flag rose is where that edge sits
    sent = self.output
    for name, flag in (('idle', engine_idle), ('brake', brake_requested)):
      prev = self.flag_prev[name]
      if prev is not None and flag != prev:
        if flag and sent is not None and self.flag_age[name] >= COAST_BAND_FLAG_DEBOUNCE:
          if name == 'idle':
            offset = sent - accel_coast
            if abs(offset) <= COAST_BAND_MAX_OFFSET:
              self.coast_offset += COAST_BAND_LEARN_RATE * (offset - self.coast_offset)
              self.learn_count += 1
          else:
            width = accel_coast + self.coast_offset - sent
            if 0.05 <= width <= 2. * COAST_BAND_PRIOR_WIDTH:
              self.band_width += COAST_BAND_LEARN_RATE * (width - self.band_width)
              self.learn_count += 1
        self.flag_age[name] = 0.
      else:
        self.flag_age[name] += self.dt
      self.flag_prev[name] = flag

  def update(self, accel, accel_coast, active, engine_idle, brake_requested):
    self._learn(engine_idle, brake_requested, accel_coast)
    self.since_change += self.dt
    lo, hi = self.last_edges = self.edges(accel_coast)
    state = 1 if accel > hi else (-1 if accel < lo else 0)
    target = accel
    if active and self.state is not None and state != self.state:
      down = state < self.state
      edge = (lo if state == -1 else hi) if down else (hi if state == 1 else lo)  # the edge crossed last
      if self.since_change >= COAST_BAND_HOLD_TIME or abs(accel - edge) >= COAST_BAND_OVERRIDE:
        self.since_change = 0.
      else:
        target = edge + (COAST_BAND_REST if down else -COAST_BAND_REST)  # rest just short of that edge
        if self.state != 0 and state != 0:
          state = 0  # throttle straight to brake or back: the step into coasting is taken, the second is held
          self.since_change = 0.
        else:
          state = self.state
    self.state = state

    holding = target != accel
    if self.output is None or not (active and (holding or self.holding)):
      self.output = accel
    else:
      jerk = COAST_BAND_OVERRIDE_JERK if abs(target - self.output) > COAST_BAND_OVERRIDE else COAST_BAND_JERK
      self.output = float(np.clip(target, self.output - jerk * self.dt, self.output + jerk * self.dt))
    self.holding = holding or abs(self.output - accel) > 1e-3
    return self.output


def get_coast_accel(pitch):
  # where the coast band starts before the car has shown its own edges; kept from the chill planner upstream removed
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def get_lead_distance(radarState):
  if radarState.leadOne.present and (not radarState.leadTwo.present or radarState.leadOne.dRel < radarState.leadTwo.dRel):
    return radarState.leadOne.dRel
  if radarState.leadTwo.present:
    return radarState.leadTwo.dRel
  return 0


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt, personality=log.LongitudinalPersonality.standard, lead_capped=False):
  min_accel = -get_max_decel(v_ego) * A_CRUISE_MIN_SCALE.get(personality, 1.0)
  if lead_capped:
    target_accel = np.clip((v_cruise - v_ego) / LEAD_CLOSE_ACCEL_TIME, max(min_accel, LEAD_CLOSE_MIN_ACCEL), ACCEL_MAX)
  else:
    target_accel = np.clip(v_cruise - v_ego, min_accel, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.lead_closing_limit = LeadClosingLimit(self.dt)
    self.params = Params()
    try:
      saved = self.params.get(COAST_BAND_PARAM)
    except Exception:
      saved = None
    self.coast_band_hold = CoastBandHold(self.dt, saved)
    self.coast_band_saved_count = self.coast_band_hold.learn_count
    self.coast_band_save_timer = 0.
    self.output_a_target = init_a
    self.output_should_stop = False
    self.output_v_target = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

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
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
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

    v_cruise, lead_capped = self.lead_closing_limit.update(v_cruise, v_ego, sm['radarState'].leadOne, sm['selfdriveState'].personality)
    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt, sm['selfdriveState'].personality, lead_capped)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    # only the set-speed loop flip-flops between throttle and coasting; lead and e2e requests pass untouched
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX
    if reset_state:
      self.coast_band_hold.reset()
    cs_ic = sm['carStateIC']
    coast_gate = (self.mpc.source == LongitudinalPlanSource.cruise and v_ego > COAST_BAND_MIN_SPEED and
                  accel_coast < ACCEL_MAX and not self.fcw and not self.output_should_stop)
    self.output_a_target = self.coast_band_hold.update(float(self.output_a_target), accel_coast, coast_gate,
                                                       cs_ic.engineIdle, cs_ic.brakeRequested)
    self.coast_band_save_timer += self.dt
    if self.coast_band_save_timer >= COAST_BAND_SAVE_INTERVAL and self.coast_band_hold.learn_count != self.coast_band_saved_count:
      self.coast_band_save_timer = 0.
      self.coast_band_saved_count = self.coast_band_hold.learn_count
      self.params.put(COAST_BAND_PARAM, self.coast_band_hold.calibration())  # non-blocking

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
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    # infiniteCable longitudinal plan extension
    plan_ic_send = messaging.new_message('longitudinalPlanIC')
    plan_ic_send.valid = sm.all_checks()
    plan_ic_send.longitudinalPlanIC.leadDistance = get_lead_distance(sm['radarState'])
    plan_ic_send.longitudinalPlanIC.vTarget = float(self.output_v_target)
    hold = self.coast_band_hold
    plan_ic_send.longitudinalPlanIC.coastBandLow, plan_ic_send.longitudinalPlanIC.coastBandHigh = (float(e) for e in hold.last_edges)
    plan_ic_send.longitudinalPlanIC.coastBandOffset = float(hold.coast_offset)
    plan_ic_send.longitudinalPlanIC.coastBandWidth = float(hold.band_width)
    plan_ic_send.longitudinalPlanIC.coastBandLearnCount = hold.learn_count
    plan_ic_send.longitudinalPlanIC.coastBandHolding = bool(hold.holding)
    pm.send('longitudinalPlanIC', plan_ic_send)

    self.publish_longitudinal_plan_sp(sm, pm)
