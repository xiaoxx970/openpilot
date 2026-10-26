from openpilot.cereal import log, custom
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.tests.test_mads_steering_mode import make_car_state, make_mads, run_frames
from openpilot.common.test import OpenpilotTestCase

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName


def make_uem_mads(mocker, state=State.disabled):
  mads, sd = make_mads(mocker, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
  mads.unified_engagement_mode = True
  mads.state_machine.state = state
  mads.enabled = state != State.disabled
  return mads, sd


def press_resume(mads, sd, cs, pedal_pressed):
  # selfdrived raises buttonEnable on the SET/RES release, and pedalPressed while the brake is pressed and moving
  sd.events.add(EventName.buttonEnable)
  if pedal_pressed:
    sd.events.add(EventName.pedalPressed)
  mads.update(cs)
  alert_only = sd.events_sp.has(EventNameSP.pedalPressedAlertOnly)
  sd.CS_prev = cs
  sd.events.clear()
  sd.events_sp.clear()
  return alert_only


class TestResumeWhileBraking(OpenpilotTestCase):
  def test_resume_while_braking_and_moving_does_not_engage(self, mocker):
    # 2026-10-25 13:33:55: RES at 7 km/h with the brake held, panda never granted lateral -> Controls Mismatch: Lateral
    mads, sd = make_uem_mads(mocker)
    assert press_resume(mads, sd, make_car_state(brake_pressed=True, v_ego=2.0), pedal_pressed=True)
    assert mads.state_machine.state == State.disabled
    run_frames(mads, sd, make_car_state(brake_pressed=True, v_ego=1.5), n=300)
    assert mads.state_machine.state == State.disabled

  def test_resume_after_brake_release_engages(self, mocker):
    mads, sd = make_uem_mads(mocker)
    press_resume(mads, sd, make_car_state(brake_pressed=True, v_ego=2.0), pedal_pressed=True)
    run_frames(mads, sd, make_car_state(v_ego=2.0), n=10)
    press_resume(mads, sd, make_car_state(v_ego=2.0), pedal_pressed=False)
    assert mads.state_machine.state == State.enabled

  def test_resume_at_standstill_with_brake_held_engages(self, mocker):
    # a brake held at a stop raises no pedalPressed, and panda keeps controls_allowed there too
    mads, sd = make_uem_mads(mocker)
    sd.CS_prev = make_car_state(brake_pressed=True, standstill=True)
    press_resume(mads, sd, make_car_state(brake_pressed=True, standstill=True), pedal_pressed=False)
    assert mads.state_machine.state == State.enabled

  def test_paused_mads_still_resumes(self, mocker):
    # already engaged and only paused: panda still has lateral, so the press resumes as before
    mads, sd = make_uem_mads(mocker, State.paused)
    press_resume(mads, sd, make_car_state(brake_pressed=True, v_ego=8.0), pedal_pressed=True)
    assert mads.state_machine.state == State.enabled
