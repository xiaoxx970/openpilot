from openpilot.cereal import log, custom
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.sunnypilot.mads.tests.test_mads_steering_mode import make_car_state, make_mads
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName


def step(mads, sd, cs, events=(), events_sp=()):
  for e in events:
    sd.events.add(e)
  for e in events_sp:
    sd.events_sp.add(e)
  mads.update(cs)
  sd.CS_prev = cs
  sd.events.clear()
  sd.events_sp.clear()


def make_enabled_mads(mocker, state=State.enabled):
  mads, sd = make_mads(mocker, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
  mads.state_machine.state = state
  mads.enabled = True
  mads.active = state == State.enabled
  step(mads, sd, make_car_state(v_ego=10.0))  # in drive, nothing pending
  return mads, sd


def user_disable_alerted(sd):
  return any(c.args == (ET.USER_DISABLE,) for c in sd.state_machine.current_alert_types.append.call_args_list)


class TestMadsExit(OpenpilotTestCase):
  def test_neutral_while_moving_disengages(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(v_ego=15.0), events=[EventName.wrongGear])
    assert mads.state_machine.state == State.disabled
    assert user_disable_alerted(sd)

  def test_neutral_at_low_speed_disengages(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(v_ego=1.0), events=[EventName.wrongGear])
    assert mads.state_machine.state == State.disabled

  def test_reverse_from_drive_disengages(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(v_ego=0.5), events=[EventName.wrongGear, EventName.reverseGear])
    assert mads.state_machine.state == State.disabled

  def test_leaving_drive_during_brake_hold_disengages_not_pauses(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(standstill=True), events=[EventName.wrongGear, EventName.brakeHold])
    assert mads.state_machine.state == State.disabled

  def test_paused_mads_disengages_on_leaving_drive(self, mocker):
    mads, sd = make_enabled_mads(mocker, State.paused)
    step(mads, sd, make_car_state(standstill=True), events=[EventName.wrongGear])
    assert mads.state_machine.state == State.disabled

  def test_door_opened_at_standstill_disengages(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(standstill=True), events=[EventName.doorOpen])
    assert mads.state_machine.state == State.disabled
    assert user_disable_alerted(sd)

  def test_seatbelt_unlatched_at_standstill_disengages(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(standstill=True), events=[EventName.seatbeltNotLatched])
    assert mads.state_machine.state == State.disabled

  def test_door_opened_while_moving_still_soft_disables(self, mocker):
    mads, sd = make_enabled_mads(mocker)
    step(mads, sd, make_car_state(v_ego=5.0), events=[EventName.doorOpen])
    assert mads.state_machine.state == State.softDisabling

  def test_enabled_at_park_waits_for_drive(self, mocker):
    # main switch at P after starting: MADS waits paused and engages silently in drive
    mads, sd = make_mads(mocker, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
    step(mads, sd, make_car_state(standstill=True), events=[EventName.wrongGear], events_sp=[EventNameSP.lkasEnable])
    assert mads.state_machine.state == State.paused
    for _ in range(10):
      step(mads, sd, make_car_state(standstill=True), events=[EventName.wrongGear])
    assert mads.state_machine.state == State.paused
    assert not user_disable_alerted(sd)
    step(mads, sd, make_car_state(standstill=True))
    assert mads.state_machine.state == State.enabled
