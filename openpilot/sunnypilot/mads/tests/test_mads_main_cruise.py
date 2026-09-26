from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.tests.test_mads_steering_mode import make_car_state, make_mads, run_frames
from openpilot.common.test import OpenpilotTestCase

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type


def cruise_state(available, brake_pressed=False, v_ego=10.0, buttons=()):
  cs = make_car_state(brake_pressed=brake_pressed, v_ego=v_ego)
  cs.cruiseState.available = available
  cs.buttonEvents = [structs.CarState.ButtonEvent(pressed=True, type=b) for b in buttons]
  return cs


def make_main_cruise_mads(mocker, state):
  mads, sd = make_mads(mocker, MadsSteeringModeOnBrake.PAUSE)
  mads.main_enabled_toggle = True
  mads.state_machine.state = state
  mads.enabled = state != State.disabled
  mads.active = state == State.enabled
  return mads, sd


class TestMainCruiseEdge(OpenpilotTestCase):
  def test_main_switch_on_enables(self, mocker):
    mads, sd = make_main_cruise_mads(mocker, State.disabled)
    run_frames(mads, sd, cruise_state(False), n=10)
    run_frames(mads, sd, cruise_state(True))
    assert mads.state_machine.state == State.enabled

  def test_hard_brake_recovery_does_not_enable_when_off(self, mocker):
    # cruise drops out under hard braking and returns after the brake is released, nothing pressed
    mads, sd = make_main_cruise_mads(mocker, State.disabled)
    run_frames(mads, sd, cruise_state(True, brake_pressed=True, v_ego=12.0), n=10)
    run_frames(mads, sd, cruise_state(False, brake_pressed=True, v_ego=6.0), n=200)
    run_frames(mads, sd, cruise_state(False, v_ego=1.0), n=100)
    run_frames(mads, sd, cruise_state(True, v_ego=1.3), n=10)
    assert mads.state_machine.state == State.disabled

  def test_hard_brake_recovery_reengages_when_on(self, mocker):
    mads, sd = make_main_cruise_mads(mocker, State.enabled)
    run_frames(mads, sd, cruise_state(True, brake_pressed=True, v_ego=12.0), n=10)
    run_frames(mads, sd, cruise_state(False, brake_pressed=True, v_ego=6.0))
    assert mads.state_machine.state == State.disabled
    run_frames(mads, sd, cruise_state(False, brake_pressed=True, v_ego=6.0), n=200)
    run_frames(mads, sd, cruise_state(False, v_ego=1.0), n=100)
    run_frames(mads, sd, cruise_state(True, v_ego=1.3))
    assert mads.state_machine.state == State.enabled

  def test_main_switch_press_after_brake_drop_enables(self, mocker):
    # VW reports the momentary main switch as cancel
    mads, sd = make_main_cruise_mads(mocker, State.disabled)
    run_frames(mads, sd, cruise_state(True, brake_pressed=True), n=10)
    run_frames(mads, sd, cruise_state(False, brake_pressed=True), n=100)
    run_frames(mads, sd, cruise_state(False, buttons=(ButtonType.cancel,)))
    run_frames(mads, sd, cruise_state(False), n=10)
    run_frames(mads, sd, cruise_state(True))
    assert mads.state_machine.state == State.enabled

  def test_block_only_covers_one_recovery(self, mocker):
    mads, sd = make_main_cruise_mads(mocker, State.disabled)
    run_frames(mads, sd, cruise_state(True, brake_pressed=True), n=10)
    run_frames(mads, sd, cruise_state(False, brake_pressed=True), n=100)
    run_frames(mads, sd, cruise_state(True), n=10)
    assert mads.state_machine.state == State.disabled
    run_frames(mads, sd, cruise_state(False), n=10)
    run_frames(mads, sd, cruise_state(True))
    assert mads.state_machine.state == State.enabled
