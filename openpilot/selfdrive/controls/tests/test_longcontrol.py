from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, RESTOP_WINDOW_FRAMES, START_DEBOUNCE_FRAMES, long_control_state_trans


class TestLongControlStateTransition(OpenpilotTestCase):

  def test_stay_stopped(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid


class TestLongControlStartDebounce(OpenpilotTestCase):

  def make(self):
    CP = car.CarParams.new_message()
    CP.longitudinalTuning.kiBP = [0.]
    CP.longitudinalTuning.kiV = [0.]
    LoC = LongControl(CP, custom.CarParamsSP.new_message())
    LoC.long_control_state = LongCtrlState.stopping
    return LoC

  @staticmethod
  def CS(standstill):
    return SimpleNamespace(brakePressed=False, cruiseState=SimpleNamespace(standstill=False), standstill=standstill,
                           aEgo=0.0, vEgo=0.0 if standstill else 1.0)

  def update(self, LoC, should_stop, standstill=True):
    LoC.update(True, self.CS(standstill), 0.2, should_stop=should_stop, accel_limits=(-3.5, 2.0))
    return LoC.long_control_state

  def test_first_pull_away_is_not_delayed(self):
    LoC = self.make()
    assert self.update(LoC, False) == LongCtrlState.pid

  def test_flickering_start_keeps_the_hold(self):
    LoC = self.make()
    assert self.update(LoC, False) == LongCtrlState.pid
    assert self.update(LoC, True) == LongCtrlState.stopping
    # after the first fall back, short go requests keep the hold
    for i in range(10 * START_DEBOUNCE_FRAMES):
      assert self.update(LoC, (i // 25) % 2 == 0) == LongCtrlState.stopping

  def test_steady_start_releases_after_a_fall_back(self):
    LoC = self.make()
    self.update(LoC, False)
    self.update(LoC, True)
    for _ in range(START_DEBOUNCE_FRAMES - 1):
      assert self.update(LoC, False) == LongCtrlState.stopping
    assert self.update(LoC, False) == LongCtrlState.pid

  def test_debounce_ends_after_the_window(self):
    LoC = self.make()
    self.update(LoC, False)
    self.update(LoC, True)
    for _ in range(RESTOP_WINDOW_FRAMES):
      self.update(LoC, True)
    assert self.update(LoC, False) == LongCtrlState.pid

  def test_no_debounce_while_moving(self):
    LoC = self.make()
    self.update(LoC, False)
    self.update(LoC, True, standstill=False)
    assert self.update(LoC, False, standstill=False) == LongCtrlState.pid
