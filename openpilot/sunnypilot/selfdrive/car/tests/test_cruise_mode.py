import time

from opendbc.car.structs import car
from openpilot.common.parameterized import parameterized_class
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car import cruise_helpers
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import CruiseHelper, DISTANCE_LONG_PRESS

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


@parameterized_class(('openpilot_longitudinal',), [(True,)])
class TestCruiseHelper(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.CP = car.CarParams(openpilotLongitudinalControl=self.openpilot_longitudinal)
    self.cruise_helper = CruiseHelper(self.CP)
    self.cruise_helper.experimental_mode_switched = False
    self.events = Events()

  def reset(self, dec):
    self.params.put_bool("DynamicExperimentalControl", dec, block=True)
    for _ in range(2):
      CS = car.CarState(cruiseState={"available": False})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=False)]
      self.cruise_helper.experimental_mode_switched = False
      self.cruise_helper.update(CS, self.events, False)

  def wait_for_dec(self, expected, timeout=1.):
    end = time.monotonic() + timeout
    while self.params.get_bool("DynamicExperimentalControl") != expected:
      if time.monotonic() > end:
        return False
      time.sleep(0.01)
    return True

  def test_gap_adjust_cruise_long_press_toggles_dec(self) -> None:
    for pressed in (True, False):
      for dec in (True, False):
        for experimental_mode in (True, False):
          self.reset(dec)
          self.params.put_bool("ExperimentalMode", experimental_mode, block=True)
          toggled = not dec if pressed else dec

          for i in range(DISTANCE_LONG_PRESS):
            CS = car.CarState(cruiseState={"available": True})
            CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
            self.cruise_helper.update(CS, self.events, experimental_mode)

          # DEC is toggled (the param write is asynchronous), experimental mode is left alone
          if pressed:
            assert cruise_helpers.last_dec_switch == toggled
          assert self.wait_for_dec(toggled)
          assert self.params.get_bool("ExperimentalMode") == experimental_mode
          assert self.cruise_helper.experimental_mode_switched is pressed

          # keep holding the button after switching
          for _ in range(DISTANCE_LONG_PRESS):
            CS = car.CarState(cruiseState={"available": True})
            CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
            self.cruise_helper.update(CS, self.events, experimental_mode)

          # not toggled again
          assert self.wait_for_dec(toggled)
          assert self.cruise_helper.experimental_mode_switched is pressed

  def test_gap_adjust_cruise_short_press_keeps_dec(self) -> None:
    for pressed in (True, False):
      for dec in (True, False):
        self.reset(dec)

        for i in range(DISTANCE_LONG_PRESS - 1):
          CS = car.CarState(cruiseState={"available": True})
          CS.buttonEvents = [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)] if i == 0 else []
          self.cruise_helper.update(CS, self.events, True)

        assert self.wait_for_dec(dec)
        assert self.cruise_helper.experimental_mode_switched is False
