from opendbc.car.structs import car
from openpilot.selfdrive.car.distance_display import DISTANCE_DISPLAY_FRAMES, DistanceDisplay

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


class TestDistanceDisplay:
  def setup_method(self):
    self.dd = DistanceDisplay()

  def press(self, btn, available=True):
    """Returns (cycle, step, passed-through events) summed over press and release."""
    cycle, step, remaining = False, 0, []
    for pressed in (True, False):
      remaining += self.dd.update([ButtonEvent(type=btn, pressed=pressed)], available)
      cycle |= self.dd.cycle
      step += self.dd.step
    return cycle, step, remaining

  def idle(self, frames):
    for _ in range(frames):
      self.dd.update([], True)

  def test_first_press_only_shows(self):
    assert self.press(ButtonType.gapAdjustCruise)[0] is False
    assert self.dd.is_open
    assert self.press(ButtonType.gapAdjustCruise)[0] is True

  def test_closes_after_display_time(self):
    self.press(ButtonType.gapAdjustCruise)
    self.idle(DISTANCE_DISPLAY_FRAMES - 1)
    assert self.dd.is_open
    self.idle(1)
    assert not self.dd.is_open
    assert self.press(ButtonType.gapAdjustCruise)[0] is False

  def test_plus_minus_step_only_while_shown(self):
    cycle, step, remaining = self.press(ButtonType.accelCruise)
    assert step == 0 and len(remaining) == 2

    self.press(ButtonType.gapAdjustCruise)
    cycle, step, remaining = self.press(ButtonType.accelCruise)
    assert step == 1 and remaining == []
    cycle, step, remaining = self.press(ButtonType.decelCruise)
    assert step == -1 and remaining == []

    # every press keeps the display open for another 2 s
    self.idle(DISTANCE_DISPLAY_FRAMES - 2)
    assert self.press(ButtonType.decelCruise)[1] == -1

  def test_unavailable(self):
    self.press(ButtonType.gapAdjustCruise, available=False)
    assert not self.dd.is_open
    assert self.press(ButtonType.gapAdjustCruise)[0] is False

  def test_set_resume_pass_through(self):
    self.press(ButtonType.gapAdjustCruise)
    for btn in (ButtonType.setCruise, ButtonType.resumeCruise):
      assert len(self.press(btn)[2]) == 2
