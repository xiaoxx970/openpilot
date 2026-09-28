from opendbc.car.structs import car

ButtonType = car.CarState.ButtonEvent.Type

# Stock VW hides the distance popup 2 s after the last button is released (route 3f seg 17/19)
DISTANCE_DISPLAY_FRAMES = 200
DISTANCE_STEP = {ButtonType.accelCruise: +1, ButtonType.decelCruise: -1}


class DistanceDisplay:
  """Stock VW distance button semantics, fed once per carState.

  The first distance press only shows the current distance. A distance press while it is shown
  steps it up, and +/- step it up and down instead of changing the set speed. selfdrived and card
  each run one of these on the same carState stream, so they agree on which presses belong to it.
  """
  def __init__(self):
    self.frame = 0
    self.open_until = 0
    self.owned: set = set()  # buttons pressed while shown, handled here until released
    self.gap_press_while_open = False
    self.cycle = False  # a distance press released while shown: step up, wrapping to the shortest
    self.step = 0       # +/- pressed while shown

  @property
  def is_open(self) -> bool:
    return bool(self.owned) or self.frame < self.open_until

  def update(self, button_events, available: bool) -> list:
    """Returns the button events that are not distance presses."""
    self.frame += 1
    self.cycle = False
    self.step = 0

    if not available:
      self.open_until = 0
      self.owned.clear()
      return list(button_events)

    remaining = []
    for b in button_events:
      btn = b.type.raw
      if btn == ButtonType.gapAdjustCruise:
        if b.pressed:
          self.gap_press_while_open = self.is_open
          self.owned.add(btn)
        elif btn in self.owned:
          self.owned.discard(btn)
          self.cycle = self.gap_press_while_open
          self.open_until = self.frame + DISTANCE_DISPLAY_FRAMES
        remaining.append(b)  # still seen by the experimental mode long press
      elif btn in DISTANCE_STEP and b.pressed and self.is_open:
        self.owned.add(btn)
        self.step += DISTANCE_STEP[btn]
      elif btn in DISTANCE_STEP and not b.pressed and btn in self.owned:
        self.owned.discard(btn)
        self.open_until = self.frame + DISTANCE_DISPLAY_FRAMES
      else:
        remaining.append(b)
    return remaining
