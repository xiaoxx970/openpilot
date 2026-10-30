from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.rain_follow import RainFollow

Personality = log.LongitudinalPersonality


class JamFollow:
  """Relaxed while roadeventd asks for it for a traffic jam ahead (prompts.JamAhead), when that is farther than the stored
  personality. It ends with the jam's request: the car reaching the jam, the jam going, its data going stale, Baidu or the
  road prompts off. A distance button changed meanwhile keeps it off for the rest of that jam; the next jam switches again.
  """
  def __init__(self):
    self.requested = False
    self.touched = False  # distance buttons used during this jam

  def active(self, personality: int) -> bool:
    return self.requested and not self.touched and personality < Personality.relaxed

  def touch(self) -> None:
    self.touched = self.requested

  def update(self, requested: bool) -> None:
    if not requested:
      self.touched = False
    self.requested = requested


class FollowDistance:
  """The following distance to drive with, on top of the stored personality: rain's while it rains (RainFollow), relaxed
  while a traffic jam is ahead (JamFollow), and with both on the farther one, rain's. Neither is ever stored. Positions as
  the distance buttons step them: the personalities, then rain's past relaxed while it rains.

  A distance button changed is the driver's choice over both, for as long as what was on at the press lasts: the rest of
  that rain (stepping back to rain's position turns it on again, as RainFollow does) and the rest of that jam. A new rain
  or a new jam afterwards switches again. The distance is shown whenever the one driven with changes, so a jam coming or
  going under rain shows nothing, and rain ending during a jam shows the jam's.
  """
  NONE, RAIN, JAM = 0, 1, 2

  def __init__(self):
    self.rain = RainFollow()
    self.jam = JamFollow()

  def mode(self, personality: int) -> int:
    if self.rain.active:
      return self.RAIN
    if self.jam.active(personality):
      return self.JAM
    return self.NONE

  def personality(self, personality: int) -> int:
    """The personality the planner and the HUD use: relaxed for a jam. With rain the stored one, rainMode adds the rest."""
    return Personality.relaxed if self.mode(personality) == self.JAM else personality

  def position(self, personality: int) -> int:
    return self.rain.position(self.personality(personality))

  def top(self, rain_enabled: bool) -> int:
    return self.rain.top(rain_enabled)

  def select(self, position: int, personality: int) -> int:
    """A distance button picked position; returns the personality to store."""
    self.jam.touch()
    return self.rain.select(position, personality)

  def update(self, raining: bool, rain_enabled: bool, jam_requested: bool, personality: int) -> tuple[int, bool]:
    """Returns the personality to store and whether to show the following distance."""
    before = self.position(personality), self.mode(personality)
    personality, _ = self.rain.update(raining, rain_enabled, personality)
    self.jam.update(jam_requested)
    return personality, (self.position(personality), self.mode(personality)) != before
