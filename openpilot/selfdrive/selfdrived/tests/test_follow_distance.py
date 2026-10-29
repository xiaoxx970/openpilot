from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.follow_distance import FollowDistance
from openpilot.selfdrive.selfdrived.rain_follow import RAIN_POSITION

Personality = log.LongitudinalPersonality
NONE, RAIN, JAM = FollowDistance.NONE, FollowDistance.RAIN, FollowDistance.JAM


class TestFollowDistance:
  def setup_method(self):
    self.fd = FollowDistance()
    self.personality = Personality.aggressive
    self.raining = self.jam = False

  def step(self, raining=None, jam=None, rain_enabled=True):
    """One selfdrived cycle; returns whether the following distance is shown."""
    self.raining = self.raining if raining is None else raining
    self.jam = self.jam if jam is None else jam
    self.personality, shown = self.fd.update(self.raining, rain_enabled, self.jam, self.personality)
    return shown

  def press(self, position):
    self.personality = self.fd.select(position, self.personality)

  def cycle(self):
    """A distance button press while the distance is shown, stepped as selfdrived does: up, wrapping to the shortest."""
    position = self.fd.position(self.personality)
    new = (position + 1) % (self.fd.top(True) + 1)
    if new != position:
      self.press(new)

  def driven(self):
    """(position, mode, personality for the planner)"""
    return self.fd.position(self.personality), self.fd.mode(self.personality), self.fd.personality(self.personality)

  # jam alone

  def test_jam_selects_relaxed_until_it_ends(self):
    assert self.step(jam=True)
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)
    assert not self.step()
    assert self.step(jam=False)
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)
    assert self.personality == Personality.aggressive   # never stored

  def test_never_shorter_than_the_stored_personality(self):
    self.personality = Personality.relaxed
    assert not self.step(jam=True)
    assert self.driven() == (Personality.relaxed, NONE, Personality.relaxed)
    self.personality = Personality.standard   # changed in the settings meanwhile
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)

  def test_override_during_a_jam_lasts_until_it_ends(self):
    self.step(jam=True)
    self.cycle()   # relaxed wraps to the shortest
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)
    self.press(Personality.standard)
    assert self.driven() == (Personality.standard, NONE, Personality.standard)
    assert not self.step()
    assert not self.step(jam=False)
    assert self.personality == Personality.standard
    assert self.step(jam=True)   # the next jam switches again
    assert self.driven()[1] == JAM

  def test_override_to_relaxed_is_the_drivers(self):
    self.step(jam=True)
    self.press(Personality.standard)
    self.press(Personality.relaxed)
    assert self.driven() == (Personality.relaxed, NONE, Personality.relaxed)
    assert not self.step(jam=False)
    assert self.personality == Personality.relaxed

  # rain and a jam

  def test_rain_then_jam(self):
    assert self.step(raining=True)
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.aggressive)
    assert not self.step(jam=True)   # rain's is farther
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.aggressive)

  def test_jam_then_rain(self):
    self.step(jam=True)
    assert self.step(raining=True)
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.aggressive)

  def test_jam_ends_under_rain(self):
    self.step(raining=True, jam=True)
    assert not self.step(jam=False)
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.aggressive)

  def test_rain_ends_during_a_jam(self):
    self.step(raining=True, jam=True)
    assert self.step(raining=False)   # back to the jam's, not the stored personality
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)
    assert self.step(jam=False)
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)

  def test_rain_ends_during_a_jam_on_a_stored_relaxed(self):
    self.personality = Personality.relaxed
    self.step(raining=True, jam=True)
    assert self.step(raining=False)
    assert self.driven() == (Personality.relaxed, NONE, Personality.relaxed)
    assert not self.step(jam=False)

  def test_a_flapping_jam_under_rain_shows_nothing(self):
    self.step(raining=True)
    for _ in range(5):
      assert not self.step(jam=True)
      assert not self.step(jam=False)

  def test_whole_episode_shows_each_change_once(self):
    shown = [self.step(raining=True), self.step(jam=True), self.step(), self.step(raining=False), self.step(), self.step(jam=False)]
    assert shown == [True, False, False, True, False, True]

  def test_rain_off_leaves_the_jam(self):
    assert self.step(raining=True, jam=True, rain_enabled=False)
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)

  # a distance button over both

  def test_override_during_rain_then_a_new_jam_switches(self):
    self.step(raining=True)
    self.cycle()   # rain's wraps to the shortest: off for this rain
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)
    assert self.step(jam=True)   # a jam after the press
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)
    assert self.step(jam=False)
    assert not self.step(raining=False)
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)

  def test_override_during_rain_and_a_jam_lasts_for_both(self):
    self.step(raining=True, jam=True)
    self.cycle()
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)
    assert not self.step(raining=False)   # the jam goes on, still the driver's
    assert self.driven() == (Personality.aggressive, NONE, Personality.aggressive)
    assert not self.step(jam=False)
    assert self.step(raining=True)        # a new rain switches again
    assert self.driven()[1] == RAIN

  def test_override_during_rain_and_a_jam_then_the_jam_ends(self):
    self.step(raining=True, jam=True)
    self.press(Personality.standard)
    assert not self.step(jam=False)
    assert self.driven() == (Personality.standard, NONE, Personality.standard)
    assert self.step(jam=True)            # a new jam switches again, rain's stays off for this rain
    assert self.driven() == (Personality.relaxed, JAM, Personality.relaxed)

  def test_back_to_rain_after_the_override(self):
    self.step(raining=True, jam=True)
    self.press(Personality.standard)
    self.cycle()                          # standard -> relaxed, stored
    self.cycle()                          # relaxed -> rain's: on again, as RainFollow does
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.relaxed)
    assert self.step(raining=False)       # stepped around and back: the end of the rain goes to relaxed
    assert self.driven() == (Personality.relaxed, NONE, Personality.relaxed)

  def test_override_during_a_jam_then_rain(self):
    self.step(jam=True)
    self.press(Personality.standard)
    assert self.step(raining=True)        # a new rain switches
    assert self.driven() == (RAIN_POSITION, RAIN, Personality.standard)
    assert self.step(raining=False)       # back to the driver's choice, the jam is still the one pressed in
    assert self.driven() == (Personality.standard, NONE, Personality.standard)

  def test_first_press_only_shows(self):
    # the selfdrived first press only shows the distance: no select, so no override
    self.step(jam=True)
    assert not self.step()
    assert self.driven()[1] == JAM
