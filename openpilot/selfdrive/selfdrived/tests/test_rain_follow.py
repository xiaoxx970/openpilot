from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.rain_follow import (DRY, HEAVY_RAIN, HEAVY_RAIN_HOLD_TIME, RAIN, RAIN_OFF_TIME, RAIN_ON_TIME,
                                                        RAIN_POSITION, RainDetector, RainFollow)

Personality = log.LongitudinalPersonality
DT = 0.01


class TestRainDetector:
  def setup_method(self):
    self.rd = RainDetector(DT)

  def run(self, seconds, rain_amount=0., wiper_request=0, wiper_switch=1):
    for _ in range(round(seconds / DT)):
      level = self.rd.update(rain_amount, wiper_request, wiper_switch)
    return level

  def test_rain_needs_a_few_seconds(self):
    assert self.run(RAIN_ON_TIME - 0.5, rain_amount=100.) == DRY
    assert self.run(1., rain_amount=100.) == RAIN

  def test_lone_low_reading_is_not_rain(self):
    # a dry Golf 8 sends a single 10 % frame now and then
    for _ in range(10):
      self.run(0.2, rain_amount=10.)
      self.run(3.)
    assert self.rd.level == DRY

  def test_road_stays_wet_after_the_rain(self):
    self.run(RAIN_ON_TIME + 1, rain_amount=100.)
    assert self.run(RAIN_OFF_TIME - 1) == RAIN
    assert self.run(2.) == DRY

  def test_short_dry_spell_does_not_end_it(self):
    self.run(RAIN_ON_TIME + 1, rain_amount=100.)
    self.run(RAIN_OFF_TIME - 1)
    self.run(1., rain_amount=100.)
    assert self.run(RAIN_OFF_TIME - 1) == RAIN

  def test_wiper_stalk_stage_is_rain(self):
    assert self.run(RAIN_ON_TIME + 1, wiper_switch=2) == RAIN
    assert self.run(1., wiper_switch=3) == HEAVY_RAIN

  def test_wiper_request_alone_is_not_rain(self):
    # the sensor asks for single wipes in drizzle that the amount never shows
    assert self.run(RAIN_ON_TIME + 1, wiper_request=1) == DRY

  def test_heavy_rain_from_fast_wiper_request(self):
    self.run(RAIN_ON_TIME + 1, rain_amount=100., wiper_request=1)
    assert self.rd.level == RAIN
    assert self.run(0.1, rain_amount=100., wiper_request=2) == HEAVY_RAIN
    assert self.run(HEAVY_RAIN_HOLD_TIME - 1, rain_amount=100., wiper_request=1) == HEAVY_RAIN
    assert self.run(2., rain_amount=100., wiper_request=1) == RAIN

  def test_heavy_from_the_start(self):
    assert self.run(RAIN_ON_TIME + 0.1, rain_amount=100., wiper_request=2) == HEAVY_RAIN


class TestRainFollow:
  def setup_method(self):
    self.rf = RainFollow()
    self.personality = Personality.standard

  def update(self, raining, enabled=True):
    self.personality, changed = self.rf.update(raining, enabled, self.personality)
    return changed

  def press(self, position):
    self.personality = self.rf.select(position, self.personality)

  def test_rain_switches_and_returns(self):
    assert not self.update(False)
    assert self.update(True)
    assert self.rf.active and self.rf.position(self.personality) == RAIN_POSITION
    assert not self.update(True)
    assert self.update(False)
    assert not self.rf.active and self.personality == Personality.standard

  def test_disabled_never_switches(self):
    assert not self.update(True, enabled=False)
    assert not self.rf.active
    assert self.rf.top(False) == Personality.relaxed

  def test_disabling_during_rain_switches_back(self):
    self.update(True)
    assert self.update(True, enabled=False)
    assert not self.rf.active and self.personality == Personality.standard

  def test_rain_position_only_reachable_while_raining(self):
    assert self.rf.top(True) == Personality.relaxed
    self.update(True)
    assert self.rf.top(True) == RAIN_POSITION

  def test_stepping_away_keeps_it_off_for_this_rain(self):
    self.update(True)
    self.press(Personality.aggressive)
    assert not self.rf.active and self.personality == Personality.aggressive
    assert not self.update(True)
    assert not self.rf.active
    assert not self.update(False)
    assert self.personality == Personality.aggressive
    # the next rain switches again
    assert self.update(True)
    assert self.rf.active

  def test_back_to_rain_after_stepping_ends_on_relaxed(self):
    self.update(True)
    self.press(Personality.aggressive)
    self.press(RAIN_POSITION)
    assert self.rf.active and self.personality == Personality.aggressive
    assert self.update(False)
    assert self.personality == Personality.relaxed

  def test_rain_is_never_stored(self):
    self.update(True)
    self.press(RAIN_POSITION)
    assert self.personality in (Personality.aggressive, Personality.standard, Personality.relaxed)
