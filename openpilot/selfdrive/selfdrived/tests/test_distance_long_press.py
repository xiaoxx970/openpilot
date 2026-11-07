"""The fork's stock VW distance button (DistanceDisplay, FollowDistance) together with upstream's Blendful long press."""
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from opendbc.car.structs import car
from openpilot.cereal import custom, log
from openpilot.selfdrive.car.distance_display import DistanceDisplay
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.selfdrive.selfdrived.follow_distance import FollowDistance
from openpilot.selfdrive.selfdrived.rain_follow import RainDetector
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
Personality = log.LongitudinalPersonality
BLENDFUL_ON = custom.OnroadEventSP.EventName.blendfulEnabled


class SubMaster:
  """Only what _update_distance reads: no rain, no road events."""
  def __init__(self):
    self.frame = 0
    self.seen = {'roadEventsIC': False}
    self.recv_frame = {'roadEventsIC': 0}

  def __getitem__(self, _service):
    return SimpleNamespace(rainAmount=0., rainWiperRequest=0, wiperSwitch=0)


class TestDistanceLongPress(unittest.TestCase):
  def setUp(self):
    sd = self.sd = SelfdriveD.__new__(SelfdriveD)
    self.values = {"Blendful": False, "ModelRunnerTypeCache": custom.ModelManagerSP.Runner.tinygrad}
    params = Mock()
    params.get.side_effect = self.values.get
    params.get_bool.side_effect = self.values.get
    params.put_bool.side_effect = self.values.__setitem__
    sd.sm = SubMaster()
    self.events, self.events_sp = [], []
    vars(sd).update(
      CP=SimpleNamespace(openpilotLongitudinalControl=True), params=params,
      events=SimpleNamespace(add=self.events.append), events_sp=SimpleNamespace(add=self.events_sp.append),
      distance_button_pressed_frame=None, distance_button_long_press=False, distance_display=DistanceDisplay(),
      rain_detector=RainDetector(), follow=FollowDistance(), rain_follow_enabled=False,
      personality=Personality.standard, personality_lock=threading.Lock(), personality_write_event=threading.Event(),
      personality_write_pending=None, personality_generation=0,
    )

  def tick(self, pressed=None, valid=True, frames=1):
    for i in range(frames):
      buttons = [] if pressed is None or i else [ButtonEvent(type=ButtonType.gapAdjustCruise, pressed=pressed)]
      CS = SimpleNamespace(buttonEvents=buttons, canValid=valid, cruiseState=SimpleNamespace(available=True))
      self.sd._update_distance(CS)
      self.sd.sm.frame += 1

  def short_press(self):
    self.tick(True)
    self.tick(frames=20)
    self.tick(False)

  def test_short_presses_unchanged(self):
    # the first press only shows the distance, the second (while shown) steps it, wrapping to the shortest
    self.short_press()
    self.assertEqual(self.sd.personality, Personality.standard)
    self.assertIn(EventName.personalityChanged, self.events)
    self.short_press()
    self.assertEqual(self.sd.personality, Personality.relaxed)
    self.short_press()
    self.assertEqual(self.sd.personality, Personality.aggressive)
    self.assertFalse(self.values["Blendful"])
    self.assertEqual(self.events_sp, [])

  def test_long_press_only_toggles_blendful(self):
    self.tick(True)
    self.tick(frames=150)
    self.assertTrue(self.values["Blendful"])
    self.assertEqual(self.events_sp, [BLENDFUL_ON])
    self.events.clear()
    self.tick(False)
    self.assertEqual(self.sd.personality, Personality.standard)
    self.assertNotIn(EventName.personalityChanged, self.events)

  def test_long_press_while_shown_does_not_step(self):
    self.short_press()  # distance shown
    self.events.clear()
    self.tick(True)
    self.tick(frames=110)
    self.tick(False)
    self.assertTrue(self.values["Blendful"])
    self.assertEqual(self.sd.personality, Personality.standard)
    self.assertNotIn(EventName.personalityChanged, self.events)
    self.short_press()  # a short press after it steps as before
    self.assertEqual(self.sd.personality, Personality.relaxed)

  def test_can_invalid_resets_the_hold(self):
    self.tick(True)
    self.tick(frames=50)
    self.tick(valid=False)
    self.tick(frames=100)
    self.tick(False)
    self.assertFalse(self.values["Blendful"])
    self.assertEqual(self.events_sp, [])
    # the release counts as an ordinary press again: it shows the distance, it does not step it
    self.assertEqual(self.sd.personality, Personality.standard)
    self.assertIn(EventName.personalityChanged, self.events)


if __name__ == "__main__":
  unittest.main()
