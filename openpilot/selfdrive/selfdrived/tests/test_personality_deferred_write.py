import threading
import time
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


class Params:
  def __init__(self, value):
    self.value = value
    self.writes = []

  def get_bool(self, _key):
    return False

  def get(self, key, return_default=False):
    assert key == 'LongitudinalPersonality'
    return self.value

  def put(self, key, value, block=False):
    assert key == 'LongitudinalPersonality' and block
    self.value = value
    self.writes.append(value)


def make_selfdrive(params):
  selfdrive = SelfdriveD.__new__(SelfdriveD)
  vars(selfdrive).update(  # stand-ins for Params and MADS, only what the two threads touch
    params=params,
    CP=SimpleNamespace(openpilotLongitudinalControl=True),
    mads=SimpleNamespace(read_params=lambda: None),
    personality=params.value,
    personality_lock=threading.Lock(),
    personality_write_event=threading.Event(),
    personality_write_pending=None,
    personality_generation=0,
  )
  return selfdrive


class TestPersonalityDeferredWrite(unittest.TestCase):
  def test_changes_while_the_display_is_open_are_written_once(self):
    params = Params(0)
    selfdrive = make_selfdrive(params)
    stop = threading.Event()
    reader = threading.Thread(target=selfdrive.params_thread, args=(stop,))
    writer = threading.Thread(target=selfdrive.personality_write_thread, args=(stop,))
    reader.start()
    writer.start()
    try:
      # presses while the distance display is open only change the value in memory
      selfdrive._set_personality(1)
      selfdrive._set_personality(2)
      time.sleep(0.25)  # several params_thread reads of the old value on disk
      self.assertEqual(selfdrive.personality, 2)
      self.assertEqual(params.writes, [])

      # the display closed: step() wakes the writer
      selfdrive.personality_write_event.set()
      deadline = time.monotonic() + 2
      while selfdrive.personality_write_pending is not None and time.monotonic() < deadline:
        time.sleep(0.01)
      self.assertIsNone(selfdrive.personality_write_pending)
      self.assertEqual(params.writes, [2])
      self.assertEqual(selfdrive.personality, 2)
    finally:
      stop.set()
      selfdrive.personality_write_event.set()
      reader.join(timeout=2)
      writer.join(timeout=2)
      self.assertFalse(reader.is_alive() or writer.is_alive())


if __name__ == "__main__":
  unittest.main()
