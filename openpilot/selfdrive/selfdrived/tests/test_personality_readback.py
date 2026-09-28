import threading
from types import SimpleNamespace

import openpilot.selfdrive.selfdrived.selfdrived as selfdrived


class FakeParams:
  def __init__(self, personality):
    self.personality = personality

  def get_bool(self, key):
    return False

  def get(self, key, return_default=False):
    assert key == "LongitudinalPersonality"
    return self.personality


def read_params_once(sd):
  evt = threading.Event()
  selfdrived.time.sleep.side_effect = lambda _: evt.set()
  sd.params_thread(evt)


class TestPersonalityReadback:
  def make(self, mocker, on_disk):
    mocker.patch.object(selfdrived.time, "sleep")
    sd = selfdrived.SelfdriveD.__new__(selfdrived.SelfdriveD)
    sd.params = FakeParams(on_disk)
    sd.CP = SimpleNamespace(openpilotLongitudinalControl=True)
    sd.mads = SimpleNamespace(read_params=lambda: None)
    sd.personality = on_disk
    sd.personality_pending = None
    return sd

  def test_stale_read_does_not_undo_change(self, mocker):
    sd = self.make(mocker, on_disk=2)
    now = mocker.patch.object(selfdrived.time, "monotonic", return_value=100.)

    # changed to aggressive here, the write has not landed yet
    sd.personality_pending = (0, 100.)
    sd.personality = 0
    now.return_value = 103.
    read_params_once(sd)
    assert sd.personality == 0

    # write landed: back to following the param
    sd.params.personality = 0
    read_params_once(sd)
    assert sd.personality_pending is None
    sd.params.personality = 1  # changed from the settings screen
    read_params_once(sd)
    assert sd.personality == 1

  def test_trust_param_after_timeout(self, mocker):
    sd = self.make(mocker, on_disk=2)
    now = mocker.patch.object(selfdrived.time, "monotonic", return_value=100.)
    sd.personality_pending = (0, 100.)
    sd.personality = 0
    now.return_value = 100. + selfdrived.PERSONALITY_WRITE_TIMEOUT + 1
    read_params_once(sd)
    assert sd.personality == 2 and sd.personality_pending is None
