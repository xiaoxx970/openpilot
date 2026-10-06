import threading
import time

import openpilot.system.hardware.hardwared as hardwared


class FakeParams:
  def __init__(self, release: threading.Event):
    self.release = release
    self.written = []

  def put(self, key, val, block=False):
    self.release.wait()
    self.written.append((key, val, block))

  def put_bool(self, key, val, block=False):
    self.put(key, val, block)


def wait_for(cond, timeout=2.0):
  end = time.monotonic() + timeout
  while not cond() and time.monotonic() < end:
    time.sleep(0.01)
  return cond()


class TestParamWriter:
  def test_stalled_disk_does_not_block_caller(self, mocker):
    release = threading.Event()
    fake = FakeParams(release)
    mocker.patch.object(hardwared, "Params", return_value=fake)
    writer = hardwared.ParamWriter()

    start = time.monotonic()
    writer.put("IsEngaged", True)
    writer.put("UptimeOnroad", 12.5)
    writer.put("IsEngaged", False)
    assert time.monotonic() - start < 0.1

    release.set()
    assert wait_for(lambda: len(fake.written) == 3)
    assert fake.written == [("IsEngaged", True, True), ("UptimeOnroad", 12.5, True), ("IsEngaged", False, True)]

  def test_slow_write_is_logged(self, mocker):
    release = threading.Event()
    fake = FakeParams(release)
    mocker.patch.object(hardwared, "Params", return_value=fake)
    mocker.patch.object(hardwared.ParamWriter, "SLOW_WRITE", 0.05)
    event = mocker.patch.object(hardwared.cloudlog, "event")
    writer = hardwared.ParamWriter()

    writer.put("UptimeOnroad", 1.0)
    time.sleep(0.1)
    release.set()
    assert wait_for(lambda: event.called)
    assert event.call_args.args[0] == "slow_param_write"
    assert event.call_args.kwargs["key"] == "UptimeOnroad"


class TestStallWatchdog:
  def test_logs_stack_of_stuck_loop(self, mocker):
    mocker.patch.object(hardwared.StallWatchdog, "STALL", 0.2)
    event = mocker.patch.object(hardwared.cloudlog, "event")
    stuck = threading.Event()

    def loop():
      watchdog = hardwared.StallWatchdog()
      stuck.wait(0.8)  # stands in for a call blocked on the disk
      watchdog.kick()

    t = threading.Thread(target=loop)
    t.start()
    t.join()
    names = [c.args[0] for c in event.call_args_list]
    assert names == ["hardwared_stall", "hardwared_stall_end"]
    assert "stuck.wait" in event.call_args_list[0].kwargs["stack"]
