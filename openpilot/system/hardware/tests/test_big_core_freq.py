from openpilot.system.hardware.big_core_freq import BigCoreFreqCap, BIG_CORE_BASE_FREQ, BIG_CORE_COOL_FREQ, \
                                                    BIG_CORE_COOL_TEMP, BIG_CORE_HOT_TEMP


def test_offroad_keeps_the_upstream_cap():
  assert BigCoreFreqCap().update(40., onroad=False) == BIG_CORE_BASE_FREQ


def test_cool_onroad_steps_up():
  assert BigCoreFreqCap().update(BIG_CORE_COOL_TEMP - 0.1, onroad=True) == BIG_CORE_COOL_FREQ


def test_hot_falls_back_and_waits_for_the_lower_threshold():
  cap = BigCoreFreqCap()
  cap.update(70., onroad=True)
  assert cap.update(BIG_CORE_HOT_TEMP - 0.1, onroad=True) == BIG_CORE_COOL_FREQ
  assert cap.update(BIG_CORE_HOT_TEMP, onroad=True) == BIG_CORE_BASE_FREQ
  # between the thresholds it stays where it is
  assert cap.update((BIG_CORE_COOL_TEMP + BIG_CORE_HOT_TEMP) / 2, onroad=True) == BIG_CORE_BASE_FREQ
  assert cap.update(BIG_CORE_COOL_TEMP - 0.1, onroad=True) == BIG_CORE_COOL_FREQ


def test_warm_start_stays_at_the_upstream_cap():
  assert BigCoreFreqCap().update(BIG_CORE_COOL_TEMP + 1., onroad=True) == BIG_CORE_BASE_FREQ


def test_leaving_onroad_drops_back():
  cap = BigCoreFreqCap()
  cap.update(70., onroad=True)
  assert cap.update(70., onroad=False) == BIG_CORE_BASE_FREQ
