# Big cluster (cores 4-7) frequency cap while driving. Upstream pins it at 1.69 GHz onroad (commaai #38132); on the
# Golf 8 core 4 (card, selfdrived, controlsd) sits at ~98 % there and starves kernel work on that core (2026-11-04:
# hardwared's thermal read stalled 4 s -> commIssue). While the device is cool, allow one step more.
# The device's hottest log through 2026-11-04 is 78.6 C at 1.69 GHz; openpilot calls a comma 4 overheated above 100 C.
BIG_CORE_BASE_FREQ = 1689600  # kHz, the upstream onroad cap
BIG_CORE_COOL_FREQ = 1843200  # kHz, +9 %
BIG_CORE_COOL_TEMP = 85.  # C, max component temperature to step up below
BIG_CORE_HOT_TEMP = 88.  # C, back to the base cap from here


class BigCoreFreqCap:
  def __init__(self):
    self.cap = BIG_CORE_BASE_FREQ

  def update(self, temp_c: float, onroad: bool) -> int:
    if not onroad:
      self.cap = BIG_CORE_BASE_FREQ
    elif self.cap == BIG_CORE_BASE_FREQ and temp_c < BIG_CORE_COOL_TEMP:
      self.cap = BIG_CORE_COOL_FREQ
    elif self.cap == BIG_CORE_COOL_FREQ and temp_c >= BIG_CORE_HOT_TEMP:
      self.cap = BIG_CORE_BASE_FREQ
    return self.cap
