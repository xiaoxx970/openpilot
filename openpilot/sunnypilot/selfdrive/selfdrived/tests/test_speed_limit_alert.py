import unittest
from types import SimpleNamespace
from typing import Any, cast

import openpilot.cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived import events


def _alert_text(set_speed_kph, limit_kph, metric=True):
  lp = messaging.new_message('longitudinalPlanSP')
  lp.longitudinalPlanSP.speedLimit.resolver.speedLimitFinalLast = limit_kph * CV.KPH_TO_MS
  sm = {'longitudinalPlanSP': lp.longitudinalPlanSP, 'controlsState': messaging.new_message('controlsState').controlsState}
  CP = SimpleNamespace(openpilotLongitudinalControl=True, pcmCruise=False)
  CS = SimpleNamespace(vCruiseCluster=set_speed_kph)
  return events.speed_limit_pre_active_alert(cast(Any, CP), cast(Any, CS), cast(Any, sm), metric, 0, None).alert_text_1


class TestSpeedLimitPreActiveAlert(unittest.TestCase):
  def setUp(self):
    self.is_mici = events.IS_MICI
    events.IS_MICI = True

  def tearDown(self):
    events.IS_MICI = self.is_mici

  def test_the_comma_screen_names_the_limit(self):
    self.assertEqual(_alert_text(40, 60), "Press + to confirm 60 km/h")
    self.assertEqual(_alert_text(80, 60), "Press - to confirm 60 km/h")


if __name__ == "__main__":
  unittest.main()
