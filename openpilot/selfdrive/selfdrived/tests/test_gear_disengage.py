from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import ET, Events

AudibleAlert = log.SelfdriveState.AudibleAlert
EventName = log.OnroadEvent.EventName


def disengage_sounds(events, v_ego_kph):
  CS = car.CarState(vEgo=v_ego_kph * CV.KPH_TO_MS)
  alerts = Events()
  for e in events:
    alerts.add(e)
  return [a.audible_alert for a in alerts.create_alerts([ET.USER_DISABLE], [car.CarParams(), CS, None, True, 0, None])]


class TestGearDisengage(OpenpilotTestCase):
  def test_parking_is_silent(self):
    assert disengage_sounds([EventName.wrongGear], 0.) == [AudibleAlert.none]
    assert disengage_sounds([EventName.wrongGear], 9.) == [AudibleAlert.none]

  def test_reversing_is_silent(self):
    assert disengage_sounds([EventName.wrongGear, EventName.reverseGear], 2.) == [AudibleAlert.none, AudibleAlert.none]

  def test_leaving_drive_at_speed_chimes(self):
    assert disengage_sounds([EventName.wrongGear], 11.) == [AudibleAlert.disengage]
    assert disengage_sounds([EventName.wrongGear], 60.) == [AudibleAlert.disengage]
