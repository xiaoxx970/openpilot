import os

from openpilot.selfdrive.navigation import mapsd


def test_pins_to_modeld_core(mocker):
  setaff = mocker.patch.object(os, 'sched_setaffinity', create=True)
  mapsd.pin_to_core()
  setaff.assert_called_once_with(0, {mapsd.MAPSD_CORE})
  assert mapsd.MAPSD_CORE == 7


def test_missing_core_is_ignored(mocker):
  mocker.patch.object(os, 'sched_setaffinity', create=True, side_effect=OSError(22, 'Invalid argument'))
  mapsd.pin_to_core()
