from openpilot.selfdrive.controls.lib.lane_line_visibility import LaneLineVisibility, LANE_LINE_HOLD_TIME

DT = 0.01


def _run(llv, prob, seconds):
  for _ in range(round(seconds / DT)):
    llv.update(prob)
  return llv.visible


class TestLaneLineVisibility:
  def test_starts_hidden(self):
    assert not LaneLineVisibility(DT).visible

  def test_shows_after_hold_time(self):
    llv = LaneLineVisibility(DT)
    assert not _run(llv, 0.9, LANE_LINE_HOLD_TIME - 0.1)
    assert _run(llv, 0.9, 0.2)

  def test_hysteresis_keeps_a_weak_line(self):
    llv = LaneLineVisibility(DT)
    _run(llv, 0.9, 3.)
    # between the off and on thresholds: stays visible
    assert _run(llv, 0.1, 10.)

  def test_hides_after_hold_time(self):
    llv = LaneLineVisibility(DT)
    _run(llv, 0.9, 3.)
    assert _run(llv, 0.0, LANE_LINE_HOLD_TIME - 0.1)
    assert not _run(llv, 0.0, 0.2)

  def test_short_dropout_is_ignored(self):
    llv = LaneLineVisibility(DT)
    _run(llv, 0.9, 3.)
    _run(llv, 0.0, 1.)
    assert _run(llv, 0.9, 5.)

  def test_parking_lot_stays_hidden(self):
    assert not _run(LaneLineVisibility(DT), 0.01, 60.)
