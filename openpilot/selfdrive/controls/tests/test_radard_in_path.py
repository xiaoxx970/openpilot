from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import KalmanParams, LaneAssignment, Track, get_lead, RADAR_TO_CAMERA, IN_PATH_MIN_FRAMES

V_EGO = 22.
CP = SimpleNamespace(brand="volkswagen", flags=0)
CP_SP = SimpleNamespace(flags=0)
PATH = SimpleNamespace(x=[0., 50., 100., 150.], y=[0., 0., 0., 0.])


def vision_lead(d_rel, v_lead=V_EGO, prob=0.9):
  return SimpleNamespace(x=[d_rel + RADAR_TO_CAMERA], y=[0.], v=[v_lead], a=[0.], xStd=[1.], yStd=[1.], vStd=[1.], prob=prob)


def make_track(track_id, d_rel, y_rel, v_lead, lane, frames):
  track = Track(track_id, v_lead, KalmanParams(DT_MDL))
  for _ in range(frames):
    track.update(d_rel, y_rel, v_lead - V_EGO, v_lead, lane)
    track.update_in_path(PATH)
  return track


def lead_one(tracks, lead_msg):
  return get_lead(V_EGO, True, {t.identifier: t for t in tracks}, lead_msg, V_EGO, lead_msg.prob, CP, CP_SP, PATH,
                  low_speed_override=True, in_path_fallback=True)


class TestInPathFallback:
  def test_departed_vision_lead(self):
    # the model still follows a car that left the lane at ~95 m, the radar has a slower car in our lane at 62 m
    lead = lead_one([make_track(1, 62., -0.3, 18.6, LaneAssignment.same, IN_PATH_MIN_FRAMES)], vision_lead(95.))
    assert lead['radar'] and lead['radarTrackId'] == 1
    assert lead['dRel'] == 62.

  def test_needs_one_second_in_path(self):
    lead = lead_one([make_track(1, 62., -0.3, 18.6, LaneAssignment.same, IN_PATH_MIN_FRAMES - 1)], vision_lead(95.))
    assert not lead['radar']

  def test_stationary_track_ignored(self):
    lead = lead_one([make_track(1, 62., 0., 0., LaneAssignment.same, 100)], vision_lead(95.))
    assert not lead['radar']

  def test_neighbour_lane_track_ignored(self):
    for lane in (LaneAssignment.left, LaneAssignment.right, LaneAssignment.unknown):
      lead = lead_one([make_track(1, 62., -0.3, 18.6, lane, 100)], vision_lead(95.))
      assert not lead['radar'], lane

  def test_off_path_track_ignored(self):
    lead = lead_one([make_track(1, 62., -1.5, 18.6, LaneAssignment.same, 100)], vision_lead(95.))
    assert not lead['radar']

  def test_closer_than_matched_track(self):
    far = make_track(1, 70., 0., V_EGO, LaneAssignment.same, 100)
    near = make_track(2, 45., 0.2, 18., LaneAssignment.same, 100)
    lead = lead_one([far, near], vision_lead(70.))
    assert lead['radarTrackId'] == 2

  def test_not_clearly_closer(self):
    # within max(5 m, 20 %) of the current lead: the normal match is kept
    far = make_track(1, 60., 0., V_EGO, LaneAssignment.same, 100)
    near = make_track(2, 50., 0.2, V_EGO, LaneAssignment.same, 100)
    lead = lead_one([far, near], vision_lead(60.))
    assert lead['radarTrackId'] == 1

  def test_no_vision_lead(self):
    lead = lead_one([make_track(1, 62., -0.3, 18.6, LaneAssignment.same, 100)], vision_lead(95., prob=0.2))
    assert not lead['present']

  def test_lead_two_unchanged(self):
    track = make_track(1, 62., -0.3, 18.6, LaneAssignment.same, 100)
    lead_msg = vision_lead(95.)
    lead = get_lead(V_EGO, True, {1: track}, lead_msg, V_EGO, lead_msg.prob, CP, CP_SP, PATH, low_speed_override=False)
    assert not lead['radar']
