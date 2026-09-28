import unittest
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.yolo_lead import yolo_leadd as Y


def exp(tid, d, u, vg=400.0, f=1141.5):
  return {"tid": tid, "d": d, "u": u, "vg": vg, "w": f * 1.8 / (d + Y.RADAR_TO_CAMERA)}


class TestTypes(unittest.TestCase):
  def test_thresholds(self):
    self.assertEqual(Y.type_for("truck", 0.61), Y.TYPE_TRUCK)
    self.assertEqual(Y.type_for("bus", 0.61), Y.TYPE_TRUCK)
    self.assertIsNone(Y.type_for("truck", 0.59))
    self.assertEqual(Y.type_for("motorcycle", 0.46), Y.TYPE_TWO_WHEELER)
    self.assertEqual(Y.type_for("bicycle", 0.46), Y.TYPE_TWO_WHEELER)
    self.assertEqual(Y.type_for("person", 0.46), Y.TYPE_PERSON)
    self.assertIsNone(Y.type_for("car", 0.4))

  def test_hysteresis_back_to_car_needs_two_results(self):
    t = Y.TypeTracker()
    seq = []
    for typ in (Y.TYPE_TRUCK, Y.TYPE_CAR, Y.TYPE_TRUCK, Y.TYPE_CAR, Y.TYPE_CAR):
      t.record(7, 0.0, typ, 0.8)
      seq.append(t.shown(7)[0])
    self.assertEqual(seq, [2, 2, 2, 2, 3])

  def test_unknown_is_car_and_forgotten(self):
    t = Y.TypeTracker()
    self.assertEqual(t.shown(1), (Y.TYPE_CAR, 0.0))
    t.record(1, 0.0, Y.TYPE_TRUCK, 0.9)
    t.forget_old(Y.TRACK_FORGET + 1)
    self.assertEqual(t.shown(1)[0], Y.TYPE_CAR)


class TestTargets(unittest.TestCase):
  def test_side_targets_like_carstate(self):
    def P(tid, d, y, lane):
      return SimpleNamespace(trackId=tid, dRel=d, yRel=y, laneAssignment=lane)
    pts = [P(1, 30, 3.2, "left"), P(2, 18, 3.0, "left"), P(3, 12, 0.6, "left"),     # 3: cutting in
           P(4, 40, -3.4, "right"), P(5, 160, -3.0, "right"), P(6, 20, 0.1, "same")]
    self.assertEqual(Y.side_targets(pts), {"left": (2, 18.0, 3.0), "right": (4, 40.0, -3.4)})

  def test_far_and_near_targets_are_split(self):
    groups = Y.group_targets([exp(1, 120, 700), exp(2, 8, 300)])
    self.assertEqual(len(groups), 2)
    self.assertEqual(len(Y.group_targets([exp(1, 60, 650), exp(2, 50, 750)])), 1)

  def test_joint_matching_keeps_the_neighbour_off_the_target(self):
    truck, car = exp(1, 40, 800), exp(2, 40, 700)          # truck in the right lane, car in ours
    x0, y0, side = Y.roi_for([truck, car])
    def to_input(u, w):
      return (u - w / 2 - x0) * Y.INPUT / side, (u + w / 2 - x0) * Y.INPUT / side
    bottom = (400 - y0) * Y.INPUT / side
    tx = to_input(800, truck["w"] * 1.3)
    cx = to_input(700, car["w"])
    dets = [("car", 0.8, (cx[0], bottom - 20, cx[1], bottom)), ("truck", 0.8, (tx[0], bottom - 30, tx[1], bottom))]
    m = Y.match_targets(dets, [truck, car], x0, y0, side)
    self.assertEqual(m[1][0], "truck")
    self.assertEqual(m[2][0], "car")

  def test_drawn_unknown_first_then_pre_classify(self):
    t = Y.TypeTracker()
    a, b = [exp(1, 30, 600)], [exp(2, 90, 900)]
    self.assertIs(Y.pick_group([a, b], {1}, t, 0.0), a)     # drawn and unknown
    t.record(1, 0.0, Y.TYPE_CAR, 0.9)
    self.assertIs(Y.pick_group([a, b], {1}, t, 0.5), b)     # drawn known -> classify the one behind


class TestImage(unittest.TestCase):
  def test_crop_shape_and_grey_outside(self):
    y = np.full((760, 1344), 200, np.uint8)
    uv = np.full((380, 672, 2), 128, np.uint8)
    x = Y.nv12_crop_rgb(y, uv, -320, 100, 640)             # left half outside the frame
    self.assertEqual(x.shape, (1, 3, 320, 320))
    self.assertAlmostEqual(float(x[0, 0, 160, 10]), 114 / 255, places=3)
    self.assertAlmostEqual(float(x[0, 0, 160, 300]), 200 / 255, places=3)

  def test_decode_nms(self):
    raw = np.zeros((1, 84, 3), np.float32)
    raw[0, :4, 0] = raw[0, :4, 1] = [100, 100, 40, 40]      # two overlapping trucks
    raw[0, 4 + 7, 0], raw[0, 4 + 7, 1] = 0.9, 0.8
    raw[0, :4, 2] = [250, 250, 20, 20]
    raw[0, 4 + 0, 2] = 0.7                                  # a person elsewhere
    names = sorted(d[0] for d in Y.decode(raw))
    self.assertEqual(names, ["person", "truck"])


if __name__ == "__main__":
  unittest.main()
