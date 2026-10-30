"""Grade of the road ahead from modelV2's path, for the cluster's uphill / downhill prompts.

modelV2.position is in the calibrated frame of the car's current attitude (x forward, y right, z down). Adding its pitch
change to the current pitch reads a turn on a slope as a grade change: after a hairpin on an 8 % climb the road ahead
reads 16 % steeper than the road under the car, though the grade never changes. Rotating the path into the world frame
with the current roll and pitch first removes that. On the owner's hilly drives (analysis-output/model-slope) the path's
grade 100 m ahead is then within 1.0 % for 90 % of the frames.

Pure numpy, no openpilot imports, so the offline replay runs the same code.
"""
import math

import numpy as np

CHORD = 20.                        # m, half length of the chord a grade is measured over
GRID = np.arange(20., 191., 10.)   # m ahead where the grade is checked
MIN_RUN = 3                        # consecutive grid points beyond the threshold, i.e. 30 m of road


def path_heights(x, y, z, roll: float, pitch: float) -> np.ndarray:
  """Height (m, up) above the camera in the world frame of points in the car's calibrated frame (x fwd, y right, z down)."""
  sp, cp, sr, cr = math.sin(pitch), math.cos(pitch), math.sin(roll), math.cos(roll)
  return sp * np.asarray(x) - cp * (sr * np.asarray(y) + cr * np.asarray(z))


def grades_ahead(x, y, z, roll: float, pitch: float) -> np.ndarray:
  """World grade (rise over run) on GRID along the predicted path, NaN where the path is too short."""
  x, y = np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)
  h = path_heights(x, y, z, roll, pitch)
  arc = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
  arc = np.maximum.accumulate(arc + np.arange(len(arc)) * 1e-6)   # strictly increasing for interp
  hi = np.interp(GRID + CHORD, arc, h, right=np.nan)
  lo = np.interp(GRID - CHORD, arc, h, right=np.nan)
  return (hi - lo) / (2 * CHORD)


def slope_ahead(grades: np.ndarray, threshold: float) -> tuple[int, float]:
  """(+1 uphill / -1 downhill, distance to its start) of the nearest run of MIN_RUN grid points beyond the threshold."""
  best = (0, 0.)
  for sign in (1, -1):
    run = 0
    for i, g in enumerate(grades):
      run = run + 1 if np.isfinite(g) and sign * g >= threshold else 0
      if run == MIN_RUN:
        start = float(GRID[i - MIN_RUN + 1])
        if best[0] == 0 or start < best[1]:
          best = (sign, start)
        break
  return best
