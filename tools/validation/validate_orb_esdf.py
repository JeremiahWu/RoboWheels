#!/usr/bin/env python3
"""Run on X5 (OpenCV/SciPy): synthetic metric VO and ESDF regressions."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/robowheels"))
import cv2
import numpy as np
from robowheels.localization.orb_odometry import OrbOdometry
from robowheels.planning.grid import signed_distance, astar

rng = np.random.default_rng(42)
image = rng.integers(0, 256, (240, 320), dtype=np.uint8)
image = cv2.GaussianBlur(image, (3, 3), 0)
depth = np.full(image.shape, 2.0, dtype=np.float32)
K = np.array([[200.0, 0, 160.0], [0, 200.0, 120.0], [0, 0, 1.0]])
vo = OrbOdometry()
assert vo.update(image, depth, K, 1.0)["pose"] is None
static = vo.update(image, depth, K, 1.2)
assert (
    static["state"] == "TRACKING_VO_ONLY"
    and np.linalg.norm(static["pose"] - np.eye(4)) < 1e-4
)
moved = cv2.warpAffine(image, np.float32([[1, 0, 5], [0, 1, 0]]), (320, 240))
r = vo.update(moved, depth, K, 1.4)
assert r["pose"] is not None, r
assert abs(r["pose"][0, 3] + 0.05) < 0.015, r["pose"]
assert vo.update(moved, depth, K, 1.3)["state"] == "NON_MONOTONIC_INPUT"
r = vo.update(np.zeros_like(image), depth, K, 1.6)
assert r["pose"] is None and r["segment"] == 1
# Positive in observed free space, negative inside blocked/unknown cells.
g = np.zeros((21, 21), dtype=np.int8)
g[:, 0] = 100
g[10, 10] = 100
g[0, 0] = -1
d = signed_distance(g, 0.1)
assert abs(d[10, 12] - 0.2) < 1e-6 and d[10, 10] < 0 and d[0, 0] < 0
assert np.isfinite(signed_distance(np.zeros((4, 4), dtype=np.int8), 0.1)).all()
assert not astar(
    np.full((4, 4), -1, dtype=np.int8),
    (1, 1),
    (2, 2),
    clearance=signed_distance(np.full((4, 4), -1, dtype=np.int8), 0.1),
    clearance_weight=0.15,
)
print(
    "Passed ORB static/metric motion, lost tracking, time ordering, ESDF sign/units/boundaries and unknown-space rejection"
)
