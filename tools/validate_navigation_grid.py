#!/usr/bin/env python3
"""Offline checks: obstacles, unknown space, corner cuts and bounded map rays."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/robowheels"))
import numpy as np
from robowheels.planning.grid import astar, inflate, ray_grid


def main():
    g = np.zeros((12, 12), dtype=np.int8)
    g[5, :10] = 100
    p = astar(g, (1, 1), (1, 10))
    assert p and all(g[y, x] == 0 for x, y in p)
    assert any(x >= 10 for x, y in p)
    g[5, 10:] = 100
    assert not astar(g, (1, 1), (1, 10))
    assert not astar(np.full((3, 3), -1, dtype=np.int8), (0, 0), (2, 2))
    assert not astar(np.array([[0, 100], [100, 0]], dtype=np.int8), (0, 0), (1, 1))
    g = np.zeros((7, 7), dtype=np.int8)
    g[3, 3] = 100
    inf = inflate(g, 2)
    assert inf[3, 5] == 100 and inf[5, 5] == 0
    g = ray_grid([(0, 0, 1), (0, 0, float("nan")), (0, 0, -1)], size=20, resolution=0.2)
    assert g[10, 15] == 100 and g[10, 12] == 0 and g[0, 0] == -1
    assert ray_grid([(0, 0, 100)], size=20, resolution=0.2).shape == (20, 20)
    print(
        "Passed obstacle detours, blocked/unknown goals, diagonal corners, inflation, ray bounds and invalid depth"
    )


if __name__ == "__main__":
    main()
