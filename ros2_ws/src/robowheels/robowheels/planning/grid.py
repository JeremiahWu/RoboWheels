"""ROS-independent conservative grid planning for a local preview."""

import heapq
import math
import numpy as np


def inflate(grid, radius_cells):
    result = grid.copy()
    for y, x in np.argwhere(grid >= 50):
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < grid.shape[0] and 0 <= xx < grid.shape[1]:
                        result[yy, xx] = 100
    return result


def astar(grid, start, goal, clearance=None, clearance_weight=0.0):
    """Return (x,y) cells; unknown cells are blocked; no diagonal corner cuts."""

    def free(p):
        x, y = p
        return 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] and grid[y, x] == 0

    if not free(start) or not free(goal):
        return []
    queue = [(0.0, start)]
    cost = {start: 0.0}
    parent = {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        x, y = current
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ):
            nxt = (x + dx, y + dy)
            if not free(nxt) or (
                dx and dy and (not free((x + dx, y)) or not free((x, y + dy)))
            ):
                continue
            penalty = (
                clearance_weight / max(float(clearance[nxt[1], nxt[0]]), 0.05)
                if clearance is not None
                else 0.0
            )
            trial = cost[current] + math.hypot(dx, dy) * (1.0 + penalty)
            if trial < cost.get(nxt, float("inf")):
                cost[nxt] = trial
                parent[nxt] = current
                heapq.heappush(
                    queue, (trial + math.hypot(nxt[0] - goal[0], nxt[1] - goal[1]), nxt)
                )
    return []


def signed_distance(grid, resolution):
    """Signed Euclidean distance in meters; unknown is blocked, not free."""
    from scipy.ndimage import distance_transform_edt

    free = grid == 0
    # Pad the map with occupied cells so edge clearance is finite.
    padded = np.pad(free, 1, constant_values=False)
    outside = distance_transform_edt(padded)[1:-1, 1:-1]
    inside = distance_transform_edt(~padded)[1:-1, 1:-1]
    return ((outside - inside) * resolution).astype(np.float32)


def ray_grid(points, size=120, resolution=0.05, min_height=-0.2, max_height=0.5):
    """Vectorized bounded ray clearing; endpoint occupancy always wins."""
    grid = np.full((size, size), -1, dtype=np.int8)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0)
    valid &= (-pts[:, 1] >= min_height) & (-pts[:, 1] <= max_height)
    pts = pts[valid]
    if not len(pts):
        return grid
    center = size // 2
    ends = (
        np.floor(np.column_stack((pts[:, 2], -pts[:, 0])) / resolution).astype(np.int64)
        + center
    )
    lengths = np.max(np.abs(ends - center), axis=1)
    # At most 2*size samples along each in-map portion of a ray.
    for step in range(size * 2):
        active = lengths >= step
        if not active.any():
            break
        selected = ends[active]
        count = lengths[active]
        cells = np.rint(
            center + (selected - center) * step / np.maximum(count[:, None], 1)
        ).astype(int)
        inside = ((cells >= 0) & (cells < size)).all(axis=1)
        cells = cells[inside]
        grid[cells[:, 1], cells[:, 0]] = 0
    inside = ((ends >= 0) & (ends < size)).all(axis=1)
    ends = ends[inside]
    grid[ends[:, 1], ends[:, 0]] = 100
    return grid
