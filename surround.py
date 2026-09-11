"""Surround (angular gap) test used for the directional detection guarantee.

A directional source at q is heard only from points inside its +-90 deg beam.
Measuring channel c at a set of points S therefore guarantees detection of a
source at q if, among the points of S within the smallest possible effective
radius (1000 m), some lie in every half plane through q - equivalently, if the
largest angular gap seen from q is smaller than 180 deg.

`surround_ok` evaluates that condition on a fine grid of query points. The
threshold is kept below 180 deg so that grid sampling cannot make the test
optimistic.
"""
from __future__ import annotations

import math
from functools import lru_cache

import numpy as np

ARENA_RADIUS = 1800.0
R_MIN = 1000.0
GAP_LIMIT_DEG = 168.0


@lru_cache(maxsize=8)
def check_grid(spacing: float = 45.0) -> np.ndarray:
    """Hexagonal-ish grid of query points covering the arena disc."""
    r_max = ARENA_RADIUS
    step_y = spacing * math.sqrt(3) / 2
    rows = int(r_max / step_y) + 2
    pts = []
    for row in range(-rows, rows + 1):
        y = row * step_y
        offset = 0.0 if row % 2 == 0 else spacing / 2
        cols = int((r_max + spacing) / spacing) + 2
        for col in range(-cols, cols + 1):
            x = col * spacing + offset
            if math.hypot(x, y) <= r_max:
                pts.append((x, y))
    return np.asarray(pts, dtype=float)


def worst_gap(points, grid: np.ndarray | None = None, r_min: float = R_MIN):
    """Largest angular gap (deg) over the grid, and #grid points with no point."""
    grid = check_grid() if grid is None else grid
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return 360.0, len(grid)
    diff = grid[:, None, :] - pts[None, :, :]           # (n, m, 2)
    dist = np.hypot(diff[:, :, 0], diff[:, :, 1])       # (n, m)
    inside = dist <= r_min
    counts = inside.sum(axis=1)
    empty = int((counts == 0).sum())
    ang = np.degrees(np.arctan2(-diff[:, :, 1], -diff[:, :, 0])) % 360.0
    ang = np.where(inside, ang, np.nan)
    ang.sort(axis=1)                                   # NaN at the end
    rows = np.arange(ang.shape[0])
    last = ang[rows, np.maximum(counts - 1, 0)]
    first = ang[rows, 0]
    diffs = ang[:, 1:] - ang[:, :-1]
    diffs = np.where(np.isfinite(diffs), diffs, 0.0)
    wrap = (first + 360.0) - last
    gaps = np.concatenate([diffs, wrap[:, None]], axis=1)
    gaps[counts <= 1, :] = 360.0
    return float(np.nanmax(gaps)), empty


def surround_ok(points, gap_limit: float = GAP_LIMIT_DEG, grid: np.ndarray | None = None):
    """True when every arena point is surrounded by measurement points."""
    pts = list(points)
    if len(pts) < 3:
        return False
    worst, empty = worst_gap(pts, grid)
    return empty == 0 and worst < gap_limit
