"""Design and verify the search lattice used for directional sources (问题 4).

A directional source at q is only heard from points inside its +-90 deg half
plane. Measuring all channels at a set of points therefore detects it only if
some measurement point p satisfies
    |p - q| <= R_eff   and   p inside the half plane through q.
Since R_eff >= 1000 m this is guaranteed when the measurement points that are
within 1000 m of q *surround* q (q lies in their convex hull): any half plane
through q then contains at least one of them.
"""
from __future__ import annotations

import math
import random

SPACING = 1600.0
R_MIN = 1000.0  # smallest possible effective radius
ARENA_R = 1800.0


def triangular_lattice(spacing: float = SPACING, pad: float | None = None):
    if pad is None:
        pad = spacing / math.sqrt(3.0)  # so every arena point keeps its cell
    r_max = ARENA_R + pad
    pts = []
    rows = int(r_max / (spacing * math.sqrt(3) / 2)) + 2
    for row in range(-rows, rows + 1):
        y = row * spacing * math.sqrt(3) / 2
        offset = 0.0 if row % 2 == 0 else spacing / 2
        for col in range(-rows - 1, rows + 2):
            x = col * spacing + offset
            if math.hypot(x, y) <= r_max + 1e-9:
                pts.append((x, y))
    return pts


def tour_length(points, order) -> float:
    total = 0.0
    for i in range(len(order) - 1):
        a, b = points[order[i]], points[order[i + 1]]
        total += math.hypot(a[0] - b[0], a[1] - b[1])
    return total


def nearest_neighbour_order(points, start=(0.0, 0.0)):
    remaining = list(range(len(points)))
    cur = start
    order = []
    while remaining:
        nxt = min(remaining, key=lambda i: math.hypot(points[i][0] - cur[0], points[i][1] - cur[1]))
        order.append(nxt)
        cur = points[nxt]
        remaining.remove(nxt)
    return order


def two_opt(points, order, rounds: int = 60):
    best = order[:]
    best_len = tour_length(points, best)
    improved = True
    while improved and rounds > 0:
        improved = False
        rounds -= 1
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                length = tour_length(points, cand)
                if length < best_len - 1e-9:
                    best, best_len, improved = cand, length, True
    return best, best_len


def surround_ok(points, samples: int = 4000, seed: int = 3, r_min: float = R_MIN):
    """Check that every arena point is surrounded by measurement points."""
    rng = random.Random(seed)
    worst_gap = 0.0
    bad = 0
    for _ in range(samples):
        r = ARENA_R * math.sqrt(rng.random())
        a = rng.uniform(0.0, 2 * math.pi)
        q = (r * math.cos(a), r * math.sin(a))
        bearings = sorted(
            math.degrees(math.atan2(p[1] - q[1], p[0] - q[0])) % 360.0
            for p in points
            if math.hypot(p[0] - q[0], p[1] - q[1]) <= r_min
        )
        if not bearings:
            bad += 1
            worst_gap = 360.0
            continue
        if len(bearings) == 1:
            gaps = [360.0]
        else:
            gaps = [
                (bearings[(i + 1) % len(bearings)] - bearings[i]) % 360.0
                for i in range(len(bearings))
            ]
        gap = max(gaps)
        worst_gap = max(worst_gap, gap)
        if gap >= 180.0:
            bad += 1
    return worst_gap, bad


def main():
    print(f"{'spacing':>8} {'pad':>6} {'points':>7} {'tour_s':>8} {'max gap':>8} {'bad':>6}")
    for spacing in (900.0, 950.0, 1000.0):
        for pad in (400.0, 600.0, 800.0, 1000.0):
            pts = triangular_lattice(spacing, pad=pad)
            order = nearest_neighbour_order(pts)
            order, length = two_opt(pts, order)
            gap, bad = surround_ok(pts, 4000, seed=5)
            print(
                f"{spacing:8.0f} {pad:6.0f} {len(pts):7d} {length / 5:8.0f}"
                f" {gap:8.1f} {bad:6d}"
            )


if __name__ == "__main__":
    main()
