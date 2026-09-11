"""Shrink the directional search pattern: drop every lattice point that is not
needed for the "surround every arena point" guarantee, then shorten the tour.
"""
from __future__ import annotations

import math
import sys

from arena import triangular_lattice
from surround import check_grid, surround_ok, worst_gap

GRID = check_grid()


def tour_length(points, order):
    return sum(
        math.hypot(
            points[order[i]][0] - points[order[i + 1]][0],
            points[order[i]][1] - points[order[i + 1]][1],
        )
        for i in range(len(order) - 1)
    )


def nn_tour(points, start=(0.0, 0.0)):
    remaining = list(range(len(points)))
    cur = start
    order = []
    while remaining:
        nxt = min(
            remaining,
            key=lambda i: math.hypot(points[i][0] - cur[0], points[i][1] - cur[1]),
        )
        order.append(nxt)
        cur = points[nxt]
        remaining.remove(nxt)
    return order


def two_opt(points, order):
    best, best_len = order[:], tour_length(points, order)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                cand = best[:i] + best[i : j + 1][::-1] + best[j + 1:]
                length = tour_length(points, cand)
                if length < best_len - 1e-9:
                    best, best_len, improved = cand, length, True
    return best, best_len


def thin(points, gap_limit: float = 168.0, passes: int = 6):
    pts = list(points)
    for _ in range(passes):
        removed = False
        for p in sorted(pts, key=lambda q: -math.hypot(*q)):
            trial = [q for q in pts if q != p]
            if surround_ok(trial, gap_limit=gap_limit, grid=GRID):
                pts = trial
                removed = True
        if not removed:
            break
    return pts


def report(name, pts):
    order = nn_tour(pts)
    order, length = two_opt(pts, order)
    worst, empty = worst_gap(pts, GRID)
    print(
        f"{name:>28}: {len(pts):3d} points  tour={length:7.0f} m = {length / 5:6.0f} s"
        f"  worst gap={worst:6.1f} deg  empty={empty}"
    )
    return pts, order


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else 168.0
    for spacing, pad in ((900.0, 700.0), (1000.0, 1000.0), (800.0, 800.0), (1200.0, 1400.0)):
        pts = triangular_lattice(spacing, pad)
        report(f"lattice {spacing:.0f}/{pad:.0f}", pts)
        thinned = thin(pts, gap_limit=limit)
        report(f"  thinned (gap<{limit:.0f})", thinned)


if __name__ == "__main__":
    main()
