"""Search parameterised search patterns (inner lattice + outer rings) for the
problem-4 "surround every arena point" guarantee with the shortest tour."""
from __future__ import annotations

import math
import sys

from optimize_design import nn_tour, tour_length, two_opt
from surround import check_grid, worst_gap

GRID = check_grid()
ARENA = 1800.0


def inner_lattice(spacing: float, r_max: float = ARENA):
    pts = []
    step_y = spacing * math.sqrt(3) / 2
    rows = int(r_max / step_y) + 2
    for row in range(-rows, rows + 1):
        y = row * step_y
        offset = 0.0 if row % 2 == 0 else spacing / 2
        cols = int((r_max + spacing) / spacing) + 2
        for col in range(-cols, cols + 1):
            x = col * spacing + offset
            if math.hypot(x, y) <= r_max + 1e-9:
                pts.append((x, y))
    return pts


def ring(radius: float, n: int, phase_deg: float = 0.0):
    return [
        (
            radius * math.cos(math.radians(phase_deg + 360.0 * k / n)),
            radius * math.sin(math.radians(phase_deg + 360.0 * k / n)),
        )
        for k in range(n)
    ]


def evaluate(pts, gap_limit: float = 168.0):
    worst, empty = worst_gap(pts, GRID)
    if empty or worst >= gap_limit:
        return None
    order = nn_tour(pts)
    order, length = two_opt(pts, order)
    return length, worst


def main():
    best = []
    gap_limit = float(sys.argv[1]) if len(sys.argv) > 1 else 168.0
    grid_m = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
    grid = check_grid(grid_m)
    for spacing in (850.0, 900.0, 950.0, 1000.0):
        core = inner_lattice(spacing)
        for radius in (1900.0, 1950.0, 2000.0, 2050.0, 2100.0, 2150.0, 2200.0):
            for n in (10, 12, 13, 14, 16):
                for phase in (0.0, 15.0, 30.0):
                    pts = core + ring(radius, n, phase)
                    worst, empty = worst_gap(pts, grid)
                    if empty or worst >= gap_limit:
                        continue
                    order = nn_tour(pts)
                    order, length = two_opt(pts, order)
                    res = (length, worst)
                    if res:
                        length, worst = res
                        best.append((length, spacing, radius, n, phase, len(pts), worst))
    best.sort()
    print(f"{'tour_s':>7} {'spacing':>8} {'r_out':>7} {'n_out':>6} {'phase':>6} {'pts':>5} {'gap':>7}")
    for length, spacing, radius, n, phase, count, worst in best[:15]:
        print(
            f"{length / 5:7.0f} {spacing:8.0f} {radius:7.0f} {n:6d} {phase:6.0f}"
            f" {count:5d} {worst:7.1f}"
        )
    if not best:
        print("no feasible design found in this family")


if __name__ == "__main__":
    main()
