"""Compare strategy configurations over many random cases (offline)."""
from __future__ import annotations

import statistics
import sys

from arena import LocalRobot, MockArena, make_case
from strategy import Strategy

CONFIGS = {
    "base_v1": dict(ring_radius=1559.0, verify=True),
    "ring1200": dict(ring_radius=1200.0, verify=True),
    "nv_plain": dict(ring_radius=1200.0, verify=False, interleave=False, prune_tight_m=0.0),
    "nv_prune": dict(ring_radius=1200.0, verify=False, interleave=False),
    "nv_inter": dict(ring_radius=1200.0, verify=False, interleave=True),
    "nv_inter_norange": dict(
        ring_radius=1200.0, verify=False, interleave=True, prune_range=False
    ),
    "nv_inter_ring1150": dict(ring_radius=1150.0, verify=False, interleave=True),
    "nv_inter_ring1100": dict(ring_radius=1100.0, verify=False, interleave=True),
    "nv_inter300": dict(ring_radius=1200.0, verify=False, interleave=True, detour_limit_m=300.0),
    "nv_inter900": dict(ring_radius=1200.0, verify=False, interleave=True, detour_limit_m=900.0),
    "q4_plain": dict(mode="mixed", early_stop=False),
    "q4_early": dict(mode="mixed", early_stop=True),
    "q4_early_opp": dict(mode="mixed", early_stop=True, opportunistic=True),
}


def run(name: str, cases: int, seed0: int = 1, directional: int = 0):
    cfg = CONFIGS[name]
    ratios, totals, avgs, fails, disc = [], [], [], 0, []
    for i in range(cases):
        seed = seed0 + i
        case = make_case(seed, None, directional)
        arena = MockArena(case)
        robot = LocalRobot(arena)
        strat = Strategy(robot, verbose=False, **cfg)
        res = strat.run()
        cleared = len(res["cleared"])
        total = res["virtual_time"]
        ratios.append(cleared / case.n)
        totals.append(total)
        avgs.append(total / cleared if cleared else float("inf"))
        disc.append(getattr(strat, "t_after_discovery", 0.0))
        fails += cleared != case.n
    return {
        "name": name,
        "full_clear": 1 - fails / cases,
        "mean_ratio": statistics.mean(ratios),
        "mean_total": statistics.mean(totals),
        "mean_avg": statistics.mean(avgs),
        "max_avg": max(avgs),
        "mean_sweep": statistics.mean(disc),
    }


def main():
    cases = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    directional = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    names = sys.argv[3:] or list(CONFIGS)
    print(f"cases per config: {cases}   directional sources per case: {directional}")
    print(
        f"{'config':>22} {'full':>7} {'ratio':>7} {'total_s':>9} {'avg_s':>8}"
        f" {'worst':>8} {'sweep_s':>8}"
    )
    for name in names:
        r = run(name, cases, directional=directional)
        print(
            f"{r['name']:>22} {r['full_clear']:>7.0%} {r['mean_ratio']:>7.3f}"
            f" {r['mean_total']:>9.0f} {r['mean_avg']:>8.1f} {r['max_avg']:>8.1f}"
            f" {r['mean_sweep']:>8.0f}"
        )


if __name__ == "__main__":
    main()
