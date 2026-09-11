"""Entry point.

Offline evaluation (no simulator needed):
    python run.py --cases 20 [--sources 13] [--directional 0]

One verbose run against the local mock:
    python run.py --cases 1 --verbose

Run against the official simulator already started in the simulator GUI:
    python run.py --real --robot-id <参赛队号>

Serve a local mock simulator on port 2026 for end-to-end debugging:
    python run.py --serve --seed 3
"""
from __future__ import annotations

import argparse
import statistics
import sys

from arena import LocalRobot, MockArena, make_case
from strategy import Strategy


def run_case(seed: int, n_sources=None, n_directional=0, verbose=False, robot=None, mode="omni"):
    case = make_case(seed, n_sources, n_directional)
    if robot is None:
        arena = MockArena(case)
        robot = LocalRobot(arena)
    strat = Strategy(robot, verbose=verbose, mode=mode)
    result = strat.run()
    cleared = len(result["cleared"])
    total = result["virtual_time"]
    return {
        "seed": seed,
        "truth_n": case.n,
        "cleared": cleared,
        "ratio": cleared / case.n,
        "total_time": total,
        "avg_time": total / cleared if cleared else float("inf"),
        "n_measure": getattr(robot, "n_measure", None),
        "log": strat.log,
    }


def evaluate(
    cases: int,
    n_sources=None,
    n_directional=0,
    seed0: int = 1,
    verbose: bool = False,
    mode: str = "omni",
):
    rows = []
    for i in range(cases):
        seed = seed0 + i
        rows.append(run_case(seed, n_sources, n_directional, verbose=verbose, mode=mode))
    print()
    print(f"{'seed':>6} {'truth':>6} {'cleared':>8} {'ratio':>7} {'total_s':>10} {'avg_s':>9}")
    for r in rows:
        print(
            f"{r['seed']:>6} {r['truth_n']:>6} {r['cleared']:>8} {r['ratio']:>7.2f}"
            f" {r['total_time']:>10.0f} {r['avg_time']:>9.1f}"
        )
    ratios = [r["ratio"] for r in rows]
    totals = [r["total_time"] for r in rows]
    avgs = [r["avg_time"] for r in rows]
    print()
    print(f"cases            : {len(rows)}")
    print(f"full-clear rate  : {sum(1 for x in ratios if x >= 0.999) / len(ratios):.2%}")
    print(f"mean clear ratio : {statistics.mean(ratios):.3f}")
    print(f"mean total time  : {statistics.mean(totals):.0f} s")
    print(f"mean avg time    : {statistics.mean(avgs):.1f} s")
    print(f"worst avg time   : {max(avgs):.1f} s")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=0)
    ap.add_argument("--seed0", type=int, default=1)
    ap.add_argument("--sources", type=int, default=None)
    ap.add_argument("--directional", type=int, default=0)
    ap.add_argument("--mode", type=str, default="omni", choices=["omni", "mixed"])
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--real", action="store_true", help="talk to the official simulator")
    ap.add_argument("--robot-id", type=str, default="")
    ap.add_argument("--base-url", type=str, default="http://127.0.0.1:2026")
    ap.add_argument("--wait", type=float, default=150.0, help="seconds to wait for the interface")
    ap.add_argument("--log", type=str, default="robot_log.jsonl")
    ap.add_argument("--serve", action="store_true", help="serve the local mock simulator")
    ap.add_argument("--port", type=int, default=2026)
    args = ap.parse_args()

    if args.serve:
        from mock_server import main as serve_main

        sys.argv = ["mock_server", "--port", str(args.port), "--seed", str(args.seed0)]
        serve_main()
        return

    if args.real:
        from robot import HttpRobot

        robot = HttpRobot(
            args.robot_id,
            base_url=args.base_url,
            log_path=args.log,
            enter_wait=args.wait,
        )
        strat = Strategy(robot, verbose=True, mode=args.mode)
        result = strat.run()
        print(
            f"cleared {len(result['cleared'])} channels,"
            f" total virtual time {result['virtual_time']:.0f} s,"
            f" average {result['virtual_time'] / max(len(result['cleared']), 1):.1f} s"
        )
        robot.close()
        return

    if args.cases:
        evaluate(
            args.cases,
            args.sources,
            args.directional,
            args.seed0,
            verbose=args.verbose,
            mode=args.mode,
        )
        return

    ap.print_help()


if __name__ == "__main__":
    main()
