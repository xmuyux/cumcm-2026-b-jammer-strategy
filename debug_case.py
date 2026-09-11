"""Inspect one case: per-channel observations versus ground truth."""
from __future__ import annotations

import math
import sys

from arena import LocalRobot, MockArena, make_case
from strategy import Strategy


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 212
    directional = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    mode = sys.argv[3] if len(sys.argv) > 3 else "mixed"
    case = make_case(seed, None, directional)
    arena = MockArena(case)
    strategy = Strategy(LocalRobot(arena), verbose=False, mode=mode)
    result = strategy.run()
    print(f"seed={seed} truth={case.n} cleared={len(result['cleared'])} total={result['virtual_time']:.0f}s")
    for src in sorted(case.sources, key=lambda s: s.channel):
        st = strategy.state[src.channel]
        kind = "DIR " if src.directional else "OMNI"
        obs = " ".join(f"({o[0]:.0f},{o[1]:.0f})@{o[2]:.0f}" for o in st.obs[:6])
        print(
            f"ch{src.channel:>2} {kind} pos=({src.x:7.0f},{src.y:7.0f})"
            f" R={src.eff_radius:5.0f} dir={src.direction:5.0f}"
            f" cleared={int(st.cleared)} obs={len(st.obs)} {obs}"
        )
        if not st.cleared and st.obs:
            _, circle = strategy.region(src.channel)
            est = strategy.estimate_position(src.channel)
            print(
                f"      region r={circle.r:7.1f} m  est=({est[0]:.0f},{est[1]:.0f})"
                f"  est err={math.hypot(est[0] - src.x, est[1] - src.y):.0f} m"
            )


if __name__ == "__main__":
    main()
