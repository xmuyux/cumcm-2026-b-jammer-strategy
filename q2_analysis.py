"""问题 2：第二个检测点的选择策略。

设第一个检测点 S1 在原点，测得示向度指向正北（旋转对称，可固定方向），
干扰源位于该射线上、距离 r 未知。对候选第二检测点 S2 = d*(cos psi, sin psi)，
计算交会定位区域（两个 ±1° 楔形之交）的直径 D，并对 r 的最坏情况取上确界。
目标：找使最坏情况直径最小的 S2，并给出"候选区域"。
"""
from __future__ import annotations

import math
import sys

from arena import bearing_deg
from geometry import dist
from q12_analysis import region_polygon, polygon_diameter

THETA1 = 90.0  # 示向度指向正北
SOURCE_POS = (0.0, 0.0)  # S1


def worst_diameter(d: float, alpha_deg: float, r_values, range_limit: float | None = None):
    """候选点在最坏源距离下的定位区域直径。

    d: 与 S1 的距离；alpha_deg: 相对第一条示向度方向（正北）的偏角。
    range_limit: 若给定，则要求第二检测点在最坏情况下也能收到信号
    （到干扰源距离不超过该值，即最小有效接收半径 1000 m）。
    """
    psi = math.radians(THETA1 + alpha_deg)
    s2 = (d * math.cos(psi), d * math.sin(psi))
    if math.hypot(*s2) < 1e-6:
        return None
    worst = 0.0
    for r in r_values:
        g = (r * math.cos(math.radians(THETA1)), r * math.sin(math.radians(THETA1)))
        if range_limit is not None and dist(s2, g) > range_limit:
            return None
        obs = [(0.0, 0.0, THETA1), (s2[0], s2[1], bearing_deg(s2, g))]
        poly = region_polygon(obs)
        if len(poly) < 3:
            return None
        diameter, _ = polygon_diameter(poly)
        if not math.isfinite(diameter) or diameter > 20000:
            return None
        worst = max(worst, diameter)
    return worst


def search(range_limit, r_values, label):
    best = []
    for d in range(200, 3001, 100):
        for alpha in range(-90, 91, 5):
            value = worst_diameter(float(d), float(alpha), r_values, range_limit)
            if value:
                best.append((value, float(d), float(alpha)))
    best.sort()
    print(f"--- {label} ---")
    for value, d, alpha in best[:5]:
        print(f"  D_worst = {value:7.1f} m   d = {d:6.0f} m   alpha = {alpha:+5.0f} deg")
    return best


def main():
    r_values = [300.0, 500.0, 700.0, 900.0, 1100.0, 1300.0, 1500.0]
    best = search(None, r_values, "无距离约束")
    best = search(1000.0, r_values, "要求第二点在最坏情况下仍能收到信号（<=1000 m）") or best
    if not best:
        return
    d_opt, alpha_opt = best[0][1], best[0][2]
    print()
    print(f"最优：d = {d_opt:.0f} m, alpha = {alpha_opt:+.0f} deg（相对示向度）")
    print("该点在各源距离下的定位区域直径：")
    psi = math.radians(THETA1 + alpha_opt)
    s2 = (d_opt * math.cos(psi), d_opt * math.sin(psi))
    for r in r_values:
        g = (r * math.cos(math.radians(THETA1)), r * math.sin(math.radians(THETA1)))
        poly = region_polygon([(0.0, 0.0, THETA1), (s2[0], s2[1], bearing_deg(s2, g))])
        diameter, _ = polygon_diameter(poly)
        print(f"  r = {r:5.0f} m -> D = {diameter:7.2f} m")
    print()
    print("候选区域（最坏情况直径不超过最优值的 1.5 倍的 (d, psi)）：")
    limit = best[0][0] * 1.5
    inside = [(d, alpha) for value, d, alpha in best if value <= limit]
    ds = [d for d, _ in inside]
    psis = [p for _, p in inside]
    print(f"  点数 {len(inside)}/{len(best)}")
    print(f"  d 范围 {min(ds):.0f}–{max(ds):.0f} m, alpha 范围 {min(psis):+.0f}–{max(psis):+.0f} deg")
    with open("q2_grid.txt", "w", encoding="utf-8") as handle:
        handle.write("d alpha worst_diameter\n")
        for value, d, alpha in sorted(best, key=lambda t: (t[1], t[2])):
            handle.write(f"{d:.0f} {alpha:.0f} {value:.2f}\n")
    print("  网格数据已写入 q2_grid.txt（用于画图）")


if __name__ == "__main__":
    main()
