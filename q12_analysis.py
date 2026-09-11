"""问题 1 / 问题 2 的几何分析。

问题 1：给定检测点与示向度，构造交会定位区域（多边形），求其直径；并检验
"以定位区域直径为直径的圆能否覆盖该区域"。
问题 2：给定一个检测点和一条示向度，搜索第二个检测点的最优位置。
"""
from __future__ import annotations

import math
import random

import geometry
from arena import bearing_deg
from geometry import clip_wedge, dist, minimal_enclosing_circle, point_in_convex

HALF = 1.0  # 度，示向度误差 ±1°
BIG = 5000.0  # 定位区域裁剪用的外包正方形半边长（米）


def region_polygon(observations, half_deg: float = HALF, clip_radius: float = BIG):
    """交会定位区域：各 ±half_deg 楔形的交集（凸多边形）。"""
    poly = [
        (-clip_radius, -clip_radius),
        (clip_radius, -clip_radius),
        (clip_radius, clip_radius),
        (-clip_radius, clip_radius),
    ]
    for (x, y, theta) in observations:
        poly = clip_wedge(poly, (x, y), theta, half_deg)
        if not poly:
            return []
    return poly


def polygon_diameter(poly):
    """凸多边形直径：顶点两两距离的最大值（凸多边形直径必在顶点处取得）。"""
    best = 0.0
    pair = None
    for i in range(len(poly)):
        for j in range(i + 1, len(poly)):
            d = dist(poly[i], poly[j])
            if d > best:
                best, pair = d, (poly[i], poly[j])
    return best, pair


def diameter_disc_covers(poly, tol: float = 1e-9):
    """以定位区域直径为直径的圆（圆心取该直径中点）是否覆盖整个区域。"""
    diameter, pair = polygon_diameter(poly)
    if pair is None:
        return True, 0.0, 0.0
    center = ((pair[0][0] + pair[1][0]) / 2, (pair[0][1] + pair[1][1]) / 2)
    radius = diameter / 2
    worst = max(math.hypot(p[0] - center[0], p[1] - center[1]) for p in poly)
    return worst <= radius + tol, radius, worst


def min_enclosing_radius(poly):
    return minimal_enclosing_circle(poly).r


def scan_counterexamples(trials: int = 200000, seed: int = 0):
    """随机搜索"直径圆覆盖不了定位区域"的实例，返回最严重的几个。"""
    rng = random.Random(seed)
    worst = []
    for _ in range(trials):
        s1 = (rng.uniform(-1800, 1800), rng.uniform(-1800, 1800))
        s2 = (rng.uniform(-1800, 1800), rng.uniform(-1800, 1800))
        if dist(s1, s2) < 50:
            continue
        g = (rng.uniform(-1800, 1800), rng.uniform(-1800, 1800))
        obs = [
            (s1[0], s1[1], bearing_deg(s1, g)),
            (s2[0], s2[1], bearing_deg(s2, g)),
        ]
        poly = region_polygon(obs)
        if len(poly) < 3:
            continue
        diameter, _ = polygon_diameter(poly)
        if diameter < 1.0 or diameter > 4000.0:
            continue
        covers, radius, worst_r = diameter_disc_covers(poly)
        if not covers:
            worst.append((worst_r / radius, s1, s2, g, poly, diameter, worst_r))
    worst.sort(key=lambda t: -t[0])
    return worst


def scan_multi(n_points: int, trials: int = 60000, seed: int = 0, return_top: int = 3):
    """多检测点情形：搜索"直径圆覆盖不了定位区域"的最严重实例。"""
    rng = random.Random(seed)
    found = []
    for _ in range(trials):
        pts = [(rng.uniform(-1800, 1800), rng.uniform(-1800, 1800)) for _ in range(n_points)]
        g = (rng.uniform(-1800, 1800), rng.uniform(-1800, 1800))
        if min(dist(p, g) for p in pts) < 5.0:
            continue
        obs = [(p[0], p[1], bearing_deg(p, g)) for p in pts]
        poly = region_polygon(obs)
        if len(poly) < 3:
            continue
        diameter, _ = polygon_diameter(poly)
        if not (5.0 < diameter < 3000.0):
            continue
        covers, radius, worst_r = diameter_disc_covers(poly)
        if not covers:
            found.append((worst_r / radius, pts, g, poly, diameter, worst_r))
    found.sort(key=lambda t: -t[0])
    return found[:return_top]


def main():
    print("=== 问题 1：定位区域直径算法与覆盖性检验 ===")
    # 一个具体算例：两个检测点 + 一个干扰源
    s1, s2 = (-600.0, -300.0), (700.0, -200.0)
    g = (100.0, 900.0)
    obs = [(s1[0], s1[1], bearing_deg(s1, g)), (s2[0], s2[1], bearing_deg(s2, g))]
    poly = region_polygon(obs)
    diameter, pair = polygon_diameter(poly)
    covers, radius, worst_r = diameter_disc_covers(poly)
    r_min = min_enclosing_radius(poly)
    print(f"检测点 S1={s1} S2={s2} 干扰源 G={g}")
    print(f"定位区域顶点数 {len(poly)}")
    print(f"定位区域直径 D = {diameter:.2f} m（顶点 {pair[0]} - {pair[1]}）")
    print(f"直径圆半径 = {radius:.2f} m，区域内最远点到圆心 = {worst_r:.2f} m，能否覆盖：{covers}")
    print(f"最小包围圆半径 = {r_min:.2f} m（D/2 = {diameter / 2:.2f}，D/√3 = {diameter / math.sqrt(3):.2f}）")
    print()
    print("随机搜索反例（200000 次抽样，列出最严重的 5 个）：")
    cases = scan_counterexamples(200000, seed=1)
    print(f"找到 {len(cases)} 个反例")
    for ratio, a, b, c, p, d, wr in cases[:5]:
        print(
            f"  最远点/半径 = {ratio:.4f}  D={d:.1f} m  S1=({a[0]:.0f},{a[1]:.0f})"
            f" S2=({b[0]:.0f},{b[1]:.0f}) G=({c[0]:.0f},{c[1]:.0f})"
        )


if __name__ == "__main__":
    main()
