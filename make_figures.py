"""生成论文用图：问题 1 的定位区域与覆盖反例，问题 2 的第二检测点选择图。"""
from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from arena import bearing_deg
from geometry import minimal_enclosing_circle
from q12_analysis import (
    polygon_diameter,
    region_polygon,
    scan_multi,
)

FONT = r"C:\Windows\Fonts\msyh.ttc"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def draw_case(ax, observations, source, title, half_deg=1.0, show_rays=True):
    poly = region_polygon(observations, half_deg)
    diameter, pair = polygon_diameter(poly)
    covers, radius, worst_r = diameter_disc_covers_local(poly)
    mec = minimal_enclosing_circle(poly)

    if show_rays:
        span = max(60.0, diameter * 1.5)
        for (x, y, theta) in observations:
            for sign in (-1.0, 1.0):
                rad = math.radians(theta + sign * half_deg)
                ax.plot(
                    [x, x + span * math.cos(rad)],
                    [y, y + span * math.sin(rad)],
                    "--",
                    color="0.6",
                    lw=0.8,
                )
            rad = math.radians(theta)
            ax.plot(
                [x, x + span * math.cos(rad)],
                [y, y + span * math.sin(rad)],
                "-",
                color="tab:blue",
                lw=1.2,
            )
    px = [p[0] for p in poly] + [poly[0][0]]
    py = [p[1] for p in poly] + [poly[0][1]]
    ax.fill(px, py, color="tab:red", alpha=0.18, zorder=2)
    ax.plot(px, py, color="tab:red", lw=1.6, zorder=3)

    center = ((pair[0][0] + pair[1][0]) / 2, (pair[0][1] + pair[1][1]) / 2)
    ax.plot(
        [pair[0][0], pair[1][0]], [pair[0][1], pair[1][1]],
        "-", color="tab:green", lw=1.4, zorder=4, label="定位区域直径 D",
    )
    ax.add_patch(
        Circle(center, radius, fill=False, ls="--", color="tab:green", lw=1.2,
               zorder=4, label="以 D 为直径的圆")
    )
    ax.add_patch(
        Circle(mec.center, mec.r, fill=False, ls=":", color="tab:orange", lw=1.4,
               zorder=4, label="最小包围圆")
    )
    if not covers:
        outside = [p for p in poly
                   if math.hypot(p[0] - center[0], p[1] - center[1]) > radius + 1e-9]
        ax.scatter([p[0] for p in outside], [p[1] for p in outside],
                   s=45, color="tab:purple", zorder=6, label="跑到圆外的区域顶点")

    for (x, y, _), name in zip(observations, ["S1", "S2", "S3", "S4"][: len(observations)]):
        ax.plot(x, y, "k^", ms=7, zorder=5)
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(6, -12), fontsize=9)
    ax.plot(source[0], source[1], "r*", ms=13, zorder=6)
    ax.annotate("G", source, textcoords="offset points", xytext=(6, 4), fontsize=10)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.25, ls=":")


def diameter_disc_covers_local(poly, tol=1e-9):
    diameter, pair = polygon_diameter(poly)
    if pair is None:
        return True, 0.0, 0.0
    center = ((pair[0][0] + pair[1][0]) / 2, (pair[0][1] + pair[1][1]) / 2)
    radius = diameter / 2
    worst = max(math.hypot(p[0] - center[0], p[1] - center[1]) for p in poly)
    return worst <= radius + tol, radius, worst


def figure_q1(path: str = "fig_q1_region.png"):
    # 一个普通算例
    s1, s2 = (-600.0, -300.0), (700.0, -200.0)
    g1 = (100.0, 900.0)
    obs1 = [(s1[0], s1[1], bearing_deg(s1, g1)), (s2[0], s2[1], bearing_deg(s2, g1))]
    # 反例（来自随机搜索的最严重构型之一）
    picks = scan_multi(2, trials=40000, seed=2, return_top=1)
    ratio, pts, g2, poly2, d2, _ = picks[0]
    obs2 = [(p[0], p[1], bearing_deg(p, g2)) for p in pts]

    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    draw_case(axes[0], obs1, g1, "(a) 直径圆可以覆盖定位区域")
    draw_case(axes[1], obs2, g2, f"(b) 反例：直径圆覆盖不了（最远点超出 {ratio - 1:.1%}）")
    axes[1].legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    print("saved", path)
    return ratio, pts, g2


def figure_q2(path: str = "fig_q2_candidate.png"):
    import numpy as np

    from q2_analysis import worst_diameter

    r_values = [300.0, 500.0, 700.0, 900.0, 1100.0, 1300.0, 1500.0]
    ds = np.arange(200.0, 3001.0, 100.0)
    alphas = np.arange(-90.0, 91.0, 5.0)
    grid = np.full((len(alphas), len(ds)), np.nan)
    constrained = np.full((len(alphas), len(ds)), np.nan)
    for i, alpha in enumerate(alphas):
        for j, d in enumerate(ds):
            v = worst_diameter(float(d), float(alpha), r_values)
            if v:
                grid[i, j] = v
            v2 = worst_diameter(float(d), float(alpha), r_values, 1000.0)
            if v2:
                constrained[i, j] = v2

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, data, title in (
        (axes[0], grid, "(a) 无距离约束：最优 (d=1600 m, ±40°)"),
        (axes[1], constrained, "(b) 要求第二点在最坏情况仍能收到信号：最优 (d=1200 m, ±25°)"),
    ):
        im = ax.contourf(ds, alphas, data, levels=18, cmap="viridis_r")
        fig.colorbar(im, ax=ax, label="最坏情况定位区域直径 / m")
        best_idx = np.unravel_index(np.nanargmin(data), data.shape)
        best_d, best_a = ds[best_idx[1]], alphas[best_idx[0]]
        ax.plot(best_d, best_a, "r*", ms=14, label=f"最优点 d={best_d:.0f} m, α={best_a:+.0f}°")
        limit = np.nanmin(data) * 1.5
        ax.contour(ds, alphas, data, levels=[limit], colors="w", linewidths=1.6)
        ax.set_xlabel("第二检测点到 S1 的距离 d / m")
        ax.set_ylabel("相对第一示向度方向的偏角 α / (°)")
        ax.set_title(title, fontsize=10)
        ax.legend(loc="lower center", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    print("saved", path)


if __name__ == "__main__":
    figure_q1()
    figure_q2()
