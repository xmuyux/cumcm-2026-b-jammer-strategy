"""Convex geometry helpers: polygon clipping and minimal enclosing circles.

Everything here is built so that the *guarantee* direction is conservative:
wherever we approximate, we approximate on the safe side (the produced region
always contains the true source position).
"""
from __future__ import annotations

import math
import random
import sys

sys.setrecursionlimit(20000)

Point = tuple[float, float]


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def circumscribed_polygon(center: Point, radius: float, n: int = 360) -> list[Point]:
    """Polygon that circumscribes the circle (never smaller than the disc)."""
    r = radius / math.cos(math.pi / n) if n > 2 else radius
    cx, cy = center
    return [
        (cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def clip_halfplane(poly: list[Point], a: Point, b: Point, keep_left: bool = True) -> list[Point]:
    """Keep the part of `poly` on the left (or right) of the directed line a->b."""
    if not poly:
        return []
    sign = 1.0 if keep_left else -1.0
    ax, ay = a
    ex, ey = b[0] - ax, b[1] - ay

    def side(p: Point) -> float:
        return sign * (ex * (p[1] - ay) - ey * (p[0] - ax))

    out: list[Point] = []
    n = len(poly)
    for i in range(n):
        cur = poly[i]
        nxt = poly[(i + 1) % n]
        sc, sn = side(cur), side(nxt)
        if sc >= 0:
            out.append(cur)
        if (sc > 0 and sn < 0) or (sc < 0 and sn > 0):
            t = sc / (sc - sn)
            out.append((cur[0] + t * (nxt[0] - cur[0]), cur[1] + t * (nxt[1] - cur[1])))
    return out


def clip_wedge(
    poly: list[Point], apex: Point, bearing_deg: float, half_deg: float
) -> list[Point]:
    """Keep the points inside the wedge apex +- half_deg around bearing_deg."""
    lo = math.radians(bearing_deg - half_deg)
    hi = math.radians(bearing_deg + half_deg)
    far = 1.0e7
    p_lo = (apex[0] + far * math.cos(lo), apex[1] + far * math.sin(lo))
    p_hi = (apex[0] + far * math.cos(hi), apex[1] + far * math.sin(hi))
    # left of apex->p_lo  == counter-clockwise side  == angles > lo
    poly = clip_halfplane(poly, apex, p_lo, keep_left=True)
    # right of apex->p_hi == clockwise side == angles < hi
    poly = clip_halfplane(poly, apex, p_hi, keep_left=False)
    return poly


def polygon_centroid(poly: list[Point]) -> Point:
    if not poly:
        return (0.0, 0.0)
    xs = sum(p[0] for p in poly)
    ys = sum(p[1] for p in poly)
    return (xs / len(poly), ys / len(poly))


def clip_convex(subject: list[Point], clip_poly: list[Point]) -> list[Point]:
    """Intersect a convex subject polygon with a convex clip polygon (CCW)."""
    out = subject
    n = len(clip_poly)
    for i in range(n):
        if not out:
            return []
        out = clip_halfplane(out, clip_poly[i], clip_poly[(i + 1) % n], keep_left=True)
    return out


class Circle:
    __slots__ = ("x", "y", "r")

    def __init__(self, x: float, y: float, r: float):
        self.x, self.y, self.r = x, y, r

    @property
    def center(self) -> Point:
        return (self.x, self.y)

    def contains(self, p: Point, eps: float = 1e-9) -> bool:
        return math.hypot(p[0] - self.x, p[1] - self.y) <= self.r + eps


def _circle_two(a: Point, b: Point) -> Circle:
    cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    return Circle(cx, cy, dist(a, b) / 2)


def _circle_three(a: Point, b: Point, c: Point) -> Circle:
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        # collinear: fall back to the two farthest points
        best = max(((a, b), (a, c), (b, c)), key=lambda pair: dist(*pair))
        return _circle_two(*best)
    ux = (
        (ax * ax + ay * ay) * (by - cy)
        + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by)
    ) / d
    uy = (
        (ax * ax + ay * ay) * (cx - bx)
        + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax)
    ) / d
    center = (ux, uy)
    return Circle(ux, uy, dist(center, a))


def _trivial(pts: list[Point]) -> Circle:
    if not pts:
        return Circle(0.0, 0.0, 0.0)
    if len(pts) == 1:
        return Circle(pts[0][0], pts[0][1], 0.0)
    if len(pts) == 2:
        return _circle_two(pts[0], pts[1])
    return _circle_three(pts[0], pts[1], pts[2])


def _welzl(pts: list[Point], boundary: list[Point], n: int) -> Circle:
    if n == 0 or len(boundary) == 3:
        return _trivial(boundary)
    p = pts[n - 1]
    c = _welzl(pts, boundary, n - 1)
    if c.contains(p):
        return c
    return _welzl(pts, boundary + [p], n - 1)


def minimal_enclosing_circle(points: list[Point], seed: int = 0xC0FFEE) -> Circle:
    pts = list(points)
    if not pts:
        return Circle(0.0, 0.0, 0.0)
    rng = random.Random(seed)
    rng.shuffle(pts)
    return _welzl(pts, [], len(pts))


def ray_intersection(p: Point, a_deg: float, q: Point, b_deg: float, max_t: float = 20000.0):
    """Intersection of ray(p, a) and ray(q, b); None if parallel or behind."""
    ax, ay = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
    bx, by = math.cos(math.radians(b_deg)), math.sin(math.radians(b_deg))
    den = ax * by - ay * bx
    if abs(den) < 1e-9:
        return None
    dx, dy = q[0] - p[0], q[1] - p[1]
    t = (dx * by - dy * bx) / den
    s = (dx * ay - dy * ax) / den
    if t <= 0 or s <= 0 or t > max_t or s > max_t:
        return None
    return (p[0] + t * ax, p[1] + t * ay)


def point_in_convex(poly: list[Point], p: Point) -> bool:
    """True when p lies inside (or on) a counter-clockwise convex polygon."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -1e-7:
            return False
    return True
