"""CUMCM 2026 problem B: arena rules, an offline case generator and a mock arena.

The mock follows 题目附录 1-4 and B 题附件 1/附件 2 exactly (timing, coverage,
20 m clearing radius, 5 m "too close" threshold, per-location bearing error).
It is used to develop and tune the strategy without burning real test runs.
"""
from __future__ import annotations

import math
import random
import zlib

import geometry

ARENA_RADIUS = 1800.0
MOVE_SPEED = 5.0
T_MEASURE = 5.0
T_SWITCH = 1.0
T_CLEAR_OK = 5.0
T_CLEAR_FAIL = 3.0
R_CLEAR = 20.0
R_NEAR = 5.0
BEARING_HALF_DEG = 1.0
N_CHANNELS = 20
MIN_EFF_RADIUS = 1000.0
MAX_EFF_RADIUS = 1500.0
MAX_VIRTUAL_TIME = 360000.0
MAX_REAL_TIME = 1200.0
MIN_SOURCES, MAX_SOURCES = 10, 16


def bearing_deg(a, b) -> float:
    """Bearing from a to b, degrees counter-clockwise from +x (east)."""
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0


class Source:
    __slots__ = ("channel", "x", "y", "eff_radius", "directional", "direction")

    def __init__(self, channel, x, y, eff_radius, directional=False, direction=0.0):
        self.channel = channel
        self.x = x
        self.y = y
        self.eff_radius = eff_radius
        self.directional = directional
        self.direction = direction

    @property
    def pos(self):
        return (self.x, self.y)

    def covers(self, p) -> bool:
        if not self.directional:
            return True
        d = bearing_deg(self.pos, p)
        diff = abs((d - self.direction + 180.0) % 360.0 - 180.0)
        return diff <= 90.0


class Case:
    def __init__(self, sources: list[Source], seed: int):
        self.sources = sources
        self.seed = seed

    @property
    def n(self) -> int:
        return len(self.sources)

    @property
    def n_directional(self) -> int:
        return sum(1 for s in self.sources if s.directional)


def make_case(seed: int, n_sources: int | None = None, n_directional: int = 0) -> Case:
    rng = random.Random(seed)
    if n_sources is None:
        n_sources = rng.randint(MIN_SOURCES, MAX_SOURCES)
    channels = rng.sample(range(1, N_CHANNELS + 1), n_sources)
    sources = []
    for index, ch in enumerate(channels):
        r = ARENA_RADIUS * math.sqrt(rng.random())
        a = rng.uniform(0.0, 2 * math.pi)
        directional = index < n_directional
        sources.append(
            Source(
                ch,
                r * math.cos(a),
                r * math.sin(a),
                rng.uniform(MIN_EFF_RADIUS, MAX_EFF_RADIUS),
                directional,
                rng.uniform(0.0, 360.0),
            )
        )
    return Case(sources, seed)


class MockArena:
    """In-process implementation of the simulator semantics."""

    def __init__(self, case: Case, check_limits: bool = True):
        self.case = case
        self.by_channel = {s.channel: s for s in case.sources}
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        self.entered = False
        self.exited = False
        self.cleared: set[int] = set()
        self.n_measure = 0
        self.n_clear = 0
        self.n_clear_fail = 0
        self.check_limits = check_limits

    # -- helpers ---------------------------------------------------------
    def _travel_time(self, x: float, y: float) -> float:
        return math.hypot(x - self.pos[0], y - self.pos[1]) / MOVE_SPEED

    def _bearing_error(self, x: float, y: float, channel: int) -> float:
        """Deterministic error in [-1, 1] deg, fixed for a given (place, channel)."""
        key = f"{self.case.seed}|{round(x, 3):.3f}|{round(y, 3):.3f}|{channel}"
        h = zlib.crc32(key.encode("utf-8")) / 0xFFFFFFFF
        return 2.0 * h - 1.0

    def _limits_exceeded(self) -> bool:
        return self.check_limits and self.virtual_time > MAX_VIRTUAL_TIME

    # -- commands --------------------------------------------------------
    def enter(self) -> dict:
        if self.entered:
            return {"accepted": False, "virtual_time_s": 0}
        self.entered = True
        return {
            "accepted": True,
            "virtual_time_s": 0.0,
            "max_virtual_duration_s": MAX_VIRTUAL_TIME,
            "max_real_duration_s": MAX_REAL_TIME,
            "remaining_real_duration_s": MAX_REAL_TIME,
        }

    def exit(self) -> dict:
        self.exited = True
        return {"accepted": True, "virtual_time_s": self.virtual_time, "exit_reason": "user_exit"}

    def measure(self, x: float, y: float, channel: int) -> dict:
        if not self.entered or self.exited:
            return {"accepted": False, "virtual_time_s": 0}
        dt = self._travel_time(x, y)
        if channel != self.channel:
            dt += T_SWITCH
        dt += T_MEASURE
        self.virtual_time += dt
        self.pos = (x, y)
        self.channel = channel
        self.n_measure += 1
        if self._limits_exceeded():
            self.exited = True
        src = self.by_channel.get(channel)
        out = {"accepted": True, "virtual_time_s": round(self.virtual_time, 6)}
        if src is None or channel in self.cleared:
            out["measure_result"] = "no_signal"
            return out
        d = math.hypot(x - src.x, y - src.y)
        if d > src.eff_radius or not src.covers((x, y)):
            out["measure_result"] = "no_signal"
            return out
        if d <= R_NEAR:
            out["measure_result"] = "near"
            return out
        svd = (bearing_deg((x, y), src.pos) + self._bearing_error(x, y, channel)) % 360.0
        out["measure_result"] = "direction"
        out["svd_deg"] = round(svd, 2)
        return out

    def clear(self, x: float, y: float, channel: int) -> dict:
        if not self.entered or self.exited:
            return {"accepted": False, "virtual_time_s": 0}
        dt = self._travel_time(x, y)
        src = self.by_channel.get(channel)
        hit = (
            src is not None
            and channel not in self.cleared
            and math.hypot(x - src.x, y - src.y) <= R_CLEAR
        )
        dt += T_CLEAR_OK if hit else T_CLEAR_FAIL
        self.virtual_time += dt
        self.pos = (x, y)
        self.n_clear += 1
        if hit:
            self.cleared.add(channel)
        else:
            self.n_clear_fail += 1
        if self._limits_exceeded():
            self.exited = True
        return {
            "accepted": True,
            "virtual_time_s": round(self.virtual_time, 6),
            "clear_result": "success" if hit else "no_target_in_range",
        }


class LocalRobot:
    """Robot-side API backed by an in-process MockArena (offline development)."""

    def __init__(self, arena: MockArena):
        self.arena = arena
        self.virtual_time = 0.0
        self.log: list[tuple] = []

    def _sync(self, resp: dict) -> dict:
        if not resp.get("accepted"):
            raise RuntimeError(f"request not accepted: {resp}")
        self.virtual_time = float(resp.get("virtual_time_s") or self.virtual_time)
        return resp

    def enter(self, robot_id: str = "") -> dict:
        return self._sync(self.arena.enter())

    def exit(self) -> dict:
        return self._sync(self.arena.exit())

    def measure(self, x: float, y: float, channel: int) -> dict:
        resp = self._sync(self.arena.measure(x, y, channel))
        self.log.append(("measure", x, y, channel, resp.get("measure_result")))
        return resp

    def clear(self, x: float, y: float, channel: int) -> dict:
        resp = self._sync(self.arena.clear(x, y, channel))
        self.log.append(("clear", x, y, channel, resp.get("clear_result")))
        return resp


def survey_cover_points(ring_radius: float = 1200.0):
    """Centre + 6 ring points; covering radius grows as `ring_radius` shrinks."""
    a = ring_radius
    pts = [(0.0, 0.0)]
    for k in range(6):
        ang = 2 * math.pi * k / 6
        pts.append((a * math.cos(ang), a * math.sin(ang)))
    return pts


def covering_radius_estimate(points, samples: int = 20000, seed: int = 7) -> float:
    """Numeric covering radius of a point set over the arena disc."""
    rng = random.Random(seed)
    worst = 0.0
    for _ in range(samples):
        r = ARENA_RADIUS * math.sqrt(rng.random())
        a = rng.uniform(0.0, 2 * math.pi)
        p = (r * math.cos(a), r * math.sin(a))
        worst = max(worst, min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in points))
    return worst


def triangular_lattice(spacing: float = 900.0, pad: float = 700.0):
    """Triangular lattice over the arena (extended by `pad` metres)."""
    r_max = ARENA_RADIUS + pad
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


def lattice_tour(spacing: float = 900.0, pad: float = 700.0):
    """Lattice points ordered as a short open tour that starts at (0, 0)."""
    pts = triangular_lattice(spacing, pad)
    if (0.0, 0.0) not in pts:
        pts.append((0.0, 0.0))
    remaining = [p for p in pts if p != (0.0, 0.0)]
    cur = (0.0, 0.0)
    order = [(0.0, 0.0)]
    while remaining:
        nxt = min(remaining, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
        order.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    return order


def _tour_length(points, order) -> float:
    return sum(
        math.hypot(
            points[order[i]][0] - points[order[i + 1]][0],
            points[order[i]][1] - points[order[i + 1]][1],
        )
        for i in range(len(order) - 1)
    )


def _two_opt(points, order):
    best = list(order)
    best_len = _tour_length(points, best)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                cand = best[:i] + best[i : j + 1][::-1] + best[j + 1:]
                length = _tour_length(points, cand)
                if length < best_len - 1e-9:
                    best, best_len, improved = cand, length, True
    return best, best_len


def search_pattern_points(
    spacing: float = 900.0,
    ring_radius: float = 2050.0,
    ring_n: int = 12,
    ring_phase: float = 15.0,
):
    """Measurement pattern for problem 4 (directional sources).

    Inner triangular lattice over the arena plus a ring just outside it. The
    ring is what lets the robot hear a source sitting on the boundary and
    radiating outwards; 12 ring points at 2050 m with a 15 deg offset give a
    worst angular gap of 142.5 deg (verified on 12-45 m grids), well inside the
    180 deg required by the surrounding argument.
    """
    pts = triangular_lattice(spacing, pad=0.0)
    for k in range(ring_n):
        ang = math.radians(ring_phase + 360.0 * k / ring_n)
        pts.append((ring_radius * math.cos(ang), ring_radius * math.sin(ang)))
    return pts


def search_pattern_tour(**kwargs):
    """The measurement pattern ordered as a short tour starting at (0, 0)."""
    pts = search_pattern_points(**kwargs)
    remaining = [p for p in pts if p != (0.0, 0.0)]
    cur = (0.0, 0.0)
    order = [(0.0, 0.0)]
    while remaining:
        nxt = min(remaining, key=lambda p: math.hypot(p[0] - cur[0], p[1] - cur[1]))
        order.append(nxt)
        remaining.remove(nxt)
        cur = nxt
    index = {p: i for i, p in enumerate(pts)}
    perm = [index[p] for p in order]
    perm, _ = _two_opt(pts, perm)
    return [pts[i] for i in perm]
