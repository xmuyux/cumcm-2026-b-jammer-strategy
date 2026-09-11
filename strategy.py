"""Search / locate / clear strategy for CUMCM 2026 problem B.

Design in one paragraph
-----------------------
The bearing error is bounded by +-1 deg, so the set of places consistent with
the measurements of one channel is a *convex region* (intersection of wedges,
the arena disc and the <=1500 m detection discs). The true source always lies
inside it. If the minimal enclosing circle of that region has radius <= 20 m,
then standing at its centre guarantees a successful /clear -- that is exactly
the guarantee problem 1 asks for, used as an operational criterion here.

Search is split in three stages:
  1. discovery sweep over a 7 point covering set (covering radius = 900 m, so
     any live omnidirectional source is bound to answer from one of the
     points, because every effective radius is >= 1000 m);
  2. nearest-neighbour tour that localises and clears every detected channel;
  3. verification sweep (same covering set) over the remaining channels. The
     test ends only after a sweep that detects nothing new, which is the
     "ensure every source is cleared" requirement.
"""
from __future__ import annotations

import math

import surround
from arena import (
    ARENA_RADIUS,
    MAX_EFF_RADIUS,
    N_CHANNELS,
    R_CLEAR,
    R_NEAR,
    lattice_tour,
    search_pattern_tour,
    survey_cover_points,
)
from geometry import (
    Circle,
    circumscribed_polygon,
    clip_convex,
    clip_wedge,
    dist,
    minimal_enclosing_circle,
    point_in_convex,
    ray_intersection,
)


class ChannelState:
    __slots__ = (
        "channel",
        "obs",
        "cleared",
        "region",
        "circle",
        "detected_at",
        "misses",
        "measured_at",
    )

    def __init__(self, channel: int):
        self.channel = channel
        self.obs: list[tuple[float, float, float]] = []
        self.cleared = False
        self.region: list[tuple[float, float]] | None = None
        self.circle: Circle | None = None
        self.detected_at = -1  # stage marker
        self.misses = 0  # consecutive no_signal results while localising
        self.measured_at: list[tuple[float, float]] = []

    def add_obs(self, x: float, y: float, svd: float) -> None:
        self.obs.append((x, y, svd))
        self.region = None
        self.circle = None


class TimeBudgetExceeded(RuntimeError):
    """Raised when the real-world budget of the run is nearly exhausted."""


def bearing_spread(obs) -> float:
    """Angular extent covered by a set of bearings (degrees, 0..360)."""
    if len(obs) < 2:
        return 360.0
    angs = sorted(o[2] % 360.0 for o in obs)
    gaps = [(angs[(i + 1) % len(angs)] - angs[i]) % 360.0 for i in range(len(angs))]
    return 360.0 - max(gaps)


class Strategy:
    def __init__(
        self,
        robot,
        half_deg: float = 1.05,
        clear_limit: float = 17.0,
        max_localize_steps: int = 8,
        arena_poly_n: int = 240,
        disc_poly_n: int = 72,
        use_disc_clip_upto: int = 4,
        ring_radius: float = 1200.0,
        mode: str = "omni",
        verify: bool = False,
        real_time_margin_s: float = 25.0,
        opportunistic: bool = False,
        interleave: bool = True,
        detour_limit_m: float = 500.0,
        prune_tight_m: float = 25.0,
        prune_range: bool = True,
        early_stop: bool = True,
        verbose: bool = True,
    ):
        self.robot = robot
        self.half_deg = half_deg
        self.clear_limit = clear_limit
        self.max_localize_steps = max_localize_steps
        self.arena_poly_n = arena_poly_n
        self.disc_poly_n = disc_poly_n
        self.use_disc_clip_upto = use_disc_clip_upto
        self.ring_radius = ring_radius
        self.mode = mode
        self.verify = verify
        self.real_time_margin_s = real_time_margin_s
        self.opportunistic = opportunistic
        self.interleave = interleave
        self.detour_limit_m = detour_limit_m
        self.prune_tight_m = prune_tight_m
        self.prune_range = prune_range
        self.early_stop = early_stop
        self.verbose = verbose
        self.state = {c: ChannelState(c) for c in range(1, N_CHANNELS + 1)}
        self.pos: tuple[float, float] = (0.0, 0.0)
        self.survey_points = (
            search_pattern_tour() if mode == "mixed" else survey_cover_points(ring_radius)
        )
        self.arena_poly = circumscribed_polygon((0.0, 0.0), ARENA_RADIUS * 1.0005, arena_poly_n)
        self.log: list[str] = []
        self.new_detections = 0
        self.common_scan_points: list[tuple[float, float]] = []

    # ---------------------------------------------------------------- utils
    def say(self, msg: str) -> None:
        self.log.append(msg)
        if self.verbose:
            print(msg, flush=True)

    def _say_quiet(self, msg: str) -> None:
        self.log.append(msg)

    @property
    def virtual_time(self) -> float:
        return getattr(self.robot, "virtual_time", 0.0)

    def _move_cost_estimate(self, target) -> float:
        return dist(self.pos, target) / 5.0

    # ------------------------------------------------------------ primitives
    def _check_budget(self) -> None:
        remaining = getattr(self.robot, "remaining_real", None)
        if remaining is not None and remaining <= self.real_time_margin_s:
            raise TimeBudgetExceeded(f"real budget nearly exhausted ({remaining} s left)")

    def measure(self, x: float, y: float, channel: int) -> dict:
        self._check_budget()
        resp = self.robot.measure(x, y, channel)
        self.pos = (x, y)
        result = resp.get("measure_result")
        st = self.state[channel]
        st.measured_at.append((x, y))
        if result == "direction":
            svd = float(resp["svd_deg"])
            if not st.cleared:
                st.add_obs(x, y, svd)
                st.misses = 0
        elif result == "no_signal":
            st.misses += 1
        return resp

    def try_clear(self, x: float, y: float, channel: int) -> dict:
        self._check_budget()
        resp = self.robot.clear(x, y, channel)
        self.pos = (x, y)
        if resp.get("clear_result") == "success":
            self.state[channel].cleared = True
        return resp

    # ------------------------------------------------------------- geometry
    def region(self, channel: int):
        st = self.state[channel]
        if st.region is None:
            poly = list(self.arena_poly)
            for index, (x, y, svd) in enumerate(st.obs):
                poly = clip_wedge(poly, (x, y), svd, self.half_deg)
                if not poly:
                    break
                if index < self.use_disc_clip_upto:
                    disc = circumscribed_polygon((x, y), MAX_EFF_RADIUS * 1.001, self.disc_poly_n)
                    poly = clip_convex(poly, disc)
                    if not poly:
                        break
            st.region = poly
            st.circle = minimal_enclosing_circle(poly) if poly else Circle(0.0, 0.0, float("inf"))
        return st.region, st.circle

    def next_measure_point(self, channel: int):
        st = self.state[channel]
        target = self.estimate_position(channel)
        spread = bearing_spread(st.obs)
        if st.obs and spread < 20.0:
            # beams nearly parallel: no range information yet. Probe sideways
            # instead of walking along the beam.
            px, py, th = st.obs[-1]
            bases = [(px, py, th)]
            for (x, y, a) in reversed(st.obs[-4:]):
                if dist((x, y), self.pos) < dist((bases[0][0], bases[0][1]), self.pos):
                    bases.insert(0, (x, y, a))
            for (bx, by, bth) in bases:
                for t in (150.0, 300.0, 600.0, 900.0):
                    for sign in (1.0, -1.0):
                        rad = math.radians(bth + sign * 90.0)
                        cand = (bx + t * math.cos(rad), by + t * math.sin(rad))
                        if all(dist(cand, (a, b)) > 5.0 for a, b, _ in st.obs):
                            return cand
        if st.misses > 0:
            # likely outside a directional source's beam: come at it from the
            # side where it was already heard, and widen the offset each time
            scale = 100.0 * min(st.misses, 6)
            for (x, y, svd) in reversed(st.obs[-3:]):
                ux, uy = x - target[0], y - target[1]
                norm = math.hypot(ux, uy)
                if norm < 1e-6:
                    continue
                cand = (target[0] + scale * ux / norm, target[1] + scale * uy / norm)
                if all(dist(cand, (a, b)) > 1.0 for a, b, _ in st.obs):
                    return cand
            for ang in (0.0, 90.0, 180.0, 270.0):
                rad = math.radians(ang)
                cand = (
                    target[0] + scale * math.cos(rad),
                    target[1] + scale * math.sin(rad),
                )
                if all(dist(cand, (a, b)) > 1.0 for a, b, _ in st.obs):
                    return cand
        for (x, y, svd) in st.obs:
            if dist((x, y), target) < 1.0:
                _, circle = self.region(channel)
                step = max(20.0, min(0.5 * circle.r, 400.0))
                ang = math.radians(svd + 90.0)
                target = (x + step * math.cos(ang), y + step * math.sin(ang))
                break
        return target

    def estimate_position(self, channel: int):
        """Weighted least-squares crossing of all beams (bearing-only fix).

        Each measurement defines a line through its observation point; the
        estimate minimises the weighted sum of squared perpendicular distances,
        with weight 1/distance so that near observations dominate.
        """
        st = self.state[channel]
        obs = st.obs
        poly, circle = self.region(channel)
        if len(obs) >= 2:
            x = (sum(o[0] for o in obs) / len(obs), sum(o[1] for o in obs) / len(obs))
            for _ in range(4):
                a11 = a12 = a22 = b1 = b2 = 0.0
                for (px, py, th) in obs:
                    rad = math.radians(th)
                    ux, uy = math.cos(rad), math.sin(rad)
                    nx, ny = -uy, ux
                    d = max(dist(x, (px, py)), 1.0)
                    w = 1.0 / d
                    a11 += (w * nx) ** 2
                    a12 += (w * nx) * (w * ny)
                    a22 += (w * ny) ** 2
                    rhs = w * (nx * px + ny * py)
                    b1 += w * nx * rhs
                    b2 += w * ny * rhs
                det = a11 * a22 - a12 * a12
                if abs(det) < 1e-12:
                    break
                x = ((b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det)
            if math.isfinite(x[0]) and math.isfinite(x[1]) and math.hypot(x[0], x[1]) < 1.0e6:
                if not poly or point_in_convex(poly, x):
                    return x
        if len(obs) >= 2:
            points = []
            for i in range(len(obs)):
                for j in range(i + 1, len(obs)):
                    p = ray_intersection(
                        (obs[i][0], obs[i][1]), obs[i][2],
                        (obs[j][0], obs[j][1]), obs[j][2],
                    )
                    if p is None:
                        continue
                    if math.hypot(p[0], p[1]) > ARENA_RADIUS * 1.02:
                        continue
                    if dist(p, (obs[i][0], obs[i][1])) > MAX_EFF_RADIUS:
                        continue
                    if dist(p, (obs[j][0], obs[j][1])) > MAX_EFF_RADIUS:
                        continue
                    points.append(p)
            if points:
                mean = (
                    sum(p[0] for p in points) / len(points),
                    sum(p[1] for p in points) / len(points),
                )
                if not poly or point_in_convex(poly, mean):
                    return mean
        return circle.center

    def pending_channels(self) -> list[int]:
        return [c for c, st in self.state.items() if st.obs and not st.cleared]

    def silent_channels(self) -> list[int]:
        """Channels where nothing has been heard yet: possibly empty, possibly
        a directional source pointing away."""
        return [c for c, st in self.state.items() if not st.cleared and not st.obs]

    def open_channels(self) -> list[int]:
        return [c for c, st in self.state.items() if not st.cleared]

    def surround_satisfied(self) -> bool:
        """True when every silent channel is provably empty.

        Every silent channel has been measured at all points of
        `common_scan_points`; if those points surround every arena location
        (no angular gap >= 180 deg inside the 1000 m detection range), a
        directional source anywhere would have been heard.
        """
        if not self.silent_channels():
            return True
        return surround.surround_ok(self.common_scan_points)

    def opportunistic_scan(self) -> None:
        """Measure the still-silent channels here: extra surround points."""
        if not self.silent_channels():
            return
        if self.early_stop and self.surround_satisfied():
            return
        for channel in self.silent_channels():
            self.measure(self.pos[0], self.pos[1], channel)
        self.common_scan_points.append(self.pos)

    # ---------------------------------------------------------------- stages
    def sweep(self, point, channels) -> int:
        """Measure every requested channel at `point`; returns #new detections."""
        channels = list(channels)
        found = 0
        for ch in channels:
            resp = self.measure(point[0], point[1], ch)
            result = resp.get("measure_result")
            if result == "near":
                # a source is sitting right here: clear it now
                clear = self.try_clear(point[0], point[1], ch)
                if clear.get("clear_result") == "success":
                    found += 1
                    self.say(f"  ch{ch} cleared on the spot  t={self.virtual_time:.0f}s")
            elif result == "direction" and len(self.state[ch].obs) <= 1:
                found += 1
        self.new_detections += found
        return found

    def discovery_sweep(self) -> None:
        for i, pt in enumerate(self.survey_points):
            silent_before = self.silent_channels()
            channels = self.open_channels()
            if self.prune_tight_m > 0:
                channels = [c for c in channels if not self.is_tight(c, self.prune_tight_m)]
            if self.prune_range:
                channels = [c for c in channels if not self.cannot_detect(c, pt)]
            self.sweep(pt, channels)
            if silent_before and all(c in channels for c in silent_before):
                self.common_scan_points.append(pt)
            self.say(
                f"  sweep {i + 1}/{len(self.survey_points)} at ({pt[0]:.0f},{pt[1]:.0f})"
                f"  t={self.virtual_time:.0f}s  detected={len(self.pending_channels())}"
            )
            if self.early_stop and len(self.common_scan_points) >= 4:
                if self.surround_satisfied():
                    self.say(
                        f"  every silent channel is now surrounded"
                        f" ({len(self.common_scan_points)} scan points)"
                        f" -> stop searching  t={self.virtual_time:.0f}s"
                    )
                    return
            if self.interleave and i + 1 < len(self.survey_points):
                self.detour_clear(self.survey_points[i + 1])

    def is_tight(self, channel: int, limit: float) -> bool:
        st = self.state[channel]
        if not st.obs:
            return False
        _, circle = self.region(channel)
        return math.isfinite(circle.r) and circle.r <= limit

    def cannot_detect(self, channel: int, point) -> bool:
        """True when `point` is provably too far to receive this channel.

        The source sits inside the channel's region, so if the whole region is
        farther than the largest possible effective radius, a measurement there
        would return no_signal no matter what.
        """
        st = self.state[channel]
        if not st.obs:
            return False
        _, circle = self.region(channel)
        if not math.isfinite(circle.r):
            return False
        return dist(point, circle.center) - circle.r > MAX_EFF_RADIUS

    def detour_clear(self, next_point) -> None:
        """Clear tightly localised channels that are almost on the way."""
        while True:
            best = None
            for c in self.pending_channels():
                if not self.is_tight(c, self.clear_limit * 1.6):
                    continue
                target = self.region(c)[1].center
                direct = dist(self.pos, next_point) / 5.0
                detour = (dist(self.pos, target) + dist(target, next_point)) / 5.0 - direct
                if detour <= self.detour_limit_m / 5.0 and (best is None or detour < best[0]):
                    best = (detour, c)
            if best is None:
                return
            self.say(
                f"-> detour ch{best[1]}  +{best[0]:.0f}s"
                f"  t={self.virtual_time:.0f}s"
            )
            if not self.localize_and_clear(best[1]):
                self.say(f"  !! detour failed for ch{best[1]}")
                return
            if self.opportunistic:
                self.opportunistic_scan()

    def localize_and_clear(self, channel: int) -> bool:
        st = self.state[channel]
        for step in range(self.max_localize_steps):
            poly, circle = self.region(channel)
            if poly and circle.r <= self.clear_limit:
                target = circle.center
                resp = self.try_clear(target[0], target[1], channel)
                if resp.get("clear_result") == "success":
                    self.say(
                        f"  cleared ch{channel}  t={self.virtual_time:.0f}s"
                        f"  ({len(st.obs)} bearings, region r={circle.r:.1f} m)"
                    )
                    return True
                self.measure(target[0], target[1], channel)
                continue
            target = self.next_measure_point(channel)
            resp = self.measure(target[0], target[1], channel)
            if resp.get("measure_result") == "near":
                resp = self.try_clear(target[0], target[1], channel)
                if resp.get("clear_result") == "success":
                    self.say(f"  cleared ch{channel} (near-field)  t={self.virtual_time:.0f}s")
                    return True
        # last resort: aim at the best estimate and clear
        _, circle = self.region(channel)
        if math.isfinite(circle.r):
            target = self.estimate_position(channel)
            resp = self.try_clear(target[0], target[1], channel)
            if resp.get("clear_result") == "success":
                self.say(f"  cleared ch{channel} (fallback)  t={self.virtual_time:.0f}s")
                return True
        if self.try_clear_pattern(channel):
            return True
        return False

    def try_clear_pattern(self, channel: int) -> bool:
        """Sweep a small clearing pattern around the estimate (cheap insurance)."""
        st = self.state[channel]
        if not st.obs:
            return False
        _, circle = self.region(channel)
        radius = circle.r if math.isfinite(circle.r) else 60.0
        centers = [self.estimate_position(channel)]
        for (x, y, _) in reversed(st.obs[-3:]):
            if all(dist((x, y), c) > 10.0 for c in centers):
                centers.append((x, y))
        ring = max(10.0, min(radius + 3.0, 28.0))
        offsets = [(0.0, 0.0)]
        for k in range(8):
            ang = 2 * math.pi * k / 8
            offsets.append((ring * math.cos(ang), ring * math.sin(ang)))
        if radius > 12.0:
            for k in range(8):
                ang = 2 * math.pi * k / 8 + math.pi / 8
                offsets.append((2 * ring * math.cos(ang), 2 * ring * math.sin(ang)))
        for (cx, cy) in centers:
            for dx, dy in offsets:
                resp = self.try_clear(cx + dx, cy + dy, channel)
                if resp.get("clear_result") == "success":
                    self.say(
                        f"  cleared ch{channel} (pattern, r={radius:.1f} m)"
                        f"  t={self.virtual_time:.0f}s"
                    )
                    return True
        return False

    def clear_pending(self) -> None:
        attempted: set[int] = set()
        while True:
            pending = [
                c
                for c in self.pending_channels()
                if not self.state[c].cleared and c not in attempted
            ]
            if not pending:
                return
            best = min(pending, key=lambda c: self._move_cost_estimate(self.next_measure_point(c)))
            attempted.add(best)
            self.say(
                f"-> target ch{best}  t={self.virtual_time:.0f}s"
                f"  pending={len(pending)}"
            )
            if not self.localize_and_clear(best):
                self.say(f"  !! failed to clear ch{best} (will retry after next sweep)")
            elif self.opportunistic:
                self.opportunistic_scan()

    def run(self) -> dict:
        self.robot.enter()
        self.say("entered arena")
        try:
            self.discovery_sweep()
            self.t_after_discovery = self.virtual_time
            rounds = 0
            while True:
                self.clear_pending()
                stuck = self.pending_channels()
                if not self.verify and not stuck:
                    break
                if stuck:
                    self.say(f"  {len(stuck)} channel(s) still pending; re-sweeping")
                remaining = self.open_channels()
                found = 0
                for pt in self.survey_points:
                    found += self.sweep(pt, remaining)
                self.say(
                    f"  verification sweep: remaining={len(remaining)}"
                    f"  new={found}  t={self.virtual_time:.0f}s"
                )
                if found == 0 and not self.pending_channels():
                    break
                rounds += 1
                if rounds >= 4:
                    self.say("  aborting verification loop after 4 rounds")
                    break
        except TimeBudgetExceeded as exc:
            self.say(f"!! {exc} - exiting early")
        self.robot.exit()
        return {
            "cleared": [c for c, st in self.state.items() if st.cleared],
            "virtual_time": self.virtual_time,
        }
