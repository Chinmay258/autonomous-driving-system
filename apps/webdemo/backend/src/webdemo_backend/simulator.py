"""Kinematic bicycle vehicle following a route with pure-pursuit steering.

Local metric frame throughout; the API layer converts to WGS84. Deterministic:
fixed dt, no randomness. Safety envelope: the drive aborts on excessive
cross-track error or wall-clock-independent sim-time budget.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise

from avcore import Point2D
from avcore.filtering import max_curvature as path_max_curvature
from avcore.models import polyline_length

WHEELBASE_M = 2.7
MAX_STEER_RAD = 0.6
MAX_ACCEL_MPS2 = 1.5
MAX_DECEL_MPS2 = 3.0
COMFORT_DECEL_MPS2 = 2.0
MAX_LATERAL_ACCEL_MPS2 = 2.5
LOOKAHEAD_MIN_M = 6.0
LOOKAHEAD_MAX_M = 15.0
LOOKAHEAD_GAIN_S = 1.2
SPEED_WINDOW_M = 30.0
ARRIVAL_TOLERANCE_M = 1.5
ABORT_CTE_M = 10.0
ABORT_SIM_TIME_S = 900.0


class SimState(StrEnum):
    DRIVING = "driving"
    ARRIVED = "arrived"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class SimFrame:
    t: float
    x: float
    y: float
    heading_rad: float
    speed_mps: float
    cte_m: float
    progress: float
    state: SimState
    reason: str | None = None


class VehicleSimulator:
    """Follows a polyline route. Call step(dt) until a terminal frame."""

    def __init__(self, path: tuple[Point2D, ...], speed_limits_mps: tuple[float, ...]) -> None:
        if len(path) < 2 or len(path) != len(speed_limits_mps):
            raise ValueError("path needs >= 2 points and matching speed limits")
        self._path = path
        self._limits = speed_limits_mps
        # Cumulative arclength per path point.
        self._s = [0.0]
        for a, b in pairwise(path):
            self._s.append(self._s[-1] + a.distance_to(b))
        self._total = self._s[-1]
        # Local curvature per point (for curve speed limits).
        self._kappa = [0.0] * len(path)
        for i in range(1, len(path) - 1):
            self._kappa[i] = path_max_curvature((path[i - 1], path[i], path[i + 1]))

        first, second = path[0], path[1]
        self._x, self._y = first.x, first.y
        self._heading = math.atan2(second.y - first.y, second.x - first.x)
        self._v = 0.0
        self._t = 0.0
        self._seg = 0  # monotonic progress index
        self._last: SimFrame = self._frame(SimState.DRIVING, cte=0.0, s_now=0.0)

    @property
    def total_length_m(self) -> float:
        return self._total

    @property
    def last_frame(self) -> SimFrame:
        return self._last

    def _frame(
        self, state: SimState, *, cte: float, s_now: float, reason: str | None = None
    ) -> SimFrame:
        return SimFrame(
            t=self._t,
            x=self._x,
            y=self._y,
            heading_rad=self._heading,
            speed_mps=self._v,
            cte_m=cte,
            progress=min(1.0, s_now / self._total) if self._total > 0 else 1.0,
            state=state,
            reason=reason,
        )

    def _project(self) -> tuple[float, float]:
        """(arclength, cross-track error) of the pose, searching forward only."""
        best_s, best_d = self._s[self._seg], float("inf")
        best_seg = self._seg
        end = min(len(self._path) - 1, self._seg + 60)
        for i in range(self._seg, end):
            a, b = self._path[i], self._path[i + 1]
            abx, aby = b.x - a.x, b.y - a.y
            seg_len_sq = abx * abx + aby * aby
            if seg_len_sq == 0:
                continue
            t = ((self._x - a.x) * abx + (self._y - a.y) * aby) / seg_len_sq
            t = max(0.0, min(1.0, t))
            px, py = a.x + t * abx, a.y + t * aby
            d = math.hypot(self._x - px, self._y - py)
            if d < best_d:
                best_d = d
                best_s = self._s[i] + t * math.sqrt(seg_len_sq)
                best_seg = i
        self._seg = best_seg
        return best_s, best_d

    def _point_at(self, s: float) -> Point2D:
        s = max(0.0, min(s, self._total))
        # Find segment containing s (linear from current segment; paths are short).
        i = self._seg
        while i < len(self._s) - 2 and self._s[i + 1] < s:
            i += 1
        seg_len = self._s[i + 1] - self._s[i]
        t = 0.0 if seg_len == 0 else (s - self._s[i]) / seg_len
        a, b = self._path[i], self._path[i + 1]
        return Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))

    def _target_speed(self, s_now: float) -> float:
        # Lowest speed limit and tightest curve within the lookahead window.
        limit = float("inf")
        i = self._seg
        while i < len(self._path) and self._s[i] <= s_now + SPEED_WINDOW_M:
            limit = min(limit, self._limits[i])
            if self._kappa[i] > 1e-6:
                limit = min(limit, math.sqrt(MAX_LATERAL_ACCEL_MPS2 / self._kappa[i]))
            i += 1
        # Smooth stop into the goal.
        remaining = max(0.0, self._total - s_now)
        limit = min(limit, math.sqrt(2.0 * COMFORT_DECEL_MPS2 * remaining))
        return limit

    def step(self, dt: float = 0.1) -> SimFrame:
        if self._last.state is not SimState.DRIVING:
            return self._last

        s_now, cte = self._project()

        if self._total - s_now <= ARRIVAL_TOLERANCE_M:
            self._v = 0.0
            self._last = self._frame(SimState.ARRIVED, cte=cte, s_now=self._total)
            return self._last
        if cte > ABORT_CTE_M:
            self._last = self._frame(
                SimState.ABORTED, cte=cte, s_now=s_now, reason="cross-track error exceeded"
            )
            return self._last
        if self._t > ABORT_SIM_TIME_S:
            self._last = self._frame(
                SimState.ABORTED, cte=cte, s_now=s_now, reason="sim time budget exceeded"
            )
            return self._last

        # Pure pursuit steering toward a speed-scaled lookahead point.
        lookahead = max(LOOKAHEAD_MIN_M, min(LOOKAHEAD_MAX_M, LOOKAHEAD_GAIN_S * self._v))
        target = self._point_at(s_now + lookahead)
        alpha = math.atan2(target.y - self._y, target.x - self._x) - self._heading
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap to [-pi, pi]
        steer = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), lookahead)
        steer = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steer))

        # Longitudinal control toward the window target speed.
        v_target = self._target_speed(s_now)
        accel = (v_target - self._v) / dt
        accel = max(-MAX_DECEL_MPS2, min(MAX_ACCEL_MPS2, accel))
        self._v = max(0.0, self._v + accel * dt)

        # Kinematic bicycle update.
        self._x += self._v * math.cos(self._heading) * dt
        self._y += self._v * math.sin(self._heading) * dt
        self._heading += (self._v / WHEELBASE_M) * math.tan(steer) * dt
        self._t += dt

        self._last = self._frame(SimState.DRIVING, cte=cte, s_now=s_now)
        return self._last


def route_length(path: tuple[Point2D, ...]) -> float:
    return polyline_length(path)
