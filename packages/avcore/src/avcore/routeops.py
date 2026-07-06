"""Polyline arclength operations and route post-processing.

Used by the planner to stitch lane changes as forward diagonals and by
consumers to trim a route to the exact clicked start/goal positions.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import replace
from itertools import pairwise

from avcore.models import Point2D, RouteResult, polyline_length


def cumulative_arclength(points: tuple[Point2D, ...]) -> list[float]:
    out = [0.0]
    for a, b in pairwise(points):
        out.append(out[-1] + a.distance_to(b))
    return out


def point_at_arclength(points: tuple[Point2D, ...], cum: list[float], s: float) -> Point2D:
    s = max(0.0, min(s, cum[-1]))
    for i in range(len(points) - 1):
        if cum[i + 1] >= s:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg == 0 else (s - cum[i]) / seg
            a, b = points[i], points[i + 1]
            return Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
    return points[-1]


def slice_polyline(points: tuple[Point2D, ...], s_from: float, s_to: float) -> tuple[Point2D, ...]:
    """Sub-polyline between two arclengths, with interpolated exact endpoints."""
    cum = cumulative_arclength(points)
    s_from = max(0.0, min(s_from, cum[-1]))
    s_to = max(s_from, min(s_to, cum[-1]))
    first = point_at_arclength(points, cum, s_from)
    last = point_at_arclength(points, cum, s_to)
    middle = [p for p, s in zip(points, cum, strict=True) if s_from < s < s_to]
    out: list[Point2D] = [first]
    for p in [*middle, last]:
        if p != out[-1]:
            out.append(p)
    if len(out) == 1:  # degenerate zero-length slice
        out.append(last)
    return tuple(out)


def nearest_arclength(
    points: tuple[Point2D, ...],
    target: Point2D,
    *,
    prefer: str = "first",
    tolerance_m: float = 2.0,
) -> float:
    """Arclength of the point on the polyline nearest to ``target``.

    Routes may pass near a location twice (e.g. before and after a U-turn):
    among near-optimal candidates within ``tolerance_m`` of the best distance,
    ``prefer='first'`` returns the earliest arclength and ``prefer='last'``
    the latest — start clicks want the first pass, goal clicks the last.
    """
    cum = cumulative_arclength(points)
    candidates: list[tuple[float, float]] = []  # (distance, arclength)
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        abx, aby = b.x - a.x, b.y - a.y
        seg_sq = abx * abx + aby * aby
        if seg_sq == 0:
            continue
        t = max(0.0, min(1.0, ((target.x - a.x) * abx + (target.y - a.y) * aby) / seg_sq))
        proj = Point2D(a.x + t * abx, a.y + t * aby)
        candidates.append((proj.distance_to(target), cum[i] + t * (seg_sq**0.5)))
    if not candidates:
        return 0.0
    best = min(d for d, _ in candidates)
    near = [s for d, s in candidates if d <= best + tolerance_m]
    return min(near) if prefer == "first" else max(near)


def route_eta_s(points: tuple[Point2D, ...], speeds: tuple[float, ...]) -> float:
    """Kinematic ETA: per-segment length over the segment's source-point limit."""
    return sum(a.distance_to(b) / max(speeds[i], 0.1) for i, (a, b) in enumerate(pairwise(points)))


def trim_route(route: RouteResult, start: Point2D, goal: Point2D) -> RouteResult:
    """Cut the route centerline to the projections of the clicked endpoints.

    The lanelet id sequence is preserved; geometry, speeds, distance and ETA
    reflect the trimmed path the vehicle will actually drive.
    """
    points = route.centerline
    speeds = route.speed_limits_mps or tuple(8.3 for _ in points)
    s_start = nearest_arclength(points, start, prefer="first")
    s_goal = nearest_arclength(points, goal, prefer="last")
    total = cumulative_arclength(points)[-1]
    if s_goal <= s_start + 2.0:  # degenerate click pair; keep a minimal forward hop
        s_goal = min(total, s_start + 2.0)
        if s_goal <= s_start:
            return route

    cum = cumulative_arclength(points)
    sliced = slice_polyline(points, s_start, s_goal)
    # Speed for each sliced point: limit of the source segment it lies on.
    trimmed_speeds: list[float] = []
    run = s_start
    for j, p in enumerate(sliced):
        if j > 0:
            run += sliced[j - 1].distance_to(p)
        idx = max(0, bisect_right(cum, run) - 1)
        trimmed_speeds.append(speeds[min(idx, len(speeds) - 1)])
    pts = tuple(sliced)
    spd = tuple(trimmed_speeds)
    return replace(
        route,
        centerline=pts,
        speed_limits_mps=spd,
        distance_m=polyline_length(pts),
        eta_s=route_eta_s(pts, spd),
    )
