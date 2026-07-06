"""Detected lane boundaries -> lanelets (the original project's conversion).

Input: ordered left-to-right boundary polylines in the ego metric frame
(e.g. avlane's ``left_boundary_m``/``right_boundary_m``, as (lateral, forward)
pairs). Adjacent boundaries pair into lanelets sharing the divider polyline,
so the derived graph gets lane-change edges exactly like the OSM import.
"""

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise

from avcore import Lanelet, LaneletId, Point2D


def _resample(points: Sequence[tuple[float, float]], n: int) -> tuple[Point2D, ...]:
    pts = [Point2D(x, y) for x, y in points]
    if len(pts) < 2:
        raise ValueError("boundary needs at least 2 points")
    cum = [0.0]
    for a, b in pairwise(pts):
        cum.append(cum[-1] + a.distance_to(b))
    total = cum[-1]
    out: list[Point2D] = []
    for i in range(n):
        s = total * i / (n - 1)
        j = max(
            0,
            min(
                len(pts) - 2,
                next(k for k in range(len(cum)) if cum[k] >= s or k == len(cum) - 1) - 1,
            ),
        )
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg == 0 else (s - cum[j]) / seg
        a, b = pts[j], pts[j + 1]
        out.append(Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)))
    return tuple(out)


def polylines_to_lanelets(
    boundaries: Sequence[Sequence[tuple[float, float]]],
    *,
    speed_limit_mps: float = 13.9,
    samples: int = 12,
    first_id: int = 1,
) -> list[Lanelet]:
    """Pair adjacent boundary polylines (left->right order) into lanelets."""
    if len(boundaries) < 2:
        raise ValueError("need at least two boundaries to form a lane")
    resampled = [_resample(b, samples) for b in boundaries]
    lanelets = []
    for k, (left, right) in enumerate(pairwise(resampled)):
        lanelets.append(
            Lanelet(
                id=LaneletId(first_id + k),
                left_bound=left,
                right_bound=right,
                speed_limit_mps=speed_limit_mps,
            )
        )
    return lanelets
