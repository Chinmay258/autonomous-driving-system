"""Typed domain models. Frozen dataclasses; stdlib only."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
from typing import NewType

LaneletId = NewType("LaneletId", int)


@dataclass(frozen=True, slots=True)
class Point2D:
    """A point in the local metric frame (meters, ENU-style)."""

    x: float
    y: float

    def distance_to(self, other: Point2D) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True, slots=True)
class LatLng:
    """WGS84 coordinate. Validation of bounds happens at the web edge (schemas)."""

    lat: float
    lng: float


class LaneletSubtype(StrEnum):
    ROAD = "road"
    CROSSWALK = "crosswalk"
    SHOULDER = "shoulder"


@dataclass(frozen=True, slots=True)
class Lanelet:
    """One drivable lane segment bounded by a left and right polyline.

    Mirrors the Lanelet2 primitive; loaded from the canonical .osm map artifact.
    """

    id: LaneletId
    left_bound: tuple[Point2D, ...]
    right_bound: tuple[Point2D, ...]
    speed_limit_mps: float
    one_way: bool = True
    subtype: LaneletSubtype = LaneletSubtype.ROAD
    is_connector: bool = False  # junction turn/U-turn connector (rendering hint)

    def __post_init__(self) -> None:
        if len(self.left_bound) < 2 or len(self.right_bound) < 2:
            raise ValueError(f"lanelet {self.id}: bounds need >= 2 points each")
        if self.speed_limit_mps <= 0:
            raise ValueError(f"lanelet {self.id}: speed limit must be positive")

    @property
    def centerline(self) -> tuple[Point2D, ...]:
        """Midpoints of paired bound vertices.

        Naive pairing (zip stops at the shorter bound); P1 replaces this with
        arc-length resampling when maps with asymmetric bounds arrive.
        """
        return tuple(
            Point2D((lp.x + rp.x) / 2.0, (lp.y + rp.y) / 2.0)
            for lp, rp in zip(self.left_bound, self.right_bound, strict=False)
        )

    @property
    def length_m(self) -> float:
        return polyline_length(self.centerline)

    @property
    def mean_width_m(self) -> float:
        """Mean distance between paired bound vertices (naive pairing, cf. centerline)."""
        pairs = list(zip(self.left_bound, self.right_bound, strict=False))
        return sum(lp.distance_to(rp) for lp, rp in pairs) / len(pairs)


def polyline_length(points: tuple[Point2D, ...]) -> float:
    """Sum of segment lengths; 0.0 for fewer than 2 points."""
    return sum(a.distance_to(b) for a, b in pairwise(points))


@dataclass(frozen=True, slots=True)
class RouteResult:
    """Planner output: ordered lanelets + stitched centerline + summary stats.

    Same shape Autoware's mission planner produces, and exactly what the web
    demo's controller consumes (ADR-0002: one brain, two bodies).
    """

    lanelet_ids: tuple[LaneletId, ...]
    centerline: tuple[Point2D, ...]
    distance_m: float
    eta_s: float
    lane_changes: int
    # Per-centerline-point speed limits (parallel to `centerline`; may be
    # empty for hand-built results in tests).
    speed_limits_mps: tuple[float, ...] = ()
