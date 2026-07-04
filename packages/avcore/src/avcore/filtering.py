"""Lanelet and route filtering (P1/P2 implementation; signatures frozen now).

Pre-planning: prune geometrically invalid, disconnected, or non-drivable
lanelets. Post-planning: discard candidate routes violating vehicle constraints
and rank the survivors.
"""

from __future__ import annotations

from collections.abc import Sequence

from avcore.graph import RoutingGraph
from avcore.models import RouteResult

MIN_LANE_WIDTH_M = 2.5
MAX_LANE_WIDTH_M = 6.0
MIN_LANELET_LENGTH_M = 1.0


def filter_lanelets(graph: RoutingGraph) -> RoutingGraph:
    """Return a new graph restricted to sane, drivable, connected lanelets."""
    raise NotImplementedError("P1")


def rank_routes(
    candidates: Sequence[RouteResult],
    *,
    max_curvature: float | None = None,
    max_lane_changes: int | None = None,
) -> list[RouteResult]:
    """Drop constraint-violating candidates; return the rest best-first."""
    raise NotImplementedError("P1")
