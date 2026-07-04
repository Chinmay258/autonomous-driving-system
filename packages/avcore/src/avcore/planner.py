"""Route planning over the lanelet routing graph.

Contract frozen in ARCHITECTURE.md §5.3. A* implementation lands in P1; the
input validation and cost-model surface are fixed now so dependents can build
against them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from avcore.graph import EdgeKind, RoutingGraph
from avcore.models import Lanelet, LaneletId, RouteResult


class CostModel(Protocol):
    """Edge cost in the planner's optimization units (seconds or meters)."""

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float: ...


@dataclass(frozen=True, slots=True)
class Distance:
    """Cost = centerline length of the target lanelet, in meters."""

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        raise NotImplementedError("P1")


@dataclass(frozen=True, slots=True)
class TravelTime:
    """Cost = target length / speed limit, plus a penalty per lane change."""

    lane_change_penalty_s: float = 5.0

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        raise NotImplementedError("P1")


def plan_route(
    graph: RoutingGraph,
    start: LaneletId,
    goal: LaneletId,
    cost: CostModel | None = None,
    *,
    seed: int | None = None,
) -> RouteResult:
    """A* shortest route from start lanelet to goal lanelet.

    Deterministic given identical inputs. Raises UnknownLaneletError for ids
    absent from the graph and UnreachableGoalError when no path exists.
    """
    # Input validation is part of the frozen contract and active now.
    graph.lanelet(start)
    graph.lanelet(goal)
    raise NotImplementedError("A* lands in P1")
