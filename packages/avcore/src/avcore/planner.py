"""Route planning over the lanelet routing graph.

A* with pluggable cost models. Deterministic: ties are broken by (g, lanelet
id), never by insertion order, so identical inputs always yield identical
routes. Contract frozen in ARCHITECTURE.md §5.3.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Protocol

from avcore.errors import UnreachableGoalError
from avcore.graph import EdgeKind, RoutingGraph
from avcore.models import Lanelet, LaneletId, Point2D, RouteResult

_LANE_CHANGE_KINDS = frozenset({EdgeKind.LEFT, EdgeKind.RIGHT})


class CostModel(Protocol):
    """Cost surface for the planner. Units are meters or seconds.

    Implementations must keep ``heuristic`` admissible (never overestimate the
    remaining cost) or A* loses optimality.
    """

    def entry_cost(self, lanelet: Lanelet) -> float:
        """Cost of traversing the first lanelet of a route."""
        ...

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        """Cost of taking an edge, including traversal of ``target``."""
        ...

    def heuristic(self, node: Lanelet, goal: Lanelet, max_speed_mps: float) -> float:
        """Admissible estimate of remaining cost from ``node`` to ``goal``."""
        ...


def _endpoint(lanelet: Lanelet) -> Point2D:
    return lanelet.centerline[-1]


@dataclass(frozen=True, slots=True)
class Distance:
    """Optimize total route length in meters. Lane changes cost no extra length."""

    def entry_cost(self, lanelet: Lanelet) -> float:
        return lanelet.length_m

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        return target.length_m

    def heuristic(self, node: Lanelet, goal: Lanelet, max_speed_mps: float) -> float:
        # Remaining geometric path from node's end to goal's end is at least
        # the straight-line distance between them.
        return _endpoint(node).distance_to(_endpoint(goal))


@dataclass(frozen=True, slots=True)
class TravelTime:
    """Optimize expected travel time in seconds, penalizing lane changes."""

    lane_change_penalty_s: float = 5.0

    def entry_cost(self, lanelet: Lanelet) -> float:
        return lanelet.length_m / lanelet.speed_limit_mps

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        traversal = target.length_m / target.speed_limit_mps
        if kind in _LANE_CHANGE_KINDS:
            traversal += self.lane_change_penalty_s
        return traversal

    def heuristic(self, node: Lanelet, goal: Lanelet, max_speed_mps: float) -> float:
        # Straight-line distance at the map-wide top speed can never
        # overestimate the true remaining travel time.
        return _endpoint(node).distance_to(_endpoint(goal)) / max_speed_mps


def _stitch_centerline(lanelets: list[Lanelet]) -> tuple[Point2D, ...]:
    """Concatenate centerlines, dropping consecutive duplicate joint points."""
    points: list[Point2D] = []
    for lanelet in lanelets:
        for point in lanelet.centerline:
            if not points or points[-1] != point:
                points.append(point)
    return tuple(points)


def _build_result(
    graph: RoutingGraph,
    ids: list[LaneletId],
    kinds: list[EdgeKind],
) -> RouteResult:
    lanelets = [graph.lanelet(lid) for lid in ids]
    distance = sum(lanelet.length_m for lanelet in lanelets)
    eta = sum(lanelet.length_m / lanelet.speed_limit_mps for lanelet in lanelets)
    lane_changes = sum(1 for kind in kinds if kind in _LANE_CHANGE_KINDS)
    return RouteResult(
        lanelet_ids=tuple(ids),
        centerline=_stitch_centerline(lanelets),
        distance_m=distance,
        eta_s=eta,
        lane_changes=lane_changes,
    )


def plan_route(
    graph: RoutingGraph,
    start: LaneletId,
    goal: LaneletId,
    cost: CostModel | None = None,
    *,
    seed: int | None = None,
) -> RouteResult:
    """A* shortest route from start lanelet to goal lanelet.

    Deterministic given identical inputs (``seed`` is reserved; current
    planner is fully deterministic without it). Raises UnknownLaneletError for
    ids absent from the graph and UnreachableGoalError when no path exists.
    """
    start_lanelet = graph.lanelet(start)
    goal_lanelet = graph.lanelet(goal)
    model: CostModel = cost if cost is not None else TravelTime()

    max_speed = max(graph.lanelet(lid).speed_limit_mps for lid in graph)

    # g-scores and parent pointers keyed by lanelet id.
    g_score: dict[LaneletId, float] = {start: model.entry_cost(start_lanelet)}
    parent: dict[LaneletId, tuple[LaneletId, EdgeKind]] = {}
    # Heap entries: (f, g, id). Lexicographic order makes expansion fully
    # deterministic; stale entries are skipped via the g check.
    open_heap: list[tuple[float, float, LaneletId]] = [
        (
            g_score[start] + model.heuristic(start_lanelet, goal_lanelet, max_speed),
            g_score[start],
            start,
        )
    ]
    closed: set[LaneletId] = set()

    while open_heap:
        _, g_current, current = heapq.heappop(open_heap)
        if current in closed or g_current > g_score.get(current, float("inf")):
            continue
        if current == goal:
            ids: list[LaneletId] = [current]
            kinds: list[EdgeKind] = []
            while ids[-1] in parent:
                prev_id, kind = parent[ids[-1]]
                ids.append(prev_id)
                kinds.append(kind)
            ids.reverse()
            kinds.reverse()
            return _build_result(graph, ids, kinds)
        closed.add(current)
        current_lanelet = graph.lanelet(current)

        for edge in graph.edges_from(current):
            if edge.target in closed:
                continue
            tentative = g_current + model.edge_cost(
                current_lanelet, graph.lanelet(edge.target), edge.kind
            )
            if tentative < g_score.get(edge.target, float("inf")):
                g_score[edge.target] = tentative
                parent[edge.target] = (current, edge.kind)
                target_lanelet = graph.lanelet(edge.target)
                f = tentative + model.heuristic(target_lanelet, goal_lanelet, max_speed)
                heapq.heappush(open_heap, (f, tentative, edge.target))

    raise UnreachableGoalError(f"no route from lanelet {start} to lanelet {goal}")
