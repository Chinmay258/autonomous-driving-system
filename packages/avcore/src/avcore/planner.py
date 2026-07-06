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
from avcore.models import Lanelet, LaneletId, Point2D, RouteResult, polyline_length
from avcore.routeops import route_eta_s, slice_polyline

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
    """Optimize total route length in meters.

    A lateral (lane-change) edge moves to a parallel lane spanning the same
    stretch of road — the longitudinal distance was already paid by the lane
    being left, so it costs only a small maneuver penalty.
    """

    lane_change_penalty_m: float = 1.0

    def entry_cost(self, lanelet: Lanelet) -> float:
        return lanelet.length_m

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        if kind in _LANE_CHANGE_KINDS:
            return self.lane_change_penalty_m
        return target.length_m

    def heuristic(self, node: Lanelet, goal: Lanelet, max_speed_mps: float) -> float:
        # Remaining geometric path from node's end to goal's end is at least
        # the straight-line distance between them.
        return _endpoint(node).distance_to(_endpoint(goal))


@dataclass(frozen=True, slots=True)
class TravelTime:
    """Optimize expected travel time in seconds, penalizing lane changes.

    Lateral edges cost the maneuver penalty only (see Distance): the parallel
    lane covers the same longitudinal span already paid for.
    """

    lane_change_penalty_s: float = 5.0

    def entry_cost(self, lanelet: Lanelet) -> float:
        return lanelet.length_m / lanelet.speed_limit_mps

    def edge_cost(self, source: Lanelet, target: Lanelet, kind: EdgeKind) -> float:
        if kind in _LANE_CHANGE_KINDS:
            return self.lane_change_penalty_s
        return target.length_m / target.speed_limit_mps

    def heuristic(self, node: Lanelet, goal: Lanelet, max_speed_mps: float) -> float:
        # Straight-line distance at the map-wide top speed can never
        # overestimate the true remaining travel time.
        return _endpoint(node).distance_to(_endpoint(goal)) / max_speed_mps


LANE_CHANGE_MIN_M = 8.0
LANE_CHANGE_MAX_M = 35.0


def _build_result(
    graph: RoutingGraph,
    ids: list[LaneletId],
    kinds: list[EdgeKind],
) -> RouteResult:
    """Stitch the route geometry.

    Successor hops concatenate centerlines. A lateral hop is a *forward
    diagonal*: the previous lane's tail is cut back by the lane-change length
    and the path lands near the end of the neighbouring lane — never
    re-traversing the parallel span (which is what used to draw zigzags).
    """
    lanelets = [graph.lanelet(lid) for lid in ids]
    pieces: list[list[Point2D]] = [list(lanelets[0].centerline)]
    piece_speeds: list[float] = [lanelets[0].speed_limit_mps]

    for lanelet, kind in zip(lanelets[1:], kinds, strict=True):
        if kind is EdgeKind.SUCCESSOR:
            pieces.append(list(lanelet.centerline))
            piece_speeds.append(lanelet.speed_limit_mps)
            continue
        # Lane change: cut the tail of what we drove on the previous lane...
        prev = tuple(pieces[-1])
        prev_len = polyline_length(prev)
        change_len = min(LANE_CHANGE_MAX_M, max(LANE_CHANGE_MIN_M, 0.25 * prev_len))
        if change_len >= prev_len:
            change_len = prev_len / 2.0
        pieces[-1] = list(slice_polyline(prev, 0.0, prev_len - change_len))
        # ...and land near the end of the neighbouring lane.
        neighbour = lanelet.centerline
        neighbour_len = polyline_length(neighbour)
        tail_from = max(0.0, neighbour_len - min(2.0, neighbour_len * 0.1))
        pieces.append(list(slice_polyline(neighbour, tail_from, neighbour_len)))
        piece_speeds.append(lanelet.speed_limit_mps)

    points: list[Point2D] = []
    speeds: list[float] = []
    for piece, speed in zip(pieces, piece_speeds, strict=True):
        for point in piece:
            if not points or points[-1] != point:
                points.append(point)
                speeds.append(speed)

    centerline = tuple(points)
    lane_changes = sum(1 for kind in kinds if kind in _LANE_CHANGE_KINDS)
    return RouteResult(
        lanelet_ids=tuple(ids),
        centerline=centerline,
        distance_m=polyline_length(centerline),
        eta_s=route_eta_s(centerline, tuple(speeds)),
        lane_changes=lane_changes,
        speed_limits_mps=tuple(speeds),
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
