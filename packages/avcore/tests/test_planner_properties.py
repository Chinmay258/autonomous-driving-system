"""Property-based checks: A* is optimal (matches reference Dijkstra), routes
are graph-valid, and planning is deterministic — over randomized speed grids."""

from __future__ import annotations

import heapq
from itertools import pairwise

from hypothesis import given, settings
from hypothesis import strategies as st

from avcore import CostModel, Distance, LaneletId, RoutingGraph, TravelTime, plan_route
from avcore.models import RouteResult
from tests.mapbuilders import grid_graph

speed_grids = st.lists(
    st.lists(st.floats(min_value=1.0, max_value=30.0), min_size=1, max_size=5),
    min_size=1,
    max_size=4,
).map(lambda rows: [row[: min(len(r) for r in rows)] for row in rows])  # rectangular


def dijkstra_cost(
    graph: RoutingGraph, start: LaneletId, goal: LaneletId, model: CostModel
) -> float:
    """Reference uniform-cost search; returns optimal route cost."""
    dist: dict[LaneletId, float] = {start: model.entry_cost(graph.lanelet(start))}
    heap: list[tuple[float, LaneletId]] = [(dist[start], start)]
    while heap:
        d, node = heapq.heappop(heap)
        if d > dist.get(node, float("inf")):
            continue
        if node == goal:
            return d
        for edge in graph.edges_from(node):
            nd = d + model.edge_cost(graph.lanelet(node), graph.lanelet(edge.target), edge.kind)
            if nd < dist.get(edge.target, float("inf")):
                dist[edge.target] = nd
                heapq.heappush(heap, (nd, edge.target))
    raise AssertionError("goal unreachable in fixture grid")


def route_cost(graph: RoutingGraph, result: RouteResult, model: CostModel) -> float:
    """Re-derive the model cost of a returned route from the graph's edges."""
    total = model.entry_cost(graph.lanelet(result.lanelet_ids[0]))
    for a, b in pairwise(result.lanelet_ids):
        edges = [e for e in graph.edges_from(a) if e.target == b]
        assert edges, f"route uses nonexistent edge {a}->{b}"
        total += min(model.edge_cost(graph.lanelet(a), graph.lanelet(b), e.kind) for e in edges)
    return total


@settings(max_examples=60, deadline=None)
@given(speeds=speed_grids)
def test_astar_matches_dijkstra_travel_time(speeds: list[list[float]]) -> None:
    graph = grid_graph(speeds)
    start, goal = LaneletId(1), LaneletId(len(speeds) * len(speeds[0]))
    model = TravelTime()
    result = plan_route(graph, start, goal, model)
    assert route_cost(graph, result, model) <= dijkstra_cost(graph, start, goal, model) + 1e-9


@settings(max_examples=60, deadline=None)
@given(speeds=speed_grids)
def test_astar_matches_dijkstra_distance(speeds: list[list[float]]) -> None:
    graph = grid_graph(speeds)
    start, goal = LaneletId(1), LaneletId(len(speeds) * len(speeds[0]))
    model = Distance()
    result = plan_route(graph, start, goal, model)
    assert route_cost(graph, result, model) <= dijkstra_cost(graph, start, goal, model) + 1e-9


@settings(max_examples=40, deadline=None)
@given(speeds=speed_grids)
def test_route_is_valid_and_deterministic(speeds: list[list[float]]) -> None:
    graph = grid_graph(speeds)
    start, goal = LaneletId(1), LaneletId(len(speeds) * len(speeds[0]))
    first = plan_route(graph, start, goal)
    second = plan_route(graph, start, goal)
    assert first == second
    assert first.lanelet_ids[0] == start
    assert first.lanelet_ids[-1] == goal
    assert first.distance_m > 0
    assert first.eta_s > 0
