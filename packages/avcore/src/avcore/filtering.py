"""Lanelet and route filtering.

Pre-planning: prune geometrically invalid, non-drivable, or disconnected
lanelets so the planner only ever sees a sane graph. Post-planning: discard
candidate routes violating vehicle constraints and rank the survivors.
"""

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise

from avcore.graph import RoutingGraph
from avcore.models import Lanelet, LaneletId, LaneletSubtype, Point2D, RouteResult

MIN_LANE_WIDTH_M = 2.5
MAX_LANE_WIDTH_M = 6.0
MIN_LANELET_LENGTH_M = 1.0


def _is_sane(lanelet: Lanelet) -> bool:
    # Junction micro-connectors may be arbitrarily short (they exist to give
    # lanelet2 exact shared nodes); street lanes must meet the full minimum.
    min_length = 0.02 if lanelet.is_connector else MIN_LANELET_LENGTH_M
    return (
        lanelet.subtype is LaneletSubtype.ROAD
        and MIN_LANE_WIDTH_M <= lanelet.mean_width_m <= MAX_LANE_WIDTH_M
        and lanelet.length_m >= min_length
    )


def _largest_weak_component(
    nodes: set[LaneletId], neighbours: dict[LaneletId, set[LaneletId]]
) -> set[LaneletId]:
    """Largest weakly-connected component; size ties keep the earliest-seeded one."""
    best: set[LaneletId] = set()
    unvisited = set(nodes)
    while unvisited:
        seed_node = min(unvisited)  # deterministic seeding order
        component = {seed_node}
        frontier = [seed_node]
        while frontier:
            for neighbour in neighbours[frontier.pop()]:
                if neighbour in unvisited and neighbour not in component:
                    component.add(neighbour)
                    frontier.append(neighbour)
        unvisited -= component
        if len(component) > len(best):
            best = component
    return best


def filter_lanelets(graph: RoutingGraph) -> RoutingGraph:
    """Return a new graph restricted to sane, drivable, connected lanelets.

    Keeps only ROAD lanelets passing width/length sanity, then restricts to
    the largest weakly-connected component (weak, not strong: one-way chains
    are legitimate roads). Single lanelets with no edges survive only if they
    are the largest component, i.e. the graph has no edges at all.
    """
    sane = {lid for lid in graph if _is_sane(graph.lanelet(lid))}

    neighbours: dict[LaneletId, set[LaneletId]] = {lid: set() for lid in sane}
    for lid in sane:
        for edge in graph.edges_from(lid):
            if edge.target in sane:
                neighbours[lid].add(edge.target)
                neighbours[edge.target].add(lid)

    keep = _largest_weak_component(sane, neighbours)

    filtered = RoutingGraph()
    for lid in sorted(keep):
        filtered.add_lanelet(graph.lanelet(lid))
    for lid in sorted(keep):
        for edge in graph.edges_from(lid):
            if edge.target in keep:
                filtered.add_edge(lid, edge.target, edge.kind)
    return filtered


def _menger_curvature(a: Point2D, b: Point2D, c: Point2D) -> float:
    """Curvature (1/m) of the circle through three points; 0 for collinear."""
    ab, bc, ca = a.distance_to(b), b.distance_to(c), c.distance_to(a)
    if ab == 0 or bc == 0 or ca == 0:
        return 0.0
    cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    return 2.0 * abs(cross) / (ab * bc * ca)


def max_curvature(centerline: tuple[Point2D, ...]) -> float:
    """Max discrete (Menger) curvature over consecutive centerline triples."""
    if len(centerline) < 3:
        return 0.0
    return max(_menger_curvature(a, b, c) for (a, b), (_, c) in pairwise(pairwise(centerline)))


def rank_routes(
    candidates: Sequence[RouteResult],
    *,
    max_curvature_1pm: float | None = None,
    max_lane_changes: int | None = None,
) -> list[RouteResult]:
    """Drop constraint-violating candidates; return the rest best-first.

    Order: lowest ETA, then shortest, then fewest lane changes, then lanelet
    ids (final tiebreak keeps ranking deterministic).
    """
    survivors = [
        route
        for route in candidates
        if (max_lane_changes is None or route.lane_changes <= max_lane_changes)
        and (max_curvature_1pm is None or max_curvature(route.centerline) <= max_curvature_1pm)
    ]
    return sorted(
        survivors,
        key=lambda r: (r.eta_s, r.distance_m, r.lane_changes, r.lanelet_ids),
    )
