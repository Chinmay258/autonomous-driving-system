"""Derive the routing graph from raw lanelet geometry.

Successor edges: lanelet B follows A when A's centerline end coincides with
B's centerline start (within tolerance). Lateral edges: A and B are adjacent
same-direction lanes when they share a divider polyline (A's left bound is
B's right bound), which the deduplicating writer preserves and real Lanelet2
maps express the same way.
"""

from __future__ import annotations

from collections.abc import Sequence

from avcore import EdgeKind, Lanelet, Point2D, RoutingGraph

ENDPOINT_TOLERANCE_M = 0.5


def _close(a: Point2D, b: Point2D, tol: float) -> bool:
    return a.distance_to(b) <= tol


def build_graph(
    lanelets: Sequence[Lanelet], *, endpoint_tol: float = ENDPOINT_TOLERANCE_M
) -> RoutingGraph:
    graph = RoutingGraph()
    for lanelet in lanelets:
        graph.add_lanelet(lanelet)

    # Bucket centerline starts on a coarse grid so successor matching stays ~O(n).
    cell = max(endpoint_tol, 0.1)
    starts: dict[tuple[int, int], list[Lanelet]] = {}
    for lanelet in lanelets:
        p = lanelet.centerline[0]
        key = (int(p.x // cell), int(p.y // cell))
        starts.setdefault(key, []).append(lanelet)

    for lanelet in lanelets:
        end = lanelet.centerline[-1]
        kx, ky = int(end.x // cell), int(end.y // cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for candidate in starts.get((kx + dx, ky + dy), []):
                    if candidate.id != lanelet.id and _close(
                        end, candidate.centerline[0], endpoint_tol
                    ):
                        graph.add_edge(lanelet.id, candidate.id, EdgeKind.SUCCESSOR)

    # Shared-divider lateral adjacency (same-direction lanes only).
    by_left: dict[tuple[Point2D, ...], list[Lanelet]] = {}
    for lanelet in lanelets:
        by_left.setdefault(lanelet.left_bound, []).append(lanelet)
    for lanelet in lanelets:
        for neighbour in by_left.get(lanelet.right_bound, []):
            if neighbour.id != lanelet.id:
                # neighbour's left bound == lanelet's right bound:
                # lanelet sits to the LEFT of neighbour.
                graph.add_edge(lanelet.id, neighbour.id, EdgeKind.RIGHT)
                graph.add_edge(neighbour.id, lanelet.id, EdgeKind.LEFT)
    return graph
