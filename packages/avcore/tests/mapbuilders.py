"""Deterministic fixture maps for planner and filtering tests.

Geometry convention: axis-aligned corridors along +x, lane changes offset in y.
Successive lanelets connect end-to-start so heuristics stay admissible.
"""

from __future__ import annotations

from avcore import EdgeKind, Lanelet, LaneletId, LaneletSubtype, Point2D, RoutingGraph


def box_lanelet(
    lid: int,
    x0: float,
    y0: float,
    length: float,
    *,
    width: float = 3.5,
    speed: float = 13.9,
    subtype: LaneletSubtype = LaneletSubtype.ROAD,
) -> Lanelet:
    """Straight lanelet from (x0, y0) to (x0+length, y0)."""
    half = width / 2.0
    return Lanelet(
        id=LaneletId(lid),
        left_bound=(Point2D(x0, y0 + half), Point2D(x0 + length, y0 + half)),
        right_bound=(Point2D(x0, y0 - half), Point2D(x0 + length, y0 - half)),
        speed_limit_mps=speed,
        subtype=subtype,
    )


def dogleg_lanelet(
    lid: int, x0: float, y0: float, dx: float, peak: float, *, width: float = 3.5, speed: float
) -> Lanelet:
    """Bent lanelet (0,0)->(dx/2,peak)->(dx,0): same endpoints as a box, longer path."""
    half = width / 2.0
    mid = x0 + dx / 2.0
    return Lanelet(
        id=LaneletId(lid),
        left_bound=(
            Point2D(x0, y0 + half),
            Point2D(mid, y0 + peak + half),
            Point2D(x0 + dx, y0 + half),
        ),
        right_bound=(
            Point2D(x0, y0 - half),
            Point2D(mid, y0 + peak - half),
            Point2D(x0 + dx, y0 - half),
        ),
        speed_limit_mps=speed,
    )


def chain_graph(n: int = 3, *, length: float = 10.0, speed: float = 13.9) -> RoutingGraph:
    """Straight chain 1 -> 2 -> ... -> n."""
    g = RoutingGraph()
    for i in range(n):
        g.add_lanelet(box_lanelet(i + 1, i * length, 0.0, length, speed=speed))
    for i in range(1, n):
        g.add_edge(LaneletId(i), LaneletId(i + 1), EdgeKind.SUCCESSOR)
    return g


def fork_graph() -> RoutingGraph:
    """Start 1, then short-slow lanelet 2 vs long-fast dogleg 3, rejoining at goal 4.

    2: 100 m @ 5 m/s  = 20.0 s      (shortest)
    3: ~141 m @ 25 m/s = ~5.66 s    (fastest)
    """
    g = RoutingGraph()
    g.add_lanelet(box_lanelet(1, 0.0, 0.0, 10.0))
    g.add_lanelet(box_lanelet(2, 10.0, 0.0, 100.0, speed=5.0))
    g.add_lanelet(dogleg_lanelet(3, 10.0, 0.0, 100.0, 50.0, speed=25.0))
    g.add_lanelet(box_lanelet(4, 110.0, 0.0, 10.0))
    g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(1), LaneletId(3), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(2), LaneletId(4), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(3), LaneletId(4), EdgeKind.SUCCESSOR)
    return g


def lane_change_graph() -> RoutingGraph:
    """Two parallel lanes, two segments each. A: ids 1,2 (y=0). B: ids 3,4 (y=3.5)."""
    g = RoutingGraph()
    g.add_lanelet(box_lanelet(1, 0.0, 0.0, 50.0))
    g.add_lanelet(box_lanelet(2, 50.0, 0.0, 50.0))
    g.add_lanelet(box_lanelet(3, 0.0, 3.5, 50.0))
    g.add_lanelet(box_lanelet(4, 50.0, 3.5, 50.0))
    g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(3), LaneletId(4), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(1), LaneletId(3), EdgeKind.LEFT)
    g.add_edge(LaneletId(3), LaneletId(1), EdgeKind.RIGHT)
    g.add_edge(LaneletId(2), LaneletId(4), EdgeKind.LEFT)
    g.add_edge(LaneletId(4), LaneletId(2), EdgeKind.RIGHT)
    return g


def diamond_graph() -> RoutingGraph:
    """1 -> {2, 3} -> 4 with identical geometry per branch (a pure tie)."""
    g = RoutingGraph()
    g.add_lanelet(box_lanelet(1, 0.0, 0.0, 10.0))
    g.add_lanelet(box_lanelet(2, 10.0, 0.0, 10.0))
    g.add_lanelet(box_lanelet(3, 10.0, 3.5, 10.0))
    g.add_lanelet(box_lanelet(4, 20.0, 0.0, 10.0))
    g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(1), LaneletId(3), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(2), LaneletId(4), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(3), LaneletId(4), EdgeKind.SUCCESSOR)
    return g


def grid_graph(speeds: list[list[float]]) -> RoutingGraph:
    """DAG grid: cell (r,c) -> right is SUCCESSOR, -> down is LEFT (lane change).

    Cell (r, c) has id r*cols + c + 1, occupies x=[c*10, c*10+10] at y=r*4.
    """
    rows, cols = len(speeds), len(speeds[0])
    g = RoutingGraph()
    for r in range(rows):
        for c in range(cols):
            g.add_lanelet(
                box_lanelet(r * cols + c + 1, c * 10.0, r * 4.0, 10.0, speed=speeds[r][c])
            )
    for r in range(rows):
        for c in range(cols):
            lid = LaneletId(r * cols + c + 1)
            if c + 1 < cols:
                g.add_edge(lid, LaneletId(r * cols + c + 2), EdgeKind.SUCCESSOR)
            if r + 1 < rows:
                g.add_edge(lid, LaneletId((r + 1) * cols + c + 1), EdgeKind.LEFT)
    return g
