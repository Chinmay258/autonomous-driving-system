import pytest

from avcore import (
    EdgeKind,
    LaneletId,
    LaneletSubtype,
    Point2D,
    RouteResult,
    RoutingGraph,
    filter_lanelets,
    max_curvature,
    rank_routes,
)
from tests.mapbuilders import box_lanelet, chain_graph


def messy_graph() -> RoutingGraph:
    """Sane chain 1->2->3 plus one of every defect."""
    g = chain_graph(3)
    g.add_lanelet(box_lanelet(4, 0.0, 10.0, 10.0, subtype=LaneletSubtype.CROSSWALK))
    g.add_lanelet(box_lanelet(5, 0.0, 20.0, 10.0, width=1.0))  # too narrow
    g.add_lanelet(box_lanelet(6, 0.0, 30.0, 0.5))  # too short
    g.add_lanelet(box_lanelet(7, 500.0, 500.0, 10.0))  # sane but disconnected island
    g.add_edge(LaneletId(1), LaneletId(4), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(2), LaneletId(5), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(3), LaneletId(6), EdgeKind.SUCCESSOR)
    return g


class TestFilterLanelets:
    def test_keeps_only_sane_connected_road(self) -> None:
        filtered = filter_lanelets(messy_graph())
        assert sorted(filtered) == [LaneletId(1), LaneletId(2), LaneletId(3)]

    def test_preserves_surviving_edges(self) -> None:
        filtered = filter_lanelets(messy_graph())
        assert [e.target for e in filtered.edges_from(LaneletId(1))] == [LaneletId(2)]
        assert [e.target for e in filtered.edges_from(LaneletId(2))] == [LaneletId(3)]

    def test_too_wide_lanelet_dropped(self) -> None:
        g = chain_graph(2)
        g.add_lanelet(box_lanelet(9, 20.0, 0.0, 10.0, width=8.0))
        g.add_edge(LaneletId(2), LaneletId(9), EdgeKind.SUCCESSOR)
        assert LaneletId(9) not in filter_lanelets(g)

    def test_empty_graph(self) -> None:
        assert len(filter_lanelets(RoutingGraph())) == 0

    def test_result_is_new_graph(self) -> None:
        g = chain_graph(2)
        filtered = filter_lanelets(g)
        filtered.add_lanelet(box_lanelet(50, 100.0, 100.0, 10.0))
        assert LaneletId(50) not in g  # original untouched


def route(ids: tuple[int, ...], eta: float, dist: float, changes: int) -> RouteResult:
    return RouteResult(
        lanelet_ids=tuple(LaneletId(i) for i in ids),
        centerline=(Point2D(0, 0), Point2D(dist, 0)),
        distance_m=dist,
        eta_s=eta,
        lane_changes=changes,
    )


class TestMaxCurvature:
    def test_straight_line_is_zero(self) -> None:
        line = (Point2D(0, 0), Point2D(10, 0), Point2D(20, 0), Point2D(30, 0))
        assert max_curvature(line) == 0.0

    def test_right_angle_corner(self) -> None:
        corner = (Point2D(0, 0), Point2D(10, 0), Point2D(10, 10))
        # Circle through these three points has radius 5*sqrt(2) -> k ~ 0.1414
        assert max_curvature(corner) == pytest.approx(0.1414, abs=1e-3)

    def test_short_polyline_is_zero(self) -> None:
        assert max_curvature((Point2D(0, 0), Point2D(1, 1))) == 0.0


class TestRankRoutes:
    def test_orders_by_eta_then_distance(self) -> None:
        a = route((1, 2), eta=20.0, dist=100.0, changes=0)
        b = route((3, 4), eta=10.0, dist=200.0, changes=2)
        c = route((5, 6), eta=10.0, dist=150.0, changes=1)
        assert rank_routes([a, b, c]) == [c, b, a]

    def test_max_lane_changes_filter(self) -> None:
        a = route((1,), eta=5.0, dist=50.0, changes=3)
        b = route((2,), eta=9.0, dist=90.0, changes=1)
        assert rank_routes([a, b], max_lane_changes=2) == [b]

    def test_max_curvature_filter(self) -> None:
        sharp = RouteResult(
            lanelet_ids=(LaneletId(1),),
            centerline=(Point2D(0, 0), Point2D(10, 0), Point2D(10, 10)),
            distance_m=20.0,
            eta_s=2.0,
            lane_changes=0,
        )
        gentle = route((2,), eta=8.0, dist=80.0, changes=0)
        assert rank_routes([sharp, gentle], max_curvature_1pm=0.1) == [gentle]

    def test_empty_candidates(self) -> None:
        assert rank_routes([]) == []
