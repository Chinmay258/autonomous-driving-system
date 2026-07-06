import pytest

from avcore import LaneletId, Point2D, plan_route, trim_route
from avcore.routeops import nearest_arclength, slice_polyline
from tests.mapbuilders import chain_graph

LINE = (Point2D(0, 0), Point2D(10, 0), Point2D(20, 0), Point2D(30, 0))


class TestSlicePolyline:
    def test_interior_slice_interpolates_endpoints(self) -> None:
        sliced = slice_polyline(LINE, 5.0, 25.0)
        assert sliced[0] == Point2D(5.0, 0.0)
        assert sliced[-1] == Point2D(25.0, 0.0)
        assert Point2D(10, 0) in sliced and Point2D(20, 0) in sliced

    def test_clamps_out_of_range(self) -> None:
        assert slice_polyline(LINE, -5.0, 99.0) == LINE

    def test_degenerate_slice_has_two_points(self) -> None:
        assert len(slice_polyline(LINE, 12.0, 12.0)) == 2


class TestNearestArclength:
    def test_projects_onto_segment(self) -> None:
        assert nearest_arclength(LINE, Point2D(17.0, 3.0)) == pytest.approx(17.0)

    def test_first_vs_last_preference(self) -> None:
        # A path that doubles back: passes x=5 twice (arclengths 5 and 15).
        loop = (Point2D(0, 0), Point2D(10, 0), Point2D(10, 1), Point2D(0, 1))
        target = Point2D(5.0, 0.5)
        first = nearest_arclength(loop, target, prefer="first")
        last = nearest_arclength(loop, target, prefer="last")
        assert first < 10.0 < last


class TestTrimRoute:
    def test_trims_to_projections(self) -> None:
        route = plan_route(chain_graph(3), LaneletId(1), LaneletId(3))  # 0..30 m
        trimmed = trim_route(route, Point2D(4.0, 2.0), Point2D(26.0, -1.0))
        assert trimmed.centerline[0].x == pytest.approx(4.0)
        assert trimmed.centerline[-1].x == pytest.approx(26.0)
        assert trimmed.distance_m == pytest.approx(22.0)
        assert len(trimmed.speed_limits_mps) == len(trimmed.centerline)
        assert trimmed.eta_s < route.eta_s

    def test_degenerate_backwards_clicks_keep_minimal_hop(self) -> None:
        route = plan_route(chain_graph(3), LaneletId(1), LaneletId(3))
        trimmed = trim_route(route, Point2D(20.0, 0.0), Point2D(5.0, 0.0))
        assert trimmed.distance_m <= route.distance_m
        assert len(trimmed.centerline) >= 2

    def test_lanelet_ids_preserved(self) -> None:
        route = plan_route(chain_graph(3), LaneletId(1), LaneletId(3))
        trimmed = trim_route(route, Point2D(4.0, 0.0), Point2D(26.0, 0.0))
        assert trimmed.lanelet_ids == route.lanelet_ids
