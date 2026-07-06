from itertools import pairwise

import pytest

from avcore import Distance, LaneletId, TravelTime, UnreachableGoalError, plan_route
from tests.mapbuilders import (
    box_lanelet,
    chain_graph,
    diamond_graph,
    fork_graph,
    lane_change_graph,
)


class TestBasicRouting:
    def test_chain_route(self) -> None:
        result = plan_route(chain_graph(3), LaneletId(1), LaneletId(3))
        assert result.lanelet_ids == (LaneletId(1), LaneletId(2), LaneletId(3))
        assert result.distance_m == pytest.approx(30.0)
        assert result.eta_s == pytest.approx(30.0 / 13.9)
        assert result.lane_changes == 0

    def test_chain_centerline_is_stitched_and_continuous(self) -> None:
        result = plan_route(chain_graph(3), LaneletId(1), LaneletId(3))
        xs = [p.x for p in result.centerline]
        assert xs == sorted(xs)  # monotone along the corridor
        assert len(result.centerline) == len(set(result.centerline))  # joints deduped
        assert result.centerline[0].x == pytest.approx(0.0)
        assert result.centerline[-1].x == pytest.approx(30.0)

    def test_start_equals_goal(self) -> None:
        result = plan_route(chain_graph(3), LaneletId(2), LaneletId(2))
        assert result.lanelet_ids == (LaneletId(2),)
        assert result.distance_m == pytest.approx(10.0)
        assert result.lane_changes == 0

    def test_goal_behind_ego_is_unreachable_on_one_way_chain(self) -> None:
        with pytest.raises(UnreachableGoalError):
            plan_route(chain_graph(3), LaneletId(3), LaneletId(1))

    def test_disconnected_island_is_unreachable(self) -> None:
        g = chain_graph(2)
        g.add_lanelet(box_lanelet(99, 1000.0, 1000.0, 10.0))
        with pytest.raises(UnreachableGoalError):
            plan_route(g, LaneletId(1), LaneletId(99))


class TestCostModels:
    def test_travel_time_prefers_fast_branch(self) -> None:
        result = plan_route(fork_graph(), LaneletId(1), LaneletId(4), TravelTime())
        assert LaneletId(3) in result.lanelet_ids  # long but fast dogleg

    def test_distance_prefers_short_branch(self) -> None:
        result = plan_route(fork_graph(), LaneletId(1), LaneletId(4), Distance())
        assert LaneletId(2) in result.lanelet_ids  # short but slow

    def test_default_cost_is_travel_time(self) -> None:
        assert plan_route(fork_graph(), LaneletId(1), LaneletId(4)) == plan_route(
            fork_graph(), LaneletId(1), LaneletId(4), TravelTime()
        )


class TestLaneChanges:
    def test_route_across_lanes_counts_changes(self) -> None:
        result = plan_route(lane_change_graph(), LaneletId(1), LaneletId(4))
        assert result.lane_changes == 1
        assert len(result.lanelet_ids) == 3
        assert result.lanelet_ids[0] == LaneletId(1)
        assert result.lanelet_ids[-1] == LaneletId(4)

    def test_lane_change_never_backtracks(self) -> None:
        # Regression: lane changes used to re-traverse the neighbour lane
        # from its start, drawing a zigzag. The stitched path must move
        # strictly forward along the corridor.
        result = plan_route(lane_change_graph(), LaneletId(1), LaneletId(4))
        xs = [p.x for p in result.centerline]
        assert all(b >= a - 1e-9 for a, b in pairwise(xs))
        # Parallel span is traversed once: ~100 m + a short diagonal, not 150.
        assert 95.0 < result.distance_m < 115.0

    def test_lane_change_speeds_parallel_to_centerline(self) -> None:
        result = plan_route(lane_change_graph(), LaneletId(1), LaneletId(4))
        assert len(result.speed_limits_mps) == len(result.centerline)

    def test_double_lane_change_is_one_smooth_diagonal(self) -> None:
        # Three parallel lanes; only the top one continues. Crossing both
        # lanes must be a single forward maneuver: no sawtooth (regression
        # for stacked per-lane jogs), strictly monotonic in x AND y.
        from avcore import EdgeKind, RoutingGraph
        from tests.mapbuilders import box_lanelet

        g = RoutingGraph()
        for lane_index in range(3):
            g.add_lanelet(box_lanelet(lane_index + 1, 0.0, lane_index * 3.5, 80.0))
        g.add_lanelet(box_lanelet(4, 80.0, 7.0, 30.0))
        g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.LEFT)
        g.add_edge(LaneletId(2), LaneletId(3), EdgeKind.LEFT)
        g.add_edge(LaneletId(3), LaneletId(4), EdgeKind.SUCCESSOR)

        result = plan_route(g, LaneletId(1), LaneletId(4))
        assert result.lane_changes == 2
        xs = [p.x for p in result.centerline]
        ys = [p.y for p in result.centerline]
        assert all(b >= a - 1e-9 for a, b in pairwise(xs)), "path moved backward"
        assert all(b >= a - 1e-6 for a, b in pairwise(ys)), "diagonal wobbled"

    def test_same_lane_route_has_no_changes(self) -> None:
        result = plan_route(lane_change_graph(), LaneletId(1), LaneletId(2))
        assert result.lanelet_ids == (LaneletId(1), LaneletId(2))
        assert result.lane_changes == 0


class TestDeterminism:
    def test_pure_tie_resolves_stably(self) -> None:
        result = plan_route(diamond_graph(), LaneletId(1), LaneletId(4))
        # Equal-cost branches: tie-break favors the lower lanelet id.
        assert result.lanelet_ids == (LaneletId(1), LaneletId(2), LaneletId(4))

    def test_repeated_calls_identical(self) -> None:
        for graph_factory in (chain_graph, fork_graph, lane_change_graph, diamond_graph):
            g = graph_factory()
            first = plan_route(g, LaneletId(1), LaneletId(4) if len(g) >= 4 else LaneletId(2))
            second = plan_route(g, LaneletId(1), LaneletId(4) if len(g) >= 4 else LaneletId(2))
            assert first == second
