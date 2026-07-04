"""Contract tests for the frozen plan_route API.

Input validation is active now; the A* pass itself is P1 (xfail strict, so CI
flips loudly the moment it's implemented and these must be replaced).
"""

import pytest

from avcore import LaneletId, RoutingGraph, UnknownLaneletError, plan_route
from tests.test_models import straight_lanelet


@pytest.fixture()
def graph() -> RoutingGraph:
    g = RoutingGraph()
    g.add_lanelet(straight_lanelet(1))
    g.add_lanelet(straight_lanelet(2))
    return g


def test_unknown_start_raises(graph: RoutingGraph) -> None:
    with pytest.raises(UnknownLaneletError):
        plan_route(graph, LaneletId(99), LaneletId(2))


def test_unknown_goal_raises(graph: RoutingGraph) -> None:
    with pytest.raises(UnknownLaneletError):
        plan_route(graph, LaneletId(1), LaneletId(99))


@pytest.mark.xfail(reason="A* lands in P1", raises=NotImplementedError, strict=True)
def test_plan_route_same_node(graph: RoutingGraph) -> None:
    result = plan_route(graph, LaneletId(1), LaneletId(1))
    assert result.lanelet_ids == (LaneletId(1),)
