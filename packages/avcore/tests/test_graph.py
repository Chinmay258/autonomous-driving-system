import pytest

from avcore import Edge, EdgeKind, LaneletId, RoutingGraph, UnknownLaneletError
from tests.test_models import straight_lanelet


@pytest.fixture()
def two_node_graph() -> RoutingGraph:
    g = RoutingGraph()
    g.add_lanelet(straight_lanelet(1))
    g.add_lanelet(straight_lanelet(2))
    g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.SUCCESSOR)
    return g


class TestRoutingGraph:
    def test_membership_and_size(self, two_node_graph: RoutingGraph) -> None:
        assert LaneletId(1) in two_node_graph
        assert LaneletId(99) not in two_node_graph
        assert len(two_node_graph) == 2
        assert sorted(two_node_graph) == [LaneletId(1), LaneletId(2)]

    def test_edges_from(self, two_node_graph: RoutingGraph) -> None:
        assert two_node_graph.edges_from(LaneletId(1)) == (
            Edge(target=LaneletId(2), kind=EdgeKind.SUCCESSOR),
        )
        assert two_node_graph.edges_from(LaneletId(2)) == ()

    def test_lanelet_lookup(self, two_node_graph: RoutingGraph) -> None:
        assert two_node_graph.lanelet(LaneletId(1)).id == LaneletId(1)

    def test_duplicate_lanelet_rejected(self, two_node_graph: RoutingGraph) -> None:
        with pytest.raises(ValueError, match="already"):
            two_node_graph.add_lanelet(straight_lanelet(1))

    def test_edge_to_unknown_lanelet_rejected(self, two_node_graph: RoutingGraph) -> None:
        with pytest.raises(UnknownLaneletError):
            two_node_graph.add_edge(LaneletId(1), LaneletId(99), EdgeKind.LEFT)

    def test_lookup_unknown_lanelet_rejected(self, two_node_graph: RoutingGraph) -> None:
        with pytest.raises(UnknownLaneletError):
            two_node_graph.lanelet(LaneletId(99))
        with pytest.raises(UnknownLaneletError):
            two_node_graph.edges_from(LaneletId(99))
