import pytest
from avmap_tools import build_graph, read_osm, synthetic_town, validate_map, write_osm

from avcore import (
    EdgeKind,
    LaneletId,
    LatLng,
    TravelTime,
    filter_lanelets,
    plan_route,
)

ORIGIN = LatLng(lat=43.7384, lng=7.4246)


@pytest.fixture(scope="module")
def town_graph():
    lanelets = synthetic_town(2, 2)
    return lanelets, build_graph(lanelets)


class TestSyntheticTown:
    def test_validates_clean(self, town_graph) -> None:
        lanelets, _ = town_graph
        report = validate_map(lanelets)
        assert report.ok, report.issues

    def test_survives_filtering_intact(self, town_graph) -> None:
        lanelets, graph = town_graph
        assert len(filter_lanelets(graph)) == len(lanelets)

    def test_every_lanelet_reaches_every_other(self, town_graph) -> None:
        _, graph = town_graph
        # Strong connectivity via BFS from one node + BFS on reversed edges.
        ids = list(graph)
        forward = {lid: {e.target for e in graph.edges_from(lid)} for lid in ids}
        reverse: dict[LaneletId, set[LaneletId]] = {lid: set() for lid in ids}
        for src, targets in forward.items():
            for t in targets:
                reverse[t].add(src)

        def bfs(adjacency: dict[LaneletId, set[LaneletId]]) -> set[LaneletId]:
            seen = {ids[0]}
            frontier = [ids[0]]
            while frontier:
                for nxt in adjacency[frontier.pop()]:
                    if nxt not in seen:
                        seen.add(nxt)
                        frontier.append(nxt)
            return seen

        assert bfs(forward) == set(ids)
        assert bfs(reverse) == set(ids)

    def test_route_across_town(self, town_graph) -> None:
        lanelets, graph = town_graph
        result = plan_route(graph, LaneletId(1), LaneletId(len(lanelets) // 2), TravelTime())
        assert result.distance_m > 0
        assert result.lanelet_ids[0] == LaneletId(1)

    def test_deterministic_generation(self) -> None:
        a = synthetic_town(2, 3)
        b = synthetic_town(2, 3)
        assert a == b

    def test_rejects_bad_parameters(self) -> None:
        with pytest.raises(ValueError):
            synthetic_town(0, 2)
        with pytest.raises(ValueError):
            synthetic_town(2, 2, block_m=10.0)


class TestGraphDerivation:
    def test_successors_via_endpoint_matching(self, town_graph) -> None:
        _, graph = town_graph
        kinds = {e.kind for lid in graph for e in graph.edges_from(lid)}
        assert EdgeKind.SUCCESSOR in kinds

    def test_town_roundtrips_through_osm(self, town_graph) -> None:
        lanelets, graph = town_graph
        restored, _ = read_osm(write_osm(lanelets, ORIGIN))
        restored_graph = build_graph(restored)
        assert len(restored_graph) == len(graph)
        for lid in graph:
            assert sorted(e.target for e in restored_graph.edges_from(lid)) == sorted(
                e.target for e in graph.edges_from(lid)
            )


class TestValidation:
    def test_empty_map_fails(self) -> None:
        report = validate_map([])
        assert not report.ok

    def test_raise_if_failed(self) -> None:
        from avcore import MapValidationError

        with pytest.raises(MapValidationError):
            validate_map([]).raise_if_failed()

    def test_duplicate_ids_detected(self) -> None:
        lanelets = synthetic_town(1, 1)
        report = validate_map([*lanelets, lanelets[0]])
        assert any("duplicate" in issue for issue in report.issues)
