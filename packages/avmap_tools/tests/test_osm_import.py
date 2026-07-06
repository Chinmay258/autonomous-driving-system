"""OSM import: per-lane synthesis, shared dividers, strict lane discipline."""

import pytest
from avmap_tools import build_graph, import_osm_roads, validate_map
from avmap_tools.osm_import import lane_counts, parse_maxspeed
from avmap_tools.projection import LocalProjector

from avcore import EdgeKind, LaneletId, LatLng, Point2D, plan_route

ORIGIN = LatLng(lat=43.7384, lng=7.4246)
_PROJ = LocalProjector(ORIGIN)


def _node(nid: int, x: float, y: float) -> str:
    ll = _PROJ.to_latlng(Point2D(x, y))
    return f'<node id="{nid}" lat="{ll.lat!r}" lon="{ll.lng!r}" />'


def cross_fixture() -> str:
    """4-lane two-way primary (west-east) x 3-lane oneway secondary (northbound)."""
    return f"""<?xml version="1.0"?>
<osm version="0.6">
  {_node(1, -200.0, 0.0)}
  {_node(2, 0.0, 0.0)}
  {_node(3, 200.0, 0.0)}
  {_node(4, 0.0, -200.0)}
  {_node(5, 0.0, 200.0)}
  <way id="100">
    <nd ref="1" /><nd ref="2" /><nd ref="3" />
    <tag k="highway" v="primary" /><tag k="lanes" v="4" /><tag k="maxspeed" v="50" />
  </way>
  <way id="200">
    <nd ref="4" /><nd ref="2" /><nd ref="5" />
    <tag k="highway" v="secondary" /><tag k="lanes" v="3" /><tag k="oneway" v="yes" />
  </way>
</osm>"""


@pytest.fixture(scope="module")
def imported():
    lanelets, _origin = import_osm_roads(cross_fixture(), origin=ORIGIN)
    return lanelets, build_graph(lanelets)


class TestTagParsing:
    def test_maxspeed_kmh(self) -> None:
        assert parse_maxspeed("50", "primary") == pytest.approx(13.89, abs=0.01)

    def test_maxspeed_mph(self) -> None:
        assert parse_maxspeed("30 mph", "primary") == pytest.approx(13.41, abs=0.01)

    def test_maxspeed_fallback(self) -> None:
        assert parse_maxspeed(None, "residential") == pytest.approx(8.3)
        assert parse_maxspeed("fast", "primary") == pytest.approx(16.7)

    def test_lane_counts_two_way(self) -> None:
        assert lane_counts({"highway": "primary", "lanes": "4"}) == (2, 2)
        assert lane_counts({"highway": "primary", "lanes": "3", "lanes:forward": "2"}) == (2, 1)

    def test_lane_counts_oneway_and_defaults(self) -> None:
        assert lane_counts({"highway": "secondary", "lanes": "3", "oneway": "yes"}) == (3, 0)
        assert lane_counts({"highway": "residential"}) == (1, 1)
        assert lane_counts({"highway": "motorway", "oneway": "yes"}) == (3, 0)


class TestLaneSynthesis:
    def test_lanelet_count(self, imported) -> None:
        lanelets, _ = imported
        # Streets: primary 2 segs x (2 fwd + 2 bwd) = 8; oneway 2 segs x 3 = 6.
        # Junction connectors: east-in (2 straight + 1 left + 1 U-turn) +
        # west-in (2 straight + 1 right + 1 U-turn) + north-in (3 straight +
        # 1 left + 1 right; no U-turn on the oneway) = 13. Plus dead-end
        # U-turns at the primary's two tips = 2 (turn around at a cul-de-sac).
        assert len(lanelets) == 14 + 13 + 2

    def test_connectors_are_flagged_and_curved(self, imported) -> None:
        lanelets, _ = imported
        connectors = [ll for ll in lanelets if ll.is_connector]
        assert len(connectors) == 15
        # Bezier sampling: turning connectors carry more than 2 bound points.
        assert max(len(c.left_bound) for c in connectors) > 2

    def test_map_validates(self, imported) -> None:
        lanelets, _ = imported
        report = validate_map(lanelets)
        assert report.ok, report.issues

    def test_adjacent_same_direction_lanes_share_divider(self, imported) -> None:
        _lanelets, graph = imported
        # Eastbound lanes of the west segment are ids 1 (left) and 2 (right).
        kinds_1 = {(e.kind, e.target) for e in graph.edges_from(LaneletId(1))}
        assert (EdgeKind.RIGHT, LaneletId(2)) in kinds_1
        kinds_2 = {(e.kind, e.target) for e in graph.edges_from(LaneletId(2))}
        assert (EdgeKind.LEFT, LaneletId(1)) in kinds_2

    def test_no_lane_change_across_directions(self, imported) -> None:
        _, graph = imported
        # Westbound lanes of the west segment are ids 3, 4: no lateral edges
        # may connect them to eastbound 1, 2.
        for wb in (LaneletId(3), LaneletId(4)):
            for edge in graph.edges_from(wb):
                if edge.kind in (EdgeKind.LEFT, EdgeKind.RIGHT):
                    assert edge.target not in (LaneletId(1), LaneletId(2))

    def test_two_way_lanes_sit_on_correct_side(self, imported) -> None:
        lanelets, _ = imported
        by_id = {ll.id: ll for ll in lanelets}
        # Right-hand traffic: eastbound (fwd) lanes south of the axis (y<0),
        # westbound north (y>0).
        assert all(p.y < 0 for p in by_id[LaneletId(1)].centerline)
        assert all(p.y > 0 for p in by_id[LaneletId(3)].centerline)


class TestLaneDiscipline:
    def test_left_lane_has_no_direct_right_turn(self, imported) -> None:
        _, graph = imported
        # Westbound left lane (id 7, east segment) must NOT connect toward the
        # northbound exit lanes (12-14) without a lane change first.
        reachable_one_hop = set()
        for edge in graph.edges_from(LaneletId(7)):
            if edge.kind is EdgeKind.SUCCESSOR:
                for edge2 in graph.edges_from(edge.target):
                    reachable_one_hop.add(edge2.target)
        assert not reachable_one_hop & {LaneletId(12), LaneletId(13), LaneletId(14)}

    def test_right_turn_requires_lane_change(self, imported) -> None:
        _, graph = imported
        # Westbound left lane -> northbound middle lane: the planner must move
        # to the rightmost lane (RIGHT edge) before the turn connector.
        result = plan_route(graph, LaneletId(7), LaneletId(13))
        assert result.lane_changes >= 1
        assert LaneletId(8) in result.lanelet_ids  # the rightmost westbound lane

    def test_uturn_from_leftmost_lane(self, imported) -> None:
        lanelets, graph = imported
        # Goal directly behind on the same street: eastbound leftmost (1) can
        # U-turn at the junction onto westbound leftmost (3).
        result = plan_route(graph, LaneletId(1), LaneletId(3))
        assert result.lanelet_ids[0] == LaneletId(1)
        assert result.lanelet_ids[-1] == LaneletId(3)
        by_id = {ll.id: ll for ll in lanelets}
        assert any(by_id[lid].is_connector for lid in result.lanelet_ids)
        # The rightmost eastbound lane (2) has no U-turn of its own: any route
        # must first change left into lane 1.
        result2 = plan_route(graph, LaneletId(2), LaneletId(3))
        assert result2.lane_changes >= 1

    def test_straight_preserves_lane_index(self, imported) -> None:
        _, graph = imported
        # Eastbound left lane (1) continues via a straight connector into the
        # east segment's left lane (5), never diagonally into 6.
        connector_targets = set()
        for edge in graph.edges_from(LaneletId(1)):
            if edge.kind is EdgeKind.SUCCESSOR:
                for edge2 in graph.edges_from(edge.target):
                    if edge2.kind is EdgeKind.SUCCESSOR:
                        connector_targets.add(edge2.target)
        assert LaneletId(5) in connector_targets
        assert LaneletId(6) not in connector_targets


class TestDriveOnLeft:
    def test_left_hand_traffic_mirrors_lanes(self) -> None:
        lanelets, _ = import_osm_roads(cross_fixture(), origin=ORIGIN, drive_on="left")
        by_id = {ll.id: ll for ll in lanelets}
        # Eastbound lanes now ride the north side.
        assert all(p.y > 0 for p in by_id[LaneletId(1)].centerline)

    def test_rejects_bad_side(self) -> None:
        with pytest.raises(ValueError):
            import_osm_roads(cross_fixture(), drive_on="middle")


class TestRobustness:
    def test_no_drivable_ways_raises(self) -> None:
        xml = '<?xml version="1.0"?><osm version="0.6"></osm>'
        with pytest.raises(ValueError, match="no drivable ways"):
            import_osm_roads(xml)

    def test_footway_ignored(self) -> None:
        xml = f"""<?xml version="1.0"?>
<osm version="0.6">
  {_node(1, 0.0, 0.0)}{_node(2, 100.0, 0.0)}{_node(3, 200.0, 0.0)}
  <way id="1"><nd ref="1" /><nd ref="2" /><tag k="highway" v="footway" /></way>
  <way id="2"><nd ref="2" /><nd ref="3" /><tag k="highway" v="residential" /></way>
</osm>"""
        lanelets, _ = import_osm_roads(xml, origin=ORIGIN)
        # Only the residential way: 1 lane each direction, no junction split,
        # plus a dead-end U-turn at each tip.
        streets = [ll for ll in lanelets if not ll.is_connector]
        assert len(streets) == 2
        assert len(lanelets) == 4
