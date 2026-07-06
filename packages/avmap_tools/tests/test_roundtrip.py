import pytest
from avmap_tools import read_osm, write_osm, write_projector_info_yaml
from avmap_tools.projection import LocalProjector

from avcore import Lanelet, LaneletId, LatLng, Point2D

ORIGIN = LatLng(lat=43.7384, lng=7.4246)  # Monaco


def sample_lanelets() -> list[Lanelet]:
    return [
        Lanelet(
            id=LaneletId(1),
            left_bound=(Point2D(0.0, 1.75), Point2D(50.0, 1.75)),
            right_bound=(Point2D(0.0, -1.75), Point2D(50.0, -1.75)),
            speed_limit_mps=13.9,
        ),
        Lanelet(
            id=LaneletId(2),
            left_bound=(Point2D(0.0, 5.25), Point2D(50.0, 5.25)),
            right_bound=(Point2D(0.0, 1.75), Point2D(50.0, 1.75)),  # shares divider with 1
            speed_limit_mps=13.9,
        ),
    ]


class TestProjection:
    def test_roundtrip_is_identity(self) -> None:
        projector = LocalProjector(ORIGIN)
        point = Point2D(123.4, -567.8)
        back = projector.to_local(projector.to_latlng(point))
        assert back.x == pytest.approx(point.x, abs=1e-6)
        assert back.y == pytest.approx(point.y, abs=1e-6)


class TestOsmRoundtrip:
    def test_lanelets_survive_roundtrip(self) -> None:
        xml_text = write_osm(sample_lanelets(), ORIGIN)
        lanelets, origin = read_osm(xml_text)
        assert [ll.id for ll in lanelets] == [LaneletId(1), LaneletId(2)]
        assert origin.lat == pytest.approx(ORIGIN.lat, abs=1e-9)
        assert origin.lng == pytest.approx(ORIGIN.lng, abs=1e-9)
        for original, restored in zip(sample_lanelets(), lanelets, strict=True):
            assert restored.left_bound == original.left_bound  # exact via local_x/local_y
            assert restored.right_bound == original.right_bound
            assert restored.speed_limit_mps == pytest.approx(original.speed_limit_mps)
            assert restored.one_way == original.one_way
            assert restored.subtype == original.subtype

    def test_shared_divider_is_one_way_element(self) -> None:
        xml_text = write_osm(sample_lanelets(), ORIGIN)
        # Lanelet 1's left bound == lanelet 2's right bound -> written once.
        assert xml_text.count('k="type" v="line_thin"') == 3  # not 4

    def test_shared_divider_is_dashed_for_lane_changes(self) -> None:
        # lanelet2 traffic rules only allow lane changes across dashed lines;
        # the internal divider must be dashed, outer bounds stay solid.
        xml_text = write_osm(sample_lanelets(), ORIGIN)
        assert xml_text.count('v="dashed"') == 1
        assert xml_text.count('v="solid"') == 2

    def test_output_is_deterministic(self) -> None:
        assert write_osm(sample_lanelets(), ORIGIN) == write_osm(sample_lanelets(), ORIGIN)

    def test_projector_info_yaml(self) -> None:
        yaml_text = write_projector_info_yaml(ORIGIN)
        assert "latitude: 43.7384" in yaml_text
        assert "longitude: 7.4246" in yaml_text
