import dataclasses

import pytest

from avcore import Lanelet, LaneletId, Point2D
from avcore.models import polyline_length


def straight_lanelet(lid: int = 1, width: float = 3.5, length: float = 10.0) -> Lanelet:
    half = width / 2.0
    return Lanelet(
        id=LaneletId(lid),
        left_bound=(Point2D(0.0, half), Point2D(length, half)),
        right_bound=(Point2D(0.0, -half), Point2D(length, -half)),
        speed_limit_mps=13.9,
    )


class TestLaneletValidation:
    def test_rejects_short_bounds(self) -> None:
        with pytest.raises(ValueError, match="bounds"):
            Lanelet(
                id=LaneletId(1),
                left_bound=(Point2D(0, 0),),
                right_bound=(Point2D(0, -3), Point2D(10, -3)),
                speed_limit_mps=10.0,
            )

    def test_rejects_nonpositive_speed_limit(self) -> None:
        with pytest.raises(ValueError, match="speed limit"):
            Lanelet(
                id=LaneletId(1),
                left_bound=(Point2D(0, 0), Point2D(10, 0)),
                right_bound=(Point2D(0, -3), Point2D(10, -3)),
                speed_limit_mps=0.0,
            )

    def test_is_immutable(self) -> None:
        lanelet = straight_lanelet()
        with pytest.raises(dataclasses.FrozenInstanceError):
            lanelet.speed_limit_mps = 99.0  # type: ignore[misc]


class TestGeometry:
    def test_centerline_is_midline(self) -> None:
        lanelet = straight_lanelet(width=4.0, length=10.0)
        assert lanelet.centerline == (Point2D(0.0, 0.0), Point2D(10.0, 0.0))

    def test_length_matches_centerline(self) -> None:
        assert straight_lanelet(length=25.0).length_m == pytest.approx(25.0)

    def test_polyline_length_degenerate(self) -> None:
        assert polyline_length(()) == 0.0
        assert polyline_length((Point2D(1, 1),)) == 0.0

    def test_point_distance(self) -> None:
        assert Point2D(0, 0).distance_to(Point2D(3, 4)) == pytest.approx(5.0)
