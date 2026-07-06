"""Detection accuracy against synthetic ground truth."""

import numpy as np
import pytest
from avlane import DetectionFailed, detect_lanes, render_road_frame
from avlane.pipeline import overlay
from avlane.synthetic import analytic_curvature_m


class TestStraightRoad:
    def test_recovers_lane_width_and_center(self) -> None:
        detection = detect_lanes(render_road_frame())
        assert detection.lane_width_m == pytest.approx(3.7, abs=0.25)
        # Lane rendered centered in BEV (50..350 around 200): zero offset.
        assert detection.center_offset_m == pytest.approx(0.0, abs=0.15)

    def test_straight_road_has_huge_radius(self) -> None:
        detection = detect_lanes(render_road_frame())
        assert detection.curvature_radius_m > 2000

    def test_boundaries_are_metric_polylines(self) -> None:
        detection = detect_lanes(render_road_frame())
        assert len(detection.left_boundary_m) >= 8
        forwards = [p[1] for p in detection.left_boundary_m]
        assert forwards[0] == pytest.approx(0.0, abs=0.1)
        assert max(forwards) == pytest.approx(30.0, abs=1.0)


class TestLateralOffset:
    @pytest.mark.parametrize("offset_px", [-40.0, 40.0])
    def test_recovers_injected_offset(self, offset_px: float) -> None:
        detection = detect_lanes(render_road_frame(offset_px=offset_px))
        expected = offset_px * (3.7 / 300.0)
        assert detection.center_offset_m == pytest.approx(expected, abs=0.15)


class TestCurvature:
    @pytest.mark.parametrize("curve_px", [5e-5, -5e-5])
    def test_recovers_curvature_magnitude_and_sign(self, curve_px: float) -> None:
        detection = detect_lanes(render_road_frame(curve_px=curve_px))
        recovered_a = detection.left_fit_px[0]
        assert np.sign(recovered_a) == np.sign(curve_px)
        analytic_radius = 1.0 / (2.0 * abs(analytic_curvature_m(curve_px)))
        assert detection.curvature_radius_m == pytest.approx(analytic_radius, rel=0.6)


class TestRobustness:
    def test_empty_frame_fails_cleanly(self) -> None:
        black = np.zeros((360, 640, 3), dtype=np.uint8)
        with pytest.raises(DetectionFailed):
            detect_lanes(black)

    def test_overlay_returns_frame_shaped_image(self) -> None:
        frame = render_road_frame()
        annotated = overlay(frame, detect_lanes(frame))
        assert annotated.shape == frame.shape
