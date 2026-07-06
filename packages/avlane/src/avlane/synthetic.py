"""Synthetic road frames with known ground truth (CI-verifiable detection)."""

from __future__ import annotations

import cv2
import numpy as np

from avlane.camera import (
    BEV_HEIGHT,
    BEV_WIDTH,
    METERS_PER_PX_X,
    METERS_PER_PX_Y,
    Image,
    PerspectiveConfig,
)

LANE_WIDTH_PX = 300


def analytic_curvature_m(curve_px: float) -> float:
    """Metric quadratic coefficient equivalent of the px-space curve factor."""
    return curve_px * METERS_PER_PX_X / (METERS_PER_PX_Y**2)


def render_road_frame(
    *,
    curve_px: float = 0.0,
    offset_px: float = 0.0,
    camera: PerspectiveConfig | None = None,
) -> Image:
    """Camera-view frame of a road whose BEV lane geometry is known exactly.

    Left boundary in BEV: x(y) = 50 + offset_px + curve_px * (H - y)^2 — a
    positive ``curve_px`` bends the lane toward +x with increasing distance.
    """
    camera = camera or PerspectiveConfig()
    bev = np.full((BEV_HEIGHT, BEV_WIDTH, 3), 60, dtype=np.uint8)  # asphalt
    for y in range(BEV_HEIGHT):
        depth = BEV_HEIGHT - y
        x_left = 50.0 + offset_px + curve_px * depth * depth
        x_right = x_left + LANE_WIDTH_PX
        for x_line in (x_left, x_right):
            cv2.line(
                bev,
                (round(x_line), y),
                (round(x_line), min(y + 1, BEV_HEIGHT - 1)),
                (250, 250, 250),
                6,
            )
    frame = camera.to_camera(bev)
    return np.asarray(frame, dtype=np.uint8)
