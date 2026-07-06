"""Lane detection pipeline: threshold -> BEV -> sliding windows -> fit."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import numpy.typing as npt

from avlane.camera import (
    BEV_HEIGHT,
    BEV_WIDTH,
    METERS_PER_PX_X,
    METERS_PER_PX_Y,
    Image,
    PerspectiveConfig,
)

N_WINDOWS = 9
WINDOW_MARGIN_PX = 60
MIN_PIXELS_PER_WINDOW = 30
MIN_PIXELS_PER_LANE = 200


class DetectionFailed(Exception):
    """Not enough lane evidence in the frame."""


@dataclass(frozen=True)
class LaneDetection:
    """Metric lane geometry (ego frame: x lateral, y forward, meters)."""

    left_fit_px: tuple[float, float, float]  # x = a*y^2 + b*y + c in BEV px
    right_fit_px: tuple[float, float, float]
    lane_width_m: float
    curvature_radius_m: float
    center_offset_m: float
    left_boundary_m: tuple[tuple[float, float], ...]
    right_boundary_m: tuple[tuple[float, float], ...]


def threshold(frame_bgr: Image) -> Image:
    """Binary mask of probable lane markings (bright paint + x-gradient)."""
    hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
    lightness = hls[:, :, 1]
    sobel = cv2.Sobel(lightness, cv2.CV_64F, 1, 0, ksize=5)
    sobel_norm = np.uint8(255 * np.absolute(sobel) / max(np.max(np.absolute(sobel)), 1e-6))
    mask = ((lightness > 170) | (sobel_norm > 60)).astype(np.uint8) * 255
    return np.asarray(mask, dtype=np.uint8)


def _sliding_window(
    binary_bev: Image,
) -> tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    histogram = np.sum(binary_bev[BEV_HEIGHT // 2 :, :], axis=0)
    midpoint = BEV_WIDTH // 2
    left_base = int(np.argmax(histogram[:midpoint]))
    right_base = midpoint + int(np.argmax(histogram[midpoint:]))

    nonzero_y, nonzero_x = binary_bev.nonzero()
    window_height = BEV_HEIGHT // N_WINDOWS
    left_idx: list[npt.NDArray[np.intp]] = []
    right_idx: list[npt.NDArray[np.intp]] = []
    left_current, right_current = left_base, right_base

    for window in range(N_WINDOWS):
        y_low = BEV_HEIGHT - (window + 1) * window_height
        y_high = BEV_HEIGHT - window * window_height
        for current, bucket in ((left_current, left_idx), (right_current, right_idx)):
            hits = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= current - WINDOW_MARGIN_PX)
                & (nonzero_x < current + WINDOW_MARGIN_PX)
            ).nonzero()[0]
            bucket.append(hits)
        if len(left_idx[-1]) > MIN_PIXELS_PER_WINDOW:
            left_current = int(np.mean(nonzero_x[left_idx[-1]]))
        if len(right_idx[-1]) > MIN_PIXELS_PER_WINDOW:
            right_current = int(np.mean(nonzero_x[right_idx[-1]]))

    left = np.concatenate(left_idx)
    right = np.concatenate(right_idx)
    if len(left) < MIN_PIXELS_PER_LANE or len(right) < MIN_PIXELS_PER_LANE:
        raise DetectionFailed(f"insufficient lane pixels (left={len(left)}, right={len(right)})")
    return (
        nonzero_x[left].astype(np.float64),
        nonzero_y[left].astype(np.float64),
        nonzero_x[right].astype(np.float64),
        nonzero_y[right].astype(np.float64),
    )


def _boundary_m(fit: tuple[float, float, float]) -> tuple[tuple[float, float], ...]:
    points = []
    for y in range(BEV_HEIGHT, -1, -72):
        x = fit[0] * y * y + fit[1] * y + fit[2]
        lateral = (x - BEV_WIDTH / 2.0) * METERS_PER_PX_X
        forward = (BEV_HEIGHT - y) * METERS_PER_PX_Y
        points.append((round(lateral, 3), round(forward, 3)))
    return tuple(points)


def detect_lanes(frame_bgr: Image, camera: PerspectiveConfig | None = None) -> LaneDetection:
    """Detect the ego lane. Raises DetectionFailed on poor evidence."""
    camera = camera or PerspectiveConfig()
    binary = threshold(frame_bgr)
    bev = camera.to_birdseye(binary)
    left_x, left_y, right_x, right_y = _sliding_window(bev)

    left_fit = tuple(float(v) for v in np.polyfit(left_y, left_x, 2))
    right_fit = tuple(float(v) for v in np.polyfit(right_y, right_x, 2))

    ys = np.linspace(0, BEV_HEIGHT, 20)
    left_xs = np.polyval(left_fit, ys)
    right_xs = np.polyval(right_fit, ys)
    lane_width = float(np.mean(right_xs - left_xs)) * METERS_PER_PX_X

    # Curvature in metric space at the vehicle (bottom of the view).
    left_fit_m = np.polyfit(left_y * METERS_PER_PX_Y, left_x * METERS_PER_PX_X, 2)
    y_eval = BEV_HEIGHT * METERS_PER_PX_Y
    denominator = abs(2.0 * left_fit_m[0])
    curvature = (
        float((1.0 + (2.0 * left_fit_m[0] * y_eval + left_fit_m[1]) ** 2) ** 1.5 / denominator)
        if denominator > 1e-9
        else float("inf")
    )

    lane_center_px = (np.polyval(left_fit, BEV_HEIGHT) + np.polyval(right_fit, BEV_HEIGHT)) / 2.0
    offset = float(lane_center_px - BEV_WIDTH / 2.0) * METERS_PER_PX_X

    return LaneDetection(
        left_fit_px=(left_fit[0], left_fit[1], left_fit[2]),
        right_fit_px=(right_fit[0], right_fit[1], right_fit[2]),
        lane_width_m=round(lane_width, 3),
        curvature_radius_m=round(curvature, 1),
        center_offset_m=round(offset, 3),
        left_boundary_m=_boundary_m((left_fit[0], left_fit[1], left_fit[2])),
        right_boundary_m=_boundary_m((right_fit[0], right_fit[1], right_fit[2])),
    )


def overlay(
    frame_bgr: Image, detection: LaneDetection, camera: PerspectiveConfig | None = None
) -> Image:
    """Annotated frame: detected lane surface painted back into camera view."""
    camera = camera or PerspectiveConfig()
    canvas = np.zeros((BEV_HEIGHT, BEV_WIDTH, 3), dtype=np.uint8)
    ys = np.arange(0, BEV_HEIGHT)
    left = np.polyval(detection.left_fit_px, ys)
    right = np.polyval(detection.right_fit_px, ys)
    pts_left = np.array([np.transpose(np.vstack([left, ys]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right, ys])))])
    polygon = np.hstack((pts_left, pts_right)).astype(np.int32)
    cv2.fillPoly(canvas, [polygon], (0, 180, 70))
    unwarped = camera.to_camera(canvas)
    blended = cv2.addWeighted(frame_bgr, 1.0, unwarped, 0.35, 0)
    return np.asarray(blended, dtype=np.uint8)
