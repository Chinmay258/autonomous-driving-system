"""Perspective model: camera view <-> metric bird's-eye view."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import cv2
import numpy as np
import numpy.typing as npt

Image = npt.NDArray[np.uint8]

BEV_WIDTH = 400
BEV_HEIGHT = 720
# Lane is ~3.7 m wide and rendered ~300 px wide; view depth ~30 m.
METERS_PER_PX_X = 3.7 / 300.0
METERS_PER_PX_Y = 30.0 / 720.0


@dataclass(frozen=True)
class PerspectiveConfig:
    """Maps a camera trapezoid onto the metric bird's-eye rectangle."""

    frame_width: int = 640
    frame_height: int = 360
    # Trapezoid in the camera image that corresponds to the BEV rectangle.
    src: tuple[tuple[float, float], ...] = (
        (120.0, 355.0),
        (520.0, 355.0),
        (390.0, 200.0),
        (250.0, 200.0),
    )
    dst: tuple[tuple[float, float], ...] = field(
        default=(
            (50.0, float(BEV_HEIGHT)),
            (350.0, float(BEV_HEIGHT)),
            (350.0, 0.0),
            (50.0, 0.0),
        )
    )

    @cached_property
    def matrix(self) -> npt.NDArray[np.float64]:
        return np.asarray(
            cv2.getPerspectiveTransform(
                np.asarray(self.src, dtype=np.float32),
                np.asarray(self.dst, dtype=np.float32),
            ),
            dtype=np.float64,
        )

    @cached_property
    def matrix_inv(self) -> npt.NDArray[np.float64]:
        return np.asarray(np.linalg.inv(self.matrix), dtype=np.float64)

    def to_birdseye(self, frame: Image) -> Image:
        warped = cv2.warpPerspective(frame, self.matrix, (BEV_WIDTH, BEV_HEIGHT))
        return np.asarray(warped, dtype=np.uint8)

    def to_camera(self, birdseye: Image) -> Image:
        warped = cv2.warpPerspective(
            birdseye, self.matrix_inv, (self.frame_width, self.frame_height)
        )
        return np.asarray(warped, dtype=np.uint8)
