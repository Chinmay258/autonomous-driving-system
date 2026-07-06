"""avlane — camera lane detection (P3 of the original project).

Classical, fully explainable pipeline: color/gradient threshold -> bird's-eye
warp -> sliding-window search -> quadratic fit -> metric lane geometry. The
synthetic renderer provides ground-truth frames so the whole pipeline is
verified in CI without external datasets.
"""

from avlane.camera import PerspectiveConfig
from avlane.pipeline import DetectionFailed, LaneDetection, detect_lanes
from avlane.synthetic import render_road_frame

__all__ = [
    "DetectionFailed",
    "LaneDetection",
    "PerspectiveConfig",
    "detect_lanes",
    "render_road_frame",
]
