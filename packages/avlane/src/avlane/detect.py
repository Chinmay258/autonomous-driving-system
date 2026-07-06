"""CLI: dashcam video -> lanes.json + annotated video.

uv run python -m avlane.detect --input dashcam.mp4 --output-dir out/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from avlane.camera import PerspectiveConfig
from avlane.pipeline import DetectionFailed, detect_lanes, overlay


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="video file")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--max-frames", type=int, default=0, help="0 = all")
    args = parser.parse_args(argv)

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    camera = PerspectiveConfig()
    capture = cv2.VideoCapture(args.input)
    if not capture.isOpened():
        print(f"cannot open {args.input}", file=sys.stderr)
        return 1
    fps = capture.get(cv2.CAP_PROP_FPS) or 25.0
    writer = None
    frames: list[dict[str, object]] = []
    index = detected = 0

    while True:
        ok, frame = capture.read()
        if not ok or (args.max_frames and index >= args.max_frames):
            break
        frame = cv2.resize(frame, (camera.frame_width, camera.frame_height))
        annotated = frame
        try:
            detection = detect_lanes(np.asarray(frame, dtype=np.uint8), camera)
            detected += 1
            annotated = overlay(np.asarray(frame, dtype=np.uint8), detection, camera)
            frames.append(
                {
                    "t": round(index / fps, 3),
                    "lane_width_m": detection.lane_width_m,
                    "curvature_radius_m": detection.curvature_radius_m,
                    "center_offset_m": detection.center_offset_m,
                    "left_boundary_m": detection.left_boundary_m,
                    "right_boundary_m": detection.right_boundary_m,
                }
            )
        except DetectionFailed:
            frames.append({"t": round(index / fps, 3), "detected": False})
        if writer is None:
            writer = cv2.VideoWriter(
                str(out / "annotated.mp4"),
                cv2.VideoWriter.fourcc(*"mp4v"),
                fps,
                (camera.frame_width, camera.frame_height),
            )
        writer.write(annotated)
        index += 1

    capture.release()
    if writer is not None:
        writer.release()
    (out / "lanes.json").write_text(
        json.dumps({"meters": True, "frames": frames}, indent=1), encoding="utf-8"
    )
    print(f"{detected}/{index} frames detected -> {out / 'lanes.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
