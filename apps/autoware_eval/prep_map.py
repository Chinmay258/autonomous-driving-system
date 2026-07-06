"""Prepare the Autoware map directory from the canonical artifact.

Produces apps/autoware_eval/mapdir/ with:
- lanelet2_map.osm            (copy of the validated artifact)
- map_projector_info.yaml     (LocalCartesianUTM anchored at the artifact origin)
- pointcloud_map.pcd          (minimal dummy cloud; planning sim needs a file)

Run from the repo root:  uv run python apps/autoware_eval/prep_map.py
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

from avmap_tools import read_osm

REPO = Path(__file__).resolve().parents[2]
ARTIFACT = REPO / "maps" / "barcelona_eixample.osm"
MAPDIR = Path(__file__).resolve().parent / "mapdir"

DUMMY_PCD = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 4
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4
DATA ascii
0 0 0
50 0 0
0 50 0
-50 -50 0
"""


def main() -> int:
    lanelets, origin = read_osm(ARTIFACT.read_text(encoding="utf-8"))
    MAPDIR.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ARTIFACT, MAPDIR / "lanelet2_map.osm")
    (MAPDIR / "map_projector_info.yaml").write_text(
        "projector_type: LocalCartesianUTM\n"
        "vertical_datum: WGS84\n"
        "map_origin:\n"
        f"  latitude: {origin.lat}\n"
        f"  longitude: {origin.lng}\n"
        "  altitude: 0.0\n",
        encoding="utf-8",
    )
    (MAPDIR / "pointcloud_map.pcd").write_text(DUMMY_PCD, encoding="utf-8")
    print(f"mapdir ready: {MAPDIR} ({len(lanelets)} lanelets, origin {origin.lat}, {origin.lng})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
