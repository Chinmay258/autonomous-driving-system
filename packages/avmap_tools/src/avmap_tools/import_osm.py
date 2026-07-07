"""CLI: fetch a real city extract from Overpass and bake the lane-level map.

    python -m avmap_tools.import_osm --bbox 41.388,2.158,41.398,2.172 \
        --out maps/barcelona_eixample --name barcelona-eixample

Writes <out>.osm (canonical Lanelet2 artifact), <out>_projector_info.yaml, and
caches the raw Overpass response next to the output so re-runs are offline.
"""

from __future__ import annotations

import argparse
import sys
import urllib.request
from pathlib import Path

from avmap_tools.lanelet2_io import write_osm, write_projector_info_yaml
from avmap_tools.osm_import import import_osm_roads
from avmap_tools.validation import validate_map

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
_QUERY = """
[out:xml][timeout:90];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|residential|unclassified)(_link)?$"]
     ({s},{w},{n},{e});
);
(._;>;);
out body;
"""


def fetch_overpass(bbox: tuple[float, float, float, float], cache_file: Path) -> str:
    if cache_file.exists():
        return cache_file.read_text(encoding="utf-8")
    s, w, n, e = bbox
    query = _QUERY.format(s=s, w=w, n=n, e=e)
    request = urllib.request.Request(
        OVERPASS_URL,
        data=query.encode(),
        headers={"User-Agent": "avmap_tools/0.1 (portfolio project)"},
    )
    with urllib.request.urlopen(request, timeout=120) as response:
        text: str = response.read().decode("utf-8")
    cache_file.parent.mkdir(parents=True, exist_ok=True)
    cache_file.write_text(text, encoding="utf-8")
    return text


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bbox", required=True, help="south,west,north,east (WGS84)")
    parser.add_argument("--out", required=True, help="output path prefix (no extension)")
    parser.add_argument("--drive-on", choices=["right", "left"], default="right")
    parser.add_argument(
        "--single-lane",
        action="store_true",
        help="one lane per direction (no lane changes; for the Autoware sim)",
    )
    args = parser.parse_args(argv)

    bbox = tuple(float(v) for v in args.bbox.split(","))
    if len(bbox) != 4:
        parser.error("--bbox needs exactly 4 comma-separated numbers")
    out = Path(args.out)

    raw = fetch_overpass(
        (bbox[0], bbox[1], bbox[2], bbox[3]), out.parent / f"{out.name}.overpass.xml"
    )
    lanelets, origin = import_osm_roads(raw, drive_on=args.drive_on, single_lane=args.single_lane)
    report = validate_map(lanelets)
    print(f"lanelets: {report.lanelet_count}; issues: {list(report.issues) or 'none'}")
    report.raise_if_failed()

    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".osm").write_text(write_osm(lanelets, origin), encoding="utf-8")
    (out.parent / f"{out.name}_projector_info.yaml").write_text(
        write_projector_info_yaml(origin), encoding="utf-8"
    )
    print(f"wrote {out.with_suffix('.osm')} (origin {origin.lat:.6f}, {origin.lng:.6f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
