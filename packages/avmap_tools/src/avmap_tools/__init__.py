"""avmap_tools — everything that produces or checks the canonical map artifact.

Pure stdlib + avcore. The Linux-only `lanelet2` package is used only as an
extra validation gate in CI/WSL (P4); nothing here depends on it.
"""

from avmap_tools.graphbuild import build_graph
from avmap_tools.lanelet2_io import read_osm, write_osm, write_projector_info_yaml
from avmap_tools.projection import LocalProjector
from avmap_tools.synthetic import synthetic_town
from avmap_tools.validation import ValidationReport, validate_map

__all__ = [
    "LocalProjector",
    "ValidationReport",
    "build_graph",
    "read_osm",
    "synthetic_town",
    "validate_map",
    "write_osm",
    "write_projector_info_yaml",
]
