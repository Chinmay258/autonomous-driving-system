# Maps

| File | What it is |
|---|---|
| `barcelona_eixample.overpass.xml` | Raw OpenStreetMap extract (drivable highways) from the Overpass API, data snapshot 2026-07-06T08:25:59Z |
| `barcelona_eixample.osm` | Lanelet2 map generated from it: 2,545 lanelets, multi-lane, used by the web demo |
| `barcelona_eixample_projector_info.yaml` | Projection origin for the map (local Cartesian, 41.391464 N, 2.165375 E) |
| `barcelona_eixample_sim.overpass.xml` | Second fetch of the same area for the single-lane build, data snapshot 2026-07-07T19:20:30Z |
| `barcelona_eixample_sim.osm` | Single-lane variant (`--single-lane`): 1,144 lanelets, no lane changes, used for the Autoware drive |
| `barcelona_eixample_sim_projector_info.yaml` | Projection origin for the single-lane map |

## Reproducibility

The importer (`python -m avmap_tools.import_osm`) reads `<out>.overpass.xml` when it is
present instead of querying Overpass, and its output is deterministic. Regenerating from the
committed extracts reproduces both `.osm` files and both projector files byte for byte,
apart from line endings. Commands are in `docs/RECREATE.md`, section 3.

## Licence and attribution

Map data © OpenStreetMap contributors, available under the
[Open Database License (ODbL) 1.0](https://opendatacommons.org/licenses/odbl/). The raw
extracts and the Lanelet2 maps derived from them are distributed under the same licence. The
code in this repository is MIT-licensed (`LICENSE`); that licence does not apply to the data
in this directory.
