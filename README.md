# Autonomous Vehicle Driving System

[![CI](https://github.com/Chinmay258/autonomous-driving-system/actions/workflows/ci.yml/badge.svg)](https://github.com/Chinmay258/autonomous-driving-system/actions/workflows/ci.yml)
[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)
[![Checked with mypy](https://img.shields.io/badge/mypy-strict-2a6db2.svg)](https://mypy-lang.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Lane detection → Lanelet2 map conversion → route filtering → A\* route planning →
validated in the **Autoware Planning Simulator** — plus a **live web demo** on a
**real city map (Barcelona's Eixample, imported lane-by-lane from OpenStreetMap)**:
drop two markers and a simulated vehicle plans a lane-level route and drives it,
with strict lane discipline — right turns only from the rightmost lane, left
turns only from the leftmost, and lane changes before every turn.

> **▶ Live demo:** open the deployed app (link in the repository **About** panel),
> click a start and a destination on the map, and watch the vehicle drive the route.

**Docs:** [SPEC.md](SPEC.md) (scope & feasibility) · [ARCHITECTURE.md](ARCHITECTURE.md)
(architecture, contracts, phased plan) · [docs/PROJECT_REPORT.md](docs/PROJECT_REPORT.md)
(detailed working report) · [docs/adr/](docs/adr/) (design decisions)

## Design in one line
One shared planning core (`packages/avcore`, pure Python, mypy-strict) is
imported by both the Autoware evaluation and the deployed demo — the public demo
literally runs the algorithm validated in simulation (ADR-0002).

## Quickstart (reproducibility contract)
Requires [uv](https://docs.astral.sh/uv/). From the repo root:

```bash
make setup    # uv sync --all-packages
make check    # lint + mypy --strict + tests + coverage gate
```

Windows without make: `uv sync --all-packages && uv run pytest`.

## Repository layout
```
packages/avcore/          # map model · routing graph · filtering (SCC) · A* (the shared brain)
packages/avmap_tools/     # OSM→Lanelet2 lane-level import, polyline→lanelets, IO, validators
packages/avlane/          # camera lane detection: threshold→BEV→sliding-window fit, metric output
apps/webdemo/backend/     # FastAPI /plan + /ws/drive, vehicle sim, zero-build MapLibre frontend
apps/autoware_eval/       # headless Autoware golden evaluation (EVAL.md has results)
docs/adr/                 # architecture decision records
infra/                    # compose + cloudflared, fly.toml stub
maps/                     # canonical Lanelet2 artifacts (Autoware-validated)
```

## Run the demo locally

```bash
AV_MAP_PATH=maps/barcelona_eixample.osm uv run uvicorn webdemo_backend.app:create_app --factory --port 8017
# open http://localhost:8017 — click two points on the map, hit Drive
# (omit AV_MAP_PATH to fall back to the synthetic test town)
```

Import a different city (any bbox, right- or left-hand traffic):

```bash
uv run python -m avmap_tools.import_osm --bbox "41.385,2.158,41.398,2.174" --out maps/mycity
```

## Status
| Phase | State |
|---|---|
| P0 foundation (workspace, CI, contracts) | ✅ done |
| P1 planning core (A*, cost models, filtering; Hypothesis-verified vs Dijkstra) | ✅ done |
| P2 map tooling (Lanelet2 OSM IO, graph derivation, validation, synthetic town) | ✅ done |
| P5 backend (FastAPI /plan + /ws/drive, bicycle sim + pure pursuit) | ✅ done |
| P6 frontend (MapLibre UI: markers → route → live drive telemetry) | ✅ done (zero-build) |
| P2b real-city map (OSM lane-level import, strict lane discipline, Barcelona artifact) | ✅ done |
| P7 deploy (Docker + Cloudflare Tunnel from always-on host, fly.toml migration stub) | ✅ done |
| P4 Autoware eval (headless golden test: strict route match on the same artifact) | ✅ done — [EVAL.md](apps/autoware_eval/EVAL.md) |
| P3 lane detection (OpenCV pipeline, CI-verified vs synthetic ground truth) | ✅ done |

91 tests · ~96% coverage · mypy --strict · deterministic planner.
See [ARCHITECTURE.md](ARCHITECTURE.md) §16 for acceptance criteria per phase.
