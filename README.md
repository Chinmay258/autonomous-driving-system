# Autonomous Vehicle Driving System

Lane detection → Lanelet2 map conversion → route filtering → A* route planning →
validated in the **Autoware Planning Simulator** — plus a **live web demo** where
you drop two markers on a map of Monaco and watch a simulated vehicle plan the
route and drive it to arrival.

**Docs:** [SPEC.md](SPEC.md) (scope & feasibility) · [ARCHITECTURE.md](ARCHITECTURE.md)
(enterprise architecture, contracts, phased plan) · [docs/adr/](docs/adr/) (decisions)

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
packages/avcore/          # map model · routing graph · filtering · A* (the shared brain)
apps/webdemo/backend/     # wire contracts now; FastAPI + simulator in P5
apps/webdemo/frontend/    # MapLibre UI (P6)
apps/autoware_eval/       # pinned Autoware planning-sim setup + EVAL protocol (P4)
packages/avlane/          # lane detection: classical CV + UFLD-ONNX comparison (P3)
packages/avmap_tools/     # polyline→Lanelet2, OSM→Lanelet2, validators (P2)
docs/adr/                 # architecture decision records
infra/                    # compose + cloudflared, fly.toml stub (P7)
```

## Run the demo locally

```bash
uv run uvicorn webdemo_backend.app:create_app --factory --port 8017
# open http://localhost:8017 — click two points on the road network, hit Drive
```

## Status
| Phase | State |
|---|---|
| P0 foundation (workspace, CI, contracts) | ✅ done |
| P1 planning core (A*, cost models, filtering; Hypothesis-verified vs Dijkstra) | ✅ done |
| P2 map tooling (Lanelet2 OSM IO, graph derivation, validation, synthetic town) | ✅ done |
| P5 backend (FastAPI /plan + /ws/drive, bicycle sim + pure pursuit) | ✅ done |
| P6 frontend (MapLibre UI: markers → route → live drive telemetry) | ✅ done (zero-build) |
| P3 lane detection · P4 Autoware eval (WSL2) · P7 deploy (Cloudflare Tunnel) | ⏭ next |

91 tests · ~96% coverage · mypy --strict · deterministic planner.
See [ARCHITECTURE.md](ARCHITECTURE.md) §16 for acceptance criteria per phase.
