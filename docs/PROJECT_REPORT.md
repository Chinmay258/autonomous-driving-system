# Autonomous Vehicle Driving System — Working Report

**Author:** Chinmay Singh · **Repository:** https://github.com/Chinmay258/autonomous-driving-system
**Period:** 2026-07-05 → 2026-07-07 · **Report generated:** 2026-07-07

---

## 1. Executive summary

A complete autonomous-driving route-planning system was built, tested, deployed, and
validated in the Autoware planning simulator. The system takes a user-selected start and
destination on a real city map (Barcelona's Eixample district, imported lane-by-lane from
OpenStreetMap), plans a lane-level route with an own A\* planner, and drives a simulated
vehicle along it to arrival.

Two things run off **one shared planning core**:

1. **A live public web demo** — visitors drop two markers, the backend plans with the same
   A\* code, and a server-side vehicle simulator drives the route over WebSocket at 10 Hz.
   Live now at a Cloudflare Tunnel URL, running on the author's laptop.
2. **An Autoware validation track** — the identical Lanelet2 map loads in the real Autoware
   stack; its mission planner's routes were compared against ours (golden test), and the
   ego vehicle was driven to **ARRIVED** in rviz on the Windows desktop.

**Headline results**

| Metric | Value |
|---|---|
| Git commits | 20, phased P0→P7 + iterative fixes |
| Python files | 47 (source + tests) |
| Source LOC | avcore 796 · avmap_tools 1241 · avlane 381 · webdemo backend 607 |
| Tests | **144 passing**, 1 expected warning |
| Coverage | **90.08%** (gate: 80%), `mypy --strict` clean on 26 source files |
| Map (web) | 2545 lanelets, 1447 turn connectors, 1398 lane-change edges, strong core 2117 |
| Map (Autoware sim) | 1144 lanelets, 0 lane-change edges (single-lane), strong core 958 |
| Autoware golden test | short-hop **strict lanelet match**; cross-map **1.0 road-level coverage** |
| Autoware drive | Vehicle reached **ARRIVED** (route_state 3) on a 21-lanelet multi-turn route |
| Live demo | Up 23 h, healthy, 2117 routable lanelets |

---

## 2. Original brief and how it was met

The source project description was:

> *"Implemented Lane detection script, Lanelet conversion script, route filtering script and
> route planning algorithm. Evaluated created route on Autoware simulator."*

Plus the author's added goal: a **deployable live version** where people place location
markers and an agent finds and follows a path to the destination, and confirmation that it
can be **simulated on Autoware on the author's own hardware**.

| Brief element | Delivered as | Status |
|---|---|---|
| Lane detection script | `packages/avlane` — classical CV pipeline, metric output | ✅ |
| Lanelet conversion script | `avmap_tools.polylines_to_lanelets` + full OSM→Lanelet2 importer | ✅ |
| Route filtering script | `avcore.filtering` — sanity + connectivity (weak/strong SCC) filters | ✅ |
| Route planning algorithm | `avcore.planner` — A\* with pluggable cost models | ✅ |
| Evaluated on Autoware simulator | `apps/autoware_eval` — golden route comparison + visual drive to ARRIVED | ✅ |
| Live deployable marker demo | `apps/webdemo` — FastAPI + MapLibre, live on Cloudflare Tunnel | ✅ |
| Runs on author's hardware | Verified end-to-end on Ryzen 7 5800H / 14 GB / RTX 3050 via WSL2 | ✅ |

---

## 3. Target hardware and feasibility (measured, not assumed)

| Component | Spec |
|---|---|
| CPU | AMD Ryzen 7 5800H — 8 cores / 16 threads |
| RAM | ~14 GB usable (WSL2 VM shares this) |
| GPU | NVIDIA RTX 3050 Laptop, 4 GB VRAM |
| OS | Windows 11 + WSL2 (Ubuntu), Docker Desktop |
| Autoware image | `ghcr.io/autowarefoundation/autoware:latest-runtime`, **8.81 GB** |

**Feasibility conclusion:** The Autoware *planning simulator* (no heavy sensor rendering)
runs on this machine. It was run headless (4 GB / 6 CPU cap for evaluation) and with rviz
(5.5 GB / 14 CPU for the visual). Peak container memory during evaluation was ~2.9 GB.
The full Autoware source build (needs 32 GB+ RAM) was correctly avoided in favour of the
prebuilt image.

---

## 4. Architecture

### 4.1 The core idea — one brain, two bodies

All planning intelligence lives in `packages/avcore`, a pure-Python package with **zero**
dependencies on web frameworks, ROS/Autoware, or OpenCV. Both the web demo and the Autoware
track import it. This is what makes the claim "the live demo runs the algorithm validated in
Autoware" literally true, not marketing. (Recorded as ADR-0002.)

```
                    ┌──────────────────────────┐
                    │  packages/avcore          │
                    │  models · graph · A*      │
                    │  filtering · routeops     │  ← the shared brain (pure Python)
                    └───────┬──────────┬────────┘
                            │          │
              ┌─────────────┘          └──────────────┐
              ▼                                        ▼
   ┌──────────────────────┐              ┌──────────────────────────┐
   │ apps/webdemo          │              │ apps/autoware_eval        │
   │ FastAPI + MapLibre    │              │ headless golden test +    │
   │ vehicle sim @10 Hz    │              │ visual rviz drive         │
   │ → Cloudflare Tunnel   │              │ → real Autoware stack     │
   └──────────────────────┘              └──────────────────────────┘

   packages/avmap_tools : OSM → Lanelet2 import, IO, validation, single-lane variant
   packages/avlane      : camera lane detection (classical CV)
```

### 4.2 Key architectural decisions (ADRs)

- **ADR-0001** — Lanelet2 OSM XML is the single canonical map format; everything derives
  from it (Autoware loads it directly; the demo prebakes a routing graph from it).
- **ADR-0002** — One shared `avcore` planning core in a uv-workspace monorepo.
- **ADR-0003** — Host on Cloudflare Tunnel from the always-on laptop first; the demo is one
  12-factor Docker image so migrating to Fly.io is `fly launch` + DNS repoint.

---

## 5. Repository structure

```
autonomous-driving/
├── packages/
│   ├── avcore/           # 796 LOC src — models, RoutingGraph, A*, filtering, routeops
│   ├── avmap_tools/      # 1241 LOC src — OSM import, Lanelet2 IO, graph build, validation
│   └── avlane/           # 381 LOC src — lane detection pipeline + synthetic ground truth
├── apps/
│   ├── webdemo/backend/  # 607 LOC src — FastAPI /plan + /ws/drive, MapService, simulator
│   ├── webdemo/…/static/ # zero-build MapLibre frontend (index.html)
│   └── autoware_eval/    # golden route harness, prep_map, run_viz.ps1, EVAL.md, overrides/
├── maps/
│   ├── barcelona_eixample.osm       # 11 MB — full multi-lane map (web demo)
│   └── barcelona_eixample_sim.osm   # 5.8 MB — single-lane map (Autoware sim)
├── infra/                # docker-compose (demo + cloudflared), fly.toml
├── docs/adr/             # architecture decision records 0001–0003
├── Dockerfile · pyproject.toml · uv.lock · .github/workflows/ci.yml
```

Toolchain: **uv** workspace, **ruff** (lint+format), **mypy --strict**, **pytest** +
coverage gate (80% floor), **Hypothesis** property tests, GitHub Actions CI running the
exact same gate on every push.

---

## 6. Phase-by-phase work log (mapped to commits)

| Commit | Phase | Delivered | Result |
|---|---|---|---|
| `558f76d` | P0 | Monorepo scaffold, typed domain models, RoutingGraph, frozen REST/WS contracts, CI | 28 tests, 95% cov |
| `8cead22` | P1 | A\* planner, Distance/TravelTime cost models, filtering, Hypothesis property tests | A\* proven equal to reference Dijkstra on random grids |
| `6024c2c` | P2 | Lanelet2 OSM read/write, graph derivation, validation, synthetic town | 72 tests, deterministic OSM output |
| `04dcf90` | P5 | FastAPI backend: `/plan` + `/ws/drive`, MapService, kinematic-bicycle sim + pure pursuit | 89 tests |
| `62e94f6` | P6 | Zero-build MapLibre UI served by backend | Verified live drive-to-arrival in browser |
| `2af8862` | P7 | Dockerfile, compose + cloudflared, fly.toml; concurrent-drive cap | Public URL verified end-to-end (REST + WSS) |
| `71f22d9` | P2b | Real-city OSM lane-level import, strict lane discipline | Barcelona artifact, 2985 lanelets initial |
| `6f7d354` | fix | Click-exact route trimming, zigzag-free lane changes, curved connectors, U-turns, direction arrows | User-reported issues resolved |
| `4ec267e` | fix | Aggregated lane-change diagonals, cubic-fillet turns | Staggered-turn artifacts removed |
| `0fe5ea8`,`ca59be4` | P4 | Headless Autoware golden-route evaluation harness + orchestrator | Reproducible one-command eval |
| `d8d0b0f` | P4 | Golden evaluation run; map made natively lanelet2-routable | short-hop strict match |
| `932d7d6` | fix | Strongly-connected-component filter + P3 lane detection | "No route" made structurally impossible |
| `f31a2f1` | viz | rviz on Windows desktop via WSLg | Vehicle drives visibly |
| `fcbd6de`,`7bc2bc0`,`5f6bab8` | fix | Corner fillets, geometry densification, junction clustering, turn tags | Drive progression 0→66 m |
| `6582986`,`294cfc7` | fix | **Single-lane sim map + non-respawning rviz** | **Vehicle reaches ARRIVED** |

---

## 7. Component deep-dives

### 7.1 `avcore` — the planning core

- **models.py** — frozen dataclasses: `Point2D`, `Lanelet` (with validation, centerline,
  `length_m`, `mean_width_m`, `is_connector` flag), `RouteResult` (lanelet ids + stitched
  centerline + per-point speed limits + distance + ETA + lane-change count).
- **graph.py** — `RoutingGraph`, an adjacency-list digraph. Edge kinds: `SUCCESSOR`
  (longitudinal), `LEFT`/`RIGHT` (lane change). Typed errors on unknown/duplicate lanelets.
- **planner.py** — `plan_route`: A\* with deterministic tie-breaking `(f, g, id)` and
  admissible heuristics.
  - `Distance` cost: route length in metres; lane changes cost a small maneuver penalty.
  - `TravelTime` cost: length ÷ speed limit, + 5 s per lane change.
  - Geometry stitching turns a *run* of consecutive lane changes into one tangent-aligned
    cubic forward diagonal (fixes the zigzag artifact).
- **filtering.py** — `filter_lanelets`: keeps sane, drivable lanelets, then restricts to the
  largest **weakly** (default) or **strongly** connected component (Tarjan, `require_strong`).
  `rank_routes` with Menger-curvature and lane-change constraints.
- **routeops.py** — arclength slicing, nearest-point projection (first/last pass for
  U-turn-aware endpoints), and `trim_route` which cuts a route to the exact clicked
  start/goal positions.

**Verification:** Hypothesis property tests generate random speed grids and assert A\*'s
route cost equals a reference Dijkstra's — i.e. the planner is provably optimal, not just
"looks right."

### 7.2 `avmap_tools` — map production

- **osm_import.py** (373 LOC) — raw OpenStreetMap XML → per-lane lanelets. Splits ways at
  junction nodes, expands each segment into one lanelet per lane from `lanes`,
  `lanes:forward/backward`, `oneway`, `maxspeed` tags (with class defaults), right- or
  left-hand traffic. Builds intersection connectors with **strict lane discipline**:
  - straight movements preserve lane index (lane k → lane k),
  - right turns only rightmost→rightmost, left turns only leftmost→leftmost,
  - U-turns only from the leftmost lane onto the same street's opposite side,
  - connectors are tangent circular corner fillets (target radius 5 m, floor 2.5 m),
  - junction clustering (25 m) merges chamfered corners into one logical intersection,
  - `single_lane=True` clamps every street to one lane per direction (Autoware sim map).
- **lanelet2_io.py** — Lanelet2 OSM writer/reader. Deduplicates shared divider ways, writes
  `subtype=dashed` on internal same-direction dividers (so lanelet2 permits lane changes),
  writes `turn_direction` and `turn_connector` tags Autoware expects, snaps connector bound
  endpoints onto exact street nodes (so lanelet2 succession works).
- **graphbuild.py** — derives the RoutingGraph from geometry (successors by endpoint match,
  lateral edges by shared divider polylines).
- **validation.py** — duplicate-id, width/length sanity, dead-end ratio gate; a map must
  pass before any consumer loads it.
- **from_polylines.py** — detected lane boundaries → lanelets (closes the
  detection→conversion loop from the original brief).

### 7.3 `avlane` — camera lane detection

Classical, fully explainable pipeline: HLS + Sobel-x gradient threshold → perspective warp to
bird's-eye view → sliding-window search → 2nd-order polynomial fit → metric outputs (lane
width, curvature radius, centre offset, boundary polylines in metres). A synthetic road
renderer with **analytic ground truth** makes accuracy CI-verifiable without any dataset:
tests assert recovered width within ±0.25 m, injected lateral offset within ±0.15 m, and
curvature sign+magnitude. A CLI (`python -m avlane.detect`) processes real dashcam video into
`lanes.json` + an annotated MP4.

### 7.4 `apps/webdemo` — the live demo

- **Backend (FastAPI):** `MapService` owns the map, snaps clicks to the nearest routable
  lanelet (segment distance, 100 m radius), plans via `avcore`, trims to clicks, stores
  bounded route history. `POST /plan` returns GeoJSON + ETA; `WS /ws/drive` streams 10 Hz
  telemetry (position, heading, speed, cross-track error, progress) with pause/resume/cancel
  and a concurrent-drive cap.
- **Vehicle simulator:** kinematic bicycle model + pure-pursuit steering, curvature- and
  limit-aware speed control, smooth arrival stop, safety aborts on excessive cross-track
  error or time budget.
- **Frontend:** zero-build MapLibre page, dimmed OSM basemap, per-lane direction chevrons,
  route overlay, animated car with breadcrumb trail, live telemetry panel.

### 7.5 `apps/autoware_eval` — the Autoware track

- **prep_map.py** — bakes the map directory (Lanelet2 map + `LocalCartesianUTM` projector
  info + dummy pointcloud); prefers the single-lane sim map.
- **golden_route_check.py** — runs inside the container: loads the map with the matching
  projector, initialises pose, publishes the goal, captures Autoware's `LaneletRoute`.
- **compare_routes.py** — section-tolerant comparison of our A\* sequence vs Autoware's.
- **run_viz.ps1 / open_rviz.ps1** — one-command visual sim; rviz runs headless-then-separate
  so closing it does not respawn it.
- **overrides/** — `pid.param.yaml` (disables a standstill deadlock) and
  `lane_change.param.yaml` (short-block tuning), mounted over stock config.

---

## 8. Hard technical problems solved

Each of these was a real defect found and fixed, most surfaced by driving the actual systems.

1. **Plain uvicorn has no WebSocket support.** The `/ws/drive` endpoint failed silently under
   the test client but broke in a real browser. Fixed by `uvicorn[standard]`. *(Found by
   driving the real server, not tests.)*
2. **Zigzag lane changes.** A lane change re-entered the neighbour lane at its start, drawing
   a backward diagonal. Fixed by penalty-only lateral edge cost + forward-diagonal stitching;
   regression test asserts monotonic progress.
3. **Route overshooting the markers.** Routes were whole-lanelet; geometry ran past the click.
   Fixed by `trim_route` arclength projection (first-pass start, last-pass goal).
4. **Staggered/hooked turns.** Cubic control-point logic misbehaved on Barcelona's acute
   crossings. Replaced with tangent circular corner fillets (radius floor) — smooth for every
   angle including U-turns.
5. **"No route" between two clicks.** The map was only *weakly* connected — one-way stubs
   clipped at the bbox border were enterable but not exitable. Fixed by routing within the
   largest **strongly**-connected component (iterative Tarjan); a test plans between all pairs.
6. **Map wouldn't route in Autoware's lanelet2.** Succession is derived from *exact shared
   nodes*, and lane changes need *dashed* dividers. Fixed by snapping connector endpoints to
   street nodes and writing `subtype=dashed` on internal dividers; verified with lanelet2's
   own `RoutingGraph.getRoute` inside the container.
7. **Bowtie folds.** Offset bounds self-intersected when an arc radius dipped below the half
   lane width, and lanelet2 silently inverted them, breaking succession. Fixed by fold
   detection + teardrop U-turn arcs.
8. **rviz starved the control loop.** Software OpenGL (llvmpipe) needs ~3 cores; at the 6-CPU
   cap the 30 Hz control loop missed its rate monitors and latched a stop. Fixed by a 12+ CPU
   cap for the visual.
9. **Keep-stopped deadlock.** The longitudinal controller's
   `enable_keep_stopped_until_steer_convergence` never released at standstill (flag read once
   at boot). Fixed by a mounted `pid.param.yaml` override.
10. **Autonomous mode vetoed.** Teleporting via `/initialpose3d` leaves the ADAPI localization
    state UNINITIALIZED, so the component monitor blocked engagement. Fixed by the official
    `/api/localization/initialize` service.
11. **The car kept stopping (the big one).** Autoware's lane-change module could not execute
    the lane changes our multi-lane routes required on short city blocks — the reference path
    truncated to ~10 m and the vehicle held. Diagnosed step by step (route/localization/control
    all proven healthy; path chain only covered the current lanelet). **Fixed by generating a
    single-lane-per-direction sim map** (`single_lane=True`, zero lateral edges) so routes are
    pure succession + turns. The vehicle then drove a full 21-lanelet multi-turn route to
    ARRIVED.
12. **rviz kept respawning on close.** It was a node inside Autoware's launch graph. Fixed by
    launching the sim `rviz:=false` and running rviz as a separate process (`open_rviz.ps1`).

---

## 9. Results and metrics

### 9.1 Software quality

- **144 tests passing**, 90.08% coverage (80% gate), `mypy --strict` clean across 26 source
  files, ruff lint+format clean. CI green on GitHub Actions for every pushed commit.
- The two uncovered files (`avlane/detect.py`, `avmap_tools/import_osm.py`) are thin CLI
  wrappers exercised manually, not in unit tests.

### 9.2 Map artifacts

| Map | Lanelets | Connectors | Lane-change edges | Strong core |
|---|---|---|---|---|
| `barcelona_eixample` (web) | 2545 | 1447 | 1398 | 2117 |
| `barcelona_eixample_sim` (Autoware) | 1144 | 745 | **0** | 958 |

The zero lane-change edges in the sim map is the key structural property that makes Autoware
drive it reliably.

### 9.3 Autoware golden route comparison (our A\* vs Autoware's mission planner)

Run headless on the same Lanelet2 artifact, image `latest-runtime`, 4 GB / 6 CPU caps:

| Scenario | Ours | Autoware | Verdict |
|---|---|---|---|
| short-hop | 21 lanelets | 21 sections | **strict match — identical lane-for-lane sequence** |
| cross-map | 64 steps | 60 sections | same road-level route, **section coverage 1.0** |
| mid-range | 54 steps | 49 sections | same endpoints; diverges by cost model (our travel-time vs Autoware's length) — both valid |

The map loaded natively (`Succeeded to load lanelet2_map. Map is published.`), peak container
memory ~2.9 GB.

### 9.4 Visual drive (rviz on the Windows desktop)

| Map used | Outcome |
|---|---|
| Full multi-lane | Progressive stall as lane-change fixes landed: 0 → 15 → 25 → **66 m** |
| **Single-lane sim** | **Drove a 21-lanelet multi-turn route from spawn to goal, `route_state = 3` (ARRIVED)** — reached ~16 km/h, no stalls |

Verified twice (initial run and a repeat run at the user's request).

### 9.5 Live web demo

- Up 23 h, `healthz` OK, 2117 routable lanelets.
- Verified corner-to-corner routes in both directions through the public Cloudflare edge
  (e.g. 2.5 km / 4 lane changes; 2.1 km / 3 lane changes) plus live WSS drive telemetry.
- Idle container footprint: ~55 MiB / 0.15% CPU (demo) + ~15 MiB (tunnel), hard-capped so it
  never contends with other work on the machine.

---

## 10. Deployment

- **Image:** single 12-factor Docker image (`Dockerfile`), Barcelona map baked in, non-root
  user, healthcheck, all config via env vars.
- **Hosting:** `infra/docker-compose.yml` runs the demo + a `cloudflared` sidecar (a
  Cloudflare *quick tunnel*), both resource-capped. Public HTTPS/WSS with no port-forwarding
  and hidden origin IP.
- **Migration path:** committed `fly.toml` — moving to Fly.io is `fly launch --copy-config`
  + DNS repoint, same image, zero code change.
- **CI/CD:** GitHub Actions runs lint + mypy-strict + tests + coverage gate on every push.

---

## 11. Known limitations and honest open items

- The Autoware **visual drive uses the single-lane map**, which intentionally has no lane
  changes. Making Autoware execute lane changes on the *multi-lane* map (its `lane_change`
  module struggles on short Eixample blocks) remains an open research item, documented in
  `apps/autoware_eval/EVAL.md`. The web demo's own simulator handles lane changes fine.
- The Cloudflare **quick-tunnel URL changes on restart**; a permanent URL needs a named
  tunnel token (~5 min of dashboard setup).
- Aged Autoware containers (>30 min) develop flaky ROS 2 DDS discovery via `docker exec`; the
  workaround is a fresh container via `run_viz.ps1` (documented).
- Lane detection is validated against **synthetic** ground truth; running it on real dashcam
  footage is supported by the CLI but not part of the automated test suite.

---

## 12. How to run everything

```bash
# Quality gate (from repo root; requires uv)
uv sync --all-packages
uv run ruff check . && uv run mypy && uv run pytest      # 144 tests, 90% coverage

# Web demo locally
AV_MAP_PATH=maps/barcelona_eixample.osm \
  uv run uvicorn webdemo_backend.app:create_app --factory --port 8017
# → open http://localhost:8017, click two points, hit Drive

# Import a different city (any bbox; --single-lane for an Autoware-friendly map)
uv run python -m avmap_tools.import_osm --bbox "S,W,N,E" --out maps/mycity [--single-lane]

# Lane detection on real video
uv run python -m avlane.detect --input dashcam.mp4 --output-dir out/

# Autoware headless golden evaluation
powershell -File apps/autoware_eval/run_eval.ps1

# Autoware VISUAL drive (rviz on the Windows desktop) — auto-drives to arrival
powershell -File apps/autoware_eval/run_viz.ps1
powershell -File apps/autoware_eval/open_rviz.ps1     # reopen rviz any time
docker rm -f autoware_viz                              # teardown

# Deploy (laptop, Cloudflare Tunnel)
docker compose -f infra/docker-compose.yml up -d --build
docker logs av_tunnel | grep trycloudflare            # public URL
```

---

*Report reflects the repository state at commit `294cfc7` on 2026-07-07. All figures were
measured from the live repo and running systems, not estimated.*
