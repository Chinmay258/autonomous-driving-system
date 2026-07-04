# Autonomous Vehicle Driving System — Implementation Specification

**Goal:** Rebuild the pipeline — *lane detection → Lanelet2 map conversion → route filtering → route planning → evaluation in Autoware* — and additionally ship a **publicly deployable live demo** where visitors drop start/destination markers on a map, a planner computes a route, and a simulated agent visibly drives it to the destination.

**Target hardware (verified):** AMD Ryzen 7 5800H (8C/16T), 16 GB RAM, NVIDIA RTX 3050 Laptop (4 GB VRAM), ~43 GB free disk, Windows 11 + WSL2 (Ubuntu already installed).

---

## 1. Feasibility verdict for your hardware

| Task | Feasible? | Notes |
|---|---|---|
| Autoware **Planning Simulator** (rviz-based, no sensor sim) | ✅ Yes | This is what "evaluated on Autoware simulator" means in practice. CPU-bound, runs fine on 8 cores / 16 GB. Use the prebuilt Docker image — do **not** build Autoware from source (source build wants 32 GB+ RAM / 60 GB+ disk). |
| Lane detection (classical CV) | ✅ Yes | Real-time on CPU. |
| Lane detection (deep learning, e.g. UFLD/CLRNet inference) | ✅ Yes | RTX 3050 4 GB handles *inference* easily. Training from scratch is out; use pretrained weights. |
| AWSIM / CARLA end-to-end sensor simulation | ⚠️ Marginal | AWSIM wants ~8 GB VRAM. CARLA in low-quality mode may run at low FPS. Treat as stretch goal, not core. |
| **Live public deployment of Autoware itself** | ❌ No (by design) | Autoware is a ROS 2 desktop stack — you can't serve it per-visitor on a website. The live demo is a separate lightweight web service that reimplements the planning/control layer (Section 4). This is the standard approach and is what the spec designs for. |
| Disk budget | ⚠️ Tight | Autoware Docker image + maps ≈ 25–30 GB. With 43 GB free, clean up first or move the Docker/WSL data root. Target ≥ 35 GB free before pulling. |

**Architecture consequence:** the project splits into two tracks that share the same map format and planning algorithms:

- **Track A — Research pipeline (local, WSL2):** the faithful replication — lane detection, Lanelet2 conversion, route filtering/planning, validated in Autoware Planning Simulator. Output: code + screen recordings + metrics.
- **Track B — Live portfolio demo (cloud, always-on):** a FastAPI + MapLibre web app where anyone places two markers; the backend plans a route over a lanelet-style road graph and a simulated vehicle (kinematic bicycle + pure-pursuit controller) follows it live over WebSocket. Embeds Track A's recordings for credibility.

---

## 2. Repository layout

```
autonomous-driving/
├── SPEC.md
├── lane_detection/            # Track A, stage 1
│   ├── detect.py               # CLI: video/image → lane polylines (JSON/GeoJSON)
│   ├── pipeline/                # calibration, thresholding, birdseye, fit
│   └── models/                  # optional: pretrained UFLD weights
├── lanelet_conversion/         # Track A, stage 2
│   ├── polyline_to_lanelet.py   # lane polylines → Lanelet2 .osm
│   └── osm_to_lanelet.py        # OSM road extract → Lanelet2 .osm (for real-world maps)
├── route_planning/             # Track A, stages 3–4 (shared with Track B)
│   ├── graph.py                 # routing graph from Lanelet2 map
│   ├── filtering.py             # route/lanelet filtering rules
│   ├── planner.py               # A* / Dijkstra shortest-route
│   └── tests/
├── autoware_eval/              # Track A, stage 5
│   ├── docker-compose.yml       # pinned Autoware planning-sim image
│   ├── maps/                    # your generated .osm + pointcloud stub
│   └── EVAL.md                  # scenarios, metrics, results
├── webdemo/                    # Track B
│   ├── backend/
│   │   ├── main.py              # FastAPI: /plan, /ws/drive
│   │   ├── simulator.py         # kinematic bicycle model
│   │   ├── controller.py        # pure pursuit + speed profile
│   │   └── mapdata/             # prebaked road graph (from route_planning/)
│   ├── frontend/                # Vite + MapLibre GL JS
│   └── Dockerfile
└── .github/workflows/deploy.yml
```

---

## 3. Track A — Research pipeline (the replication)

### 3.1 Lane detection script

**Input:** dashcam-style video or image sequence (e.g. TuSimple/CULane clips, or the Udacity advanced-lane-finding video).
**Output:** per-frame lane boundary polylines in image space + bird's-eye (metric) space, serialized to JSON.

Primary implementation — classical CV (fully explainable in a portfolio writeup):
1. Camera calibration (chessboard, `cv2.calibrateCamera`) → undistort.
2. Color/gradient thresholding: HLS S-channel + Sobel-x magnitude → binary mask.
3. Perspective transform to bird's-eye view (fixed trapezoid → rectangle homography).
4. Sliding-window histogram search → 2nd-order polynomial fit per lane line; low-pass filter across frames; sanity checks (parallelism, curvature agreement, lane width 3.2–4.0 m).
5. Compute curvature radius + lateral offset; inverse-warp overlay for the demo video.

Optional secondary: run pretrained **Ultra-Fast-Lane-Detection v2** (ONNX Runtime, CUDA) on the same clips and compare F1/latency — one table, strong portfolio signal.

**Deliverables:** annotated output video, `lanes.json` (polylines in meters, ego frame), metrics (fps on your CPU/GPU, detection rate).

### 3.2 Lanelet conversion script

Two converters, same output format (**Lanelet2 OSM XML**):

- `polyline_to_lanelet.py` — takes `lanes.json`, pairs adjacent lane boundaries into lanelets: each lanelet = left bound polyline + right bound polyline as OSM `way`s of `node`s, tagged `type=lanelet`, `subtype=road`, `location=urban`, `one_way=yes`, `speed_limit`. Assigns sequential IDs, links successive lanelets by shared endpoints. Projects local metric coords to lat/lon with a `UTM`/`local` projector anchored at a chosen origin (Autoware needs a georeferenced map + matching `map_projector_info`).
- `osm_to_lanelet.py` — pulls a small real-world road network (OSMnx, ~1 km² of a city you choose) and synthesizes lanelet pairs from road centerlines + lane counts. This produces the bigger, more interesting map used for route planning and the web demo.

Validate with the `lanelet2` Python package (`pip install lanelet2`, Linux/WSL): load map, run `lanelet2.routing.RoutingGraph` construction — if the graph builds and is connected, the conversion is sound. Also visually inspect in **Vector Map Builder** (Tier IV's free web tool) — good screenshot material.

### 3.3 Route filtering script

Prunes the raw lanelet set before/during planning:
- Drop lanelets failing geometric sanity (width < 2.5 m or > 6 m, self-intersecting bounds, length < 1 m).
- Drop lanelets disconnected from the largest strongly-connected component of the routing graph.
- Attribute filters: exclude `subtype != road` (crosswalks, shoulders), respect `one_way`, optional turn-restriction handling.
- Post-planning filter: given N candidate routes, discard those violating constraints (max route curvature vs. vehicle steering limit, number of lane changes, U-turn bans), then rank by cost.

### 3.4 Route planning algorithm

Implement **A\*** over the lanelet routing graph (write it yourself — that's the portfolio piece — and cross-check against `lanelet2.routing`'s built-in shortest path):
- **Nodes:** lanelets. **Edges:** successor (longitudinal), left/right adjacent (lane change, higher cost).
- **Edge cost:** lanelet centerline length ÷ speed limit (travel time) + lane-change penalty (e.g. +5 s).
- **Heuristic:** straight-line distance from lanelet end to goal ÷ max speed (admissible).
- **Output:** ordered lanelet ID sequence + stitched centerline polyline (this is exactly the format Autoware's mission planner produces, and what the web demo's controller consumes).
- Unit tests: known small graphs, unreachable goal, start == goal, lane-change necessity.

### 3.5 Autoware evaluation (WSL2)

Setup (~1 evening):
1. In WSL2 Ubuntu: install Docker Engine + `nvidia-container-toolkit` (optional; planning sim is fine CPU-only). Ensure WSL config gives it ≥ 12 GB RAM (`.wslconfig`).
2. Pull the prebuilt image: `ghcr.io/autowarefoundation/autoware:universe` (pin a 2025 release tag). WSLg gives you GUI (rviz2) out of the box on Windows 11.
3. Smoke test with the official `sample-map-planning` map:
   `ros2 launch autoware_launch planning_simulator.launch.xml map_path:=... vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit`
4. Swap in **your** map: `map_path` pointing at your generated `lanelet2_map.osm` (+ a dummy `pointcloud_map.pcd` — planning sim tolerates a sparse/flat one) and `map_projector_info.yaml`.
5. In rviz: **2D Pose Estimate** → init pose on your map; **2D Goal Pose** → goal. Autoware's mission planner routes over *your lanelets*; the dummy-kinematics vehicle follows it.

**Evaluation protocol (write results in `EVAL.md`):**
- 5+ scenarios: short hop, cross-map route, route requiring lane change, unreachable goal, goal behind ego.
- Compare Autoware's chosen lanelet sequence vs. your own A* planner's output on the same map (should match or differ explainably by cost weights).
- Metrics: route length, planning latency, lateral deviation of the followed path from centerline (from `/localization/kinematic_state` vs. route), success/failure per scenario.
- Record everything (OBS on Windows captures the WSLg rviz window) — these clips go on the portfolio page.

---

## 4. Track B — Live deployable demo

**User experience:** visitor opens your site → sees a real city map → clicks a start marker and a destination marker → route draws itself → a car icon drives the route with realistic steering, slowing for curves, until it reaches the destination. A side panel shows live telemetry (speed, heading, cross-track error) and links to the Autoware videos + GitHub.

### 4.1 Architecture

```
Browser (MapLibre GL JS + OSM raster/vector tiles)
   │  REST  POST /plan {start:[lat,lng], goal:[lat,lng]}
   │  ◄── {route: GeoJSON LineString, lanelet_ids, eta}
   │  WS    /ws/drive?route_id=...
   │  ◄── 10 Hz: {lat, lng, heading, speed, cte, progress}
   ▼
FastAPI backend (single container)
   ├── mapdata: prebaked routing graph (pickled, built by route_planning/ from OSM)
   ├── planner.py  ← same A* module as Track A (shared package!)
   ├── controller.py: pure pursuit (lookahead 6–12 m, speed-scheduled)
   └── simulator.py: kinematic bicycle, dt=0.1 s, v≤ speed limit,
                     curvature-based slowdown, arrival when d(goal)<3 m
```

Key decisions:
- **Same planner code in both tracks** — `route_planning/` is a pip-installable package imported by the web backend. This makes the claim "the live demo runs my actual planning algorithm" literally true.
- Vehicle simulation runs **server-side** (authoritative, cheat-proof, and lets you show real controller telemetry); the frontend only renders. Multiple concurrent visitors = multiple lightweight sim tasks (asyncio); a bicycle model at 10 Hz costs ~nothing.
- Map: pick one fixed, recognizable area (e.g. 2–3 km² of your city or Monaco). Prebake the graph at build time; snap user clicks to nearest routable lanelet, reject clicks outside coverage with a friendly message.
- Frontend: Vite + TypeScript + MapLibre GL JS; free OSM tiles (or MapTiler free tier); car icon rotated by heading; route as animated GeoJSON layer; breadcrumb trail behind the car.

### 4.2 Deployment

- One Docker image (backend serves the built frontend as static files). ~200 MB.
- Host: **Fly.io / Render / Railway free-or-hobby tier** — a shared-CPU 512 MB instance is ample. WebSockets supported on all three. Custom domain + HTTPS included.
- CI: GitHub Actions — on push to `main`: run planner unit tests → build image → deploy.
- No GPU, no ROS, no per-user cost. This is why Track B exists instead of "Autoware in the cloud" (which would need a GPU VM + VNC streaming ≈ $100+/mo and single-user).

### 4.3 Nice-to-haves (ordered by portfolio value ÷ effort)

1. Telemetry panel with a live cross-track-error sparkline (shows you understand control, not just planning).
2. Toggle: A* vs. Dijkstra vs. travel-time cost — route visibly changes.
3. "Watch it in Autoware" section embedding your rviz recordings side-by-side with the web sim on the same map.
4. Obstacle mode: visitor drops a roadblock → replan mid-drive (reuses route filtering!).
5. Lane-detection playground: upload/choose a clip, see the CV pipeline stages (run server-side, classical CV only — cheap).

---

## 5. Tech stack summary

| Layer | Choice | Why |
|---|---|---|
| Lane detection | Python, OpenCV (+ ONNX Runtime for UFLD option) | CPU real-time, explainable |
| Map format | Lanelet2 OSM XML | Native Autoware format; single source of truth |
| Map tooling | `lanelet2` pip pkg, OSMnx, Vector Map Builder | validate / generate / inspect |
| Planning | Own A* + `lanelet2.routing` cross-check | portfolio core |
| Simulation (local) | Autoware Planning Simulator via Docker on WSL2 | matches original project |
| Simulation (web) | Kinematic bicycle + pure pursuit, asyncio @10 Hz | deployable, cheap |
| Backend | FastAPI + WebSockets | async, minimal |
| Frontend | Vite + TS + MapLibre GL JS | free tiles, smooth animation |
| Deploy | Docker → Fly.io/Render, GitHub Actions CI | free/hobby tier, HTTPS |

## 6. Milestones (evenings/weekends pace)

1. **Wk 1:** Lane detection classical pipeline + demo video + `lanes.json`.
2. **Wk 2:** Lanelet2 converters; map validates in `lanelet2` + Vector Map Builder.
3. **Wk 3:** Routing graph + filtering + A*; unit tests green.
4. **Wk 4:** Autoware planning sim on WSL2 with your map; run the 5 scenarios; record videos; write `EVAL.md`.
5. **Wk 5:** Web backend (plan + drive endpoints, sim/controller) working locally.
6. **Wk 6:** Frontend, polish, deploy, custom domain, README with architecture diagram + GIFs.

## 7. Risks & mitigations

- **Disk (43 GB free):** free up to ≥ 35 GB before pulling Autoware image; or move Docker data root / WSL vdisk to another drive; prune aggressively (`docker system prune`).
- **RAM (16 GB):** never build Autoware from source; cap WSL at 12 GB; close browsers while running rviz.
- **Custom map won't load in Autoware:** commonest causes are missing `map_projector_info.yaml`, non-monotonic node ordering, or unlinked lanelet successors — the `lanelet2` Python validation step in 3.2 catches these before Autoware ever sees the map.
- **Free-tier host sleeping:** Render free tier cold-starts (~30 s). Fly.io hobby or a $5 VPS avoids the embarrassing first-load stall for recruiters.
