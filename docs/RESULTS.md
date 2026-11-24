# Measured results

Every number in the project report, the minor-project documents and the presentations, with
how it was measured. Nothing here is estimated. Re-run the scripts in `tools/eval/` to
reproduce them; absolute timings vary with the machine and its load, ratios and counts do not.

**Machine for local measurements:** AMD Ryzen 7 5800H (8 cores, 16 threads), 13.9 GB RAM,
Windows 11, Python 3.11.9 (dependency versions pinned in `uv.lock`).
**Deployment:** Fly.io app `av-route-demo`, region `bom` (Mumbai), one `shared-cpu-1x`
machine with 512 MB, auto-stop when idle (`fly.toml`).

## 1. Map artefacts

Script: `tools/eval/bench_planning.py` (section 1), counts from the committed files.

| Property | `barcelona_eixample.osm` (web demo) | `barcelona_eixample_sim.osm` (Autoware) |
|---|---|---|
| File size | 10.9 MB | 5.8 MB |
| Lanelets | 2,545 | 1,144 |
| Turn connectors | 1,447 (1,107 straight, 147 left, 193 right) | 745 |
| Nodes / ways written | 41,208 / 4,391 | - |
| Dashed dividers (lane change allowed) | 699 | 0 |
| Graph edges | 4,323 (1,398 lane-change) | successor and turn edges only |
| Largest weakly connected component | 2,535 | - |
| Largest strongly connected component | 2,117 lanelets (83.2 %), 3,600 edges (1,182 lateral) | 958 |
| Lanelets with no outgoing edge | 8 | - |
| Centre-line points per lanelet | mean 10.4, max 56 | - |
| Street lanelet length | median 44.4 m, longest 454.7 m | - |
| Projection origin | 41.391464 N, 2.165375 E | same |

Both maps regenerate **byte-identical** (ignoring line endings) from the committed raw
Overpass extracts in `maps/` — see `maps/README.md`.

## 2. Map build and service start-up

| Stage | Time |
|---|---|
| Parse Lanelet2 XML (`read_osm`) | 1.02 s |
| Validate (`validate_map`) | 0.091 s |
| Derive routing graph (`build_graph`) | 0.062 s |
| Largest weakly connected component | 0.032 s |
| Largest strongly connected component (iterative Tarjan) | 0.034 s |
| `MapService` construction, total | 1.21 s |
| Process start to first healthy `/healthz` (uvicorn) | 2.0 s |

Memory (tracemalloc): peak 173.6 MB while loading, 9.9 MB retained afterwards. The snapping
index holds 19,872 centre-line segments.

## 3. Planning latency

300 random start/goal pairs drawn from the strongly connected core (seed 42), after 20
warm-up plans, measured in-process.

| Operation | Median | p95 | p99 | Max |
|---|---|---|---|---|
| Snap both coordinates (linear scan) | 34.8 ms | 36.6 ms | 39.6 ms | 43.0 ms |
| A* search, travel-time model | 8.4 ms | 31.0 ms | 36.2 ms | 82.5 ms |
| End-to-end `plan()` (snap, A*, stitch, trim, store), travel time | 45.8 ms | 70.2 ms | 86.3 ms | 125.8 ms |
| End-to-end `plan()`, distance model | 42.0 ms | 63.9 ms | 68.4 ms | 108.7 ms |

Snapping is about three quarters of the median request. Planned routes: median 1,136 m
(longest 2,619 m), median 33 lanelets (max 75), median 3 lane changes (max 9). The per-call
scan for the map's maximum speed costs 0.16 ms.

## 4. A* against Dijkstra

150 real routes, each planned with the straight-line heuristic and with h = 0.

| Measure | A* | Dijkstra (h = 0) |
|---|---|---|
| Median nodes expanded | 359 | 1,004 |
| Maximum nodes expanded | 2,062 | 2,114 |
| Mean expansions ratio | 0.42 | 1 |
| Mean wall time | 11.1 ms | 10.6 ms |
| Routes with a different cost | 0 / 150 | reference |

A* expands 58 % fewer nodes but is not faster: `Lanelet.centerline` and `length_m` are
recomputed from the bounds on every access, and the profiler shows the centre-line generator
as the top self-time function (1.0 s of 2.98 s over 100 plans, 1.88 M `Point2D` constructions).
Caching length and end point per lanelet (safe: lanelets are frozen) brings A* to a mean of
1.58 ms (median 1.28 ms, p95 3.64 ms), 7.2 times faster with identical routes. This is a
what-if in the benchmark, not shipped code.

**Heuristic consistency (400 random pairs, A* cost vs Dijkstra cost):**
travel-time model optimal on 400 / 400. Distance model suboptimal on 227 / 400, by at most
4.04 m (0.34 %) in the original check (2026-09-13); `bench_planning.py` (seed 7, re-run
2026-11-25) gives 216 / 400, at most 3.29 m (0.54 %), with the travel-time model again
optimal on 400 / 400. Cause: a lane change costs 1.0 m but moves the lanelet end point about 3.5 m
sideways, so the straight-line estimate can drop by more than the edge cost (inconsistent
heuristic, worst slack 2.5 m), and the planner never reopens closed nodes
(`planner.py`, `if edge.target in closed`). Fix: a lane-change penalty of at least one lane
width, or allow reopening.

## 5. Vehicle simulation

`tools/eval/make_figures.py`: 80 random routes (seed 11), dt = 0.1 s.

| Measure | Value |
|---|---|
| Drives reaching the goal | 80 / 80 |
| Telemetry frames | 110,874 |
| Cross-track error | mean 0.141 m, median 0.016 m, p95 0.661 m, p99 1.566 m, max 8.696 m |

`tools/eval/bench_planning.py` (120 routes from the latency set): 120 / 120 arrived, CTE mean
0.149 m, p95 0.72 m, p99 1.71 m, max 8.76 m over about 160,000 frames; mean speed 30.9 km/h.
One simulator step costs 33.7 µs; one telemetry frame is 166 bytes and serialises in 6.5 µs.
The route store holds about 32.5 kB per route, about 33 MB at its 1,000-route cap.

## 6. Service throughput and concurrency

`tools/eval/load_test.py`, service under uvicorn on the development machine, 2026-11-11.

| Concurrent `POST /plan` | Median | p95 | Throughput | Errors |
|---|---|---|---|---|
| 1 | 44.7 ms | 65.7 ms | 20.94 req/s | 0 |
| 4 | 183.4 ms | 243.8 ms | 21.77 req/s | 0 |
| 16 | 771.0 ms | 951.5 ms | 19.86 req/s | 0 |
| 64 | 3.19 s | 3.22 s | 19.38 req/s | 0 |

Throughput is flat and latency grows with offered load: planning is CPU-bound Python in one
process behind the GIL. An earlier run on 2026-09-13, with the machine under other load,
measured the same shape at about 10 req/s (11.6 req/s at concurrency 1, p50 84.5 ms;
10.0 req/s at 16, p50 1.47 s; 10.1 req/s at 64, p50 6.2 s).

| Simultaneous drives | Telemetry rate | Interval p50 / p95 / p99 | Server CPU | Server memory |
|---|---|---|---|---|
| 1 | 9.03 Hz | 112.2 / 113.4 / 120.4 ms | 0.01 cores | 87.5 MB |
| 50 | 9.18 Hz | 110.5 / 116.5 / 119.5 ms | 0.09 cores | 95.6 MB |
| 200 | 8.97 Hz | 111.5 / 126.7 / 135.6 ms | 0.28 cores | 128.3 MB |
| 500 | 8.93 Hz | 112.1 / 130.1 / 136.4 ms | 0.68 cores | 176.3 MB |

The rate sits near 9 Hz instead of 10 Hz because the drive loop sleeps a fixed 100 ms after
each frame rather than scheduling against a deadline. Process memory: 68.1 MB after start-up,
89.1 MB after 256 plans. In the 2026-09-13 run, issuing 16 concurrent plans while 50 drives
streamed raised the drive frame-interval p95 from 125 ms to 965 ms (shared interpreter thread).

**Drive-cap race:** with `AV_MAX_CONCURRENT_DRIVES=2`, 10 drives started together were all
admitted; a connection arriving after they were running was correctly closed with 4429. The
check (`active_drives >= max_drives`) and the increment are separated by an `await` on the
first client message.

## 7. Deployed service

Measured with `curl` against `https://av-route-demo.fly.dev`.

| Measurement | 2026-11-11 | 2026-09-13 |
|---|---|---|
| First request after idle (cold start) | 11.28 s | 10.57 s |
| Warm `/healthz` | 1.13 to 1.55 s | 0.22 to 0.44 s |
| Warm `POST /plan` (12,893-byte response) | 1.25 to 1.43 s | 0.29 to 0.34 s |
| `/map/roads` | 1,145,843 B, 386,922 B Brotli | 1,145,843 B, 385,386 B Brotli |
| `/healthz` lanelets | 2,117 | 2,117 |

TLS 1.3 (AES-256-GCM), HTTP redirected to HTTPS with 301. Warm times are dominated by the
network path to Mumbai, not computation. Building `/map/roads` costs about 58 ms of CPU
(29.8 ms GeoJSON construction, 28.1 ms JSON encoding); it is rebuilt on every request.

## 8. Autoware cross-validation

Full method and logs in `apps/autoware_eval/EVAL.md`. Image
`ghcr.io/autowarefoundation/autoware:latest-runtime` at digest
`sha256:3662e297d7a41fffb54a16c6e494adc118b4162e659e0aea3b2081e46bcf91d6` (8.81 GB), run on
2026-07-06 against the 3,055-lanelet revision of the full map, caps 4 GB / 6 CPUs (peak about
2.9 GB).

| Journey | Ours | Autoware | Verdict |
|---|---|---|---|
| short_hop, lanelet 1 to 511 | 21 lanelets | 21 sections | strict match, lane for lane |
| cross_map, 1 to 1855 | 64 steps | 60 sections | same road-level route, section coverage 1.0 |
| mid_range, 1 to 1187 | 54 steps | 49 sections | same endpoints, diverges after step 9 (travel time vs length objective) |

Full-stack drive (`run_viz.ps1`, caps 5.5 GB / 14 CPUs): on the single-lane map the ego
vehicle drove a 21-lanelet multi-turn route (start lanelet 1, goal 50) to `route_state = 3`
(ARRIVED) at about 16 km/h. On the multi-lane map Autoware's lane-change module stalled on the
short Eixample blocks (progress 15 m, then 25 m, then 66 m as map fixes landed).

## 9. Quality gate

| Check | Result |
|---|---|
| `uv run pytest` | 144 passed in 12.31 s |
| Line coverage | 90.08 % (1,693 statements, 168 missed); gate 80 % |
| Coverage by package | avcore 97.6 % (56 tests), avmap_tools 87.7 % (43), avlane 73.6 % (9), webdemo_backend 94.6 % (36) |
| `uv run mypy` (strict) | no issues in 26 source files |
| `uv run ruff check .` and `ruff format --check .` | clean |
| GitHub Actions | about 22 s per run |
| Property tests | 60 + 60 + 40 Hypothesis examples on random grid graphs |

Uncovered: `avlane/detect.py` and `avmap_tools/import_osm.py` (command-line entry points,
exercised manually). Source size: avcore 796 lines, avmap_tools 1,241, avlane 381,
webdemo_backend 607; tests 1,580; Autoware harness 329.

## 10. Known issues found by measurement

- Distance-model heuristic is inconsistent (section 4).
- Drive-cap check-then-act race (section 6).
- Paused drives never expire and keep their slot; the command reader task ends on a
  malformed command; the drive loop period drifts (fixed sleep).
- Snapping is an O(segments) scan, about 75 % of request time; a grid index would remove it.
- Lanelet geometry is recomputed per access in the A* hot path (section 4).
- `apps/autoware_eval/prep_map.py` bakes the single-lane map by default while
  `compare_routes.py` plans on the full map; set `AV_SIM_MAP=maps/barcelona_eixample.osm`
  for the golden comparison so lanelet ids correspond.
- The importer does not model traffic signals, stop lines, crossings or turn-restriction
  relations.
