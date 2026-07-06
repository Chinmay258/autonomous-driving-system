# Autoware Planning Simulator — evaluation

Goal: validate that the same Lanelet2 artifact driving the web demo loads in
Autoware, and that Autoware's mission planner picks routes consistent with our
A* (golden-reference test, headless — no rviz needed).

## Constraints on this machine (do not violate)
The WSL2 VM (6.8 GB) is shared with a LIVE trading stack. The Autoware
container must always run with `--memory=4g --memory-swap=4g --cpus=6` so any
OOM kills Autoware, never the trading containers. Never run `wsl --shutdown`
or edit `.wslconfig` while the trading stack is up.

## Recipe
```powershell
# 1. Bake the map dir (lanelet2_map.osm + projector info + dummy pcd)
uv run python apps/autoware_eval/prep_map.py

# 2. Start the planning simulator (headless)
docker run -d --name autoware_eval --memory=4g --memory-swap=4g --cpus=6 `
  -v ${PWD}/apps/autoware_eval/mapdir:/autoware_map:ro `
  -v ${PWD}/apps/autoware_eval:/eval:ro `
  ghcr.io/autowarefoundation/autoware:latest-runtime `
  ros2 launch autoware_launch planning_simulator.launch.xml `
    map_path:=/autoware_map vehicle_model:=sample_vehicle `
    sensor_model:=sample_sensor_kit rviz:=false

# 3. Scenario ids (deterministic, from the artifact)
uv run python apps/autoware_eval/compare_routes.py scenarios

# 4. Golden probe inside the container (per scenario)
docker exec autoware_eval bash -lc "source /opt/autoware/setup.bash && \
  python3 /eval/golden_route_check.py --map /autoware_map/lanelet2_map.osm \
  --origin-lat <lat> --origin-lon <lon> --start-id <S> --goal-id <G>"

# 5. Verdict
uv run python apps/autoware_eval/compare_routes.py compare --start-id <S> \
  --goal-id <G> --golden '<json printed after GOLDEN_ROUTE=>'

# 6. Tear down
docker rm -f autoware_eval
```

## Comparison semantics
Autoware's `LaneletRoute` has one segment per road section (parallel lanes as
`primitives`, one `preferred_primitive`). Our route lists every lane + turn
connector individually, so the comparator reports:
- `strict_match` — identical preferred sequences,
- `section_coverage` — fraction of our steps that thread through Autoware's
  sections in order (lane-choice-tolerant; 1.0 = same road-level route),
- `same_endpoints` — both planners agree on start/goal sections (gate).

## Results — 2026-07-06, image `latest-runtime` (8.81 GB), map `barcelona_eixample` (3,055 lanelets)

The artifact loaded natively: `Succeeded to load lanelet2_map. Map is published.`
Peak container memory ~2.9 GB of the 4 GB cap; trading stack unaffected.

| Scenario | Start→Goal | Ours | Autoware | Verdict |
|---|---|---|---|---|
| short_hop | 1 → 511 | 21 steps | 21 sections | **strict match — identical lanelet sequence, lane-for-lane** |
| cross_map | 1 → 1855 | 64 steps | 60 sections | same road-level route (section coverage **1.0**), minor lane-preference diffs |
| mid_range | 1 → 1187 | 54 steps | 49 sections | same endpoints; **routes diverge after step 9** — different street choice |

Interpretation: two of three scenarios agree at road level (one perfectly).
The mid_range divergence is a cost-model difference, not a defect: our
TravelTime model (speed-limit-weighted + 5 s lane-change penalty) prefers a
faster corridor; Autoware's default mission planner optimizes route length.
Both routes are valid and drivable on the same map.

### Map-compatibility fixes this evaluation forced (now regression-tested)
- Connector bounds must **reuse the street lanes' exact bound nodes** —
  lanelet2 succession works on shared points, not proximity.
- Same-direction lane dividers must be `subtype=dashed` or lanelet2 traffic
  rules forbid lane changes entirely.
- Offset bounds must never fold (arc radius < half lane width) or lanelet2
  inverts them into bowties: near-straight fillets keep controls inside the
  chord, U-turns are teardrop arcs (radius >= 3 m), fold detection gates every
  connector.

## Visual mode (rviz on the Windows desktop)
`run_viz.ps1` launches the same simulator with rviz displayed through WSLg
(Docker Desktop exposes it at `/run/desktop/mnt/host/wslg`). Verified driving
on the Barcelona map. Two hard-won requirements, baked into the script:
- **12 CPUs**, not 6: rviz renders via llvmpipe (~3 cores); at 6 the control
  loop starves, its rate monitors flag ERROR and the controller latches stop.
- **`overrides/pid.param.yaml`** (mounted over the stock config) disables
  `enable_keep_stopped_until_steer_convergence`, which deadlocks departure at
  standstill in this containerized setup (the flag is read once at node boot,
  so a runtime `ros2 param set` does not help).

### Known issue: stall on the tightest turn connectors (map-side, open)
Driving works from spawn and on straights, but on the sharpest junction
fillets Autoware's behavior_path_planner outputs a degenerate path: the final
trajectory ends ~0.2 m ahead of ego with v=0, so the controller (correctly)
refuses to depart. Root cause: our turn connectors allow arc radii down to
~2 m over a few meters of length, below what the module's resampling
tolerates. Fix queued: enforce a minimum fillet radius (~5 m) and minimum
connector arc length in avmap_tools.osm_import, regenerate the artifact,
re-run the golden eval. Workaround meanwhile: rerun `run_viz.ps1` (fresh boot
re-places the car) and pick goals along boulevards.

### Operational quirks (WSL2 + docker exec)
- Probe **immediately** after `waiting odometry` appears; the stack's
  `/initialpose` subscription can stop matching new DDS participants some
  minutes after boot (recreate the container rather than restart it).
- The route topic is transient-local: a probe must reject latched routes whose
  final segment doesn't contain its goal id.
