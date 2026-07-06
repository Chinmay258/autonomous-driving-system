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

## Results
(recorded after each run)
