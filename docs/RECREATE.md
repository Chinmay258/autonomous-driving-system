# Recreating the project from scratch

Everything needed to rebuild, run, evaluate, measure and deploy the system lives in this
repository. This guide goes from a clean machine to every artefact the project produced.

## 0. Prerequisites

| Tool | Version used | Needed for |
|---|---|---|
| Git | any recent | cloning |
| Python | 3.11 (`.python-version`) | everything |
| [uv](https://docs.astral.sh/uv/) | 0.11 | environment and dependency locking (`pip install uv` works) |
| Docker Desktop with WSL2 | 4.x | container image, Autoware evaluation |
| flyctl | any recent | public deployment |

On Windows, uv is available as `python -m uv` after `pip install uv`.

## 1. Environment and quality gate

```bash
git clone https://github.com/Chinmay258/autonomous-driving-system.git
cd autonomous-driving-system
uv sync --all-packages          # exact versions from uv.lock
uv run ruff check . && uv run ruff format --check .
uv run mypy                     # strict, 26 source files
uv run pytest                   # 144 tests, coverage gate 80 %
```

## 2. Run the live demo locally

```bash
AV_MAP_PATH=maps/barcelona_eixample.osm uv run uvicorn webdemo_backend.app:create_app --factory --port 8017
```

PowerShell: `$env:AV_MAP_PATH="maps/barcelona_eixample.osm"; uv run uvicorn webdemo_backend.app:create_app --factory --port 8017`.
Open http://localhost:8017, click a start and a goal, press Drive. Configuration:

| Variable | Default | Meaning |
|---|---|---|
| `AV_MAP_PATH` | unset (synthetic town) | Lanelet2 artefact to serve |
| `AV_TOWN_BLOCKS` | 3 | synthetic town size when no map is given |
| `AV_SIM_REALTIME` | 1 | 1 = 10 Hz real time, 0 = as fast as possible |
| `AV_MAX_CONCURRENT_DRIVES` | 20 | WebSocket drive cap (close code 4429 above it) |

## 3. Rebuild the maps

The raw OpenStreetMap extracts used for the committed maps are in `maps/*.overpass.xml`.
The importer reads `<out>.overpass.xml` when it exists, so these commands reproduce the
committed artefacts byte for byte (the `--bbox` is ignored when the cache file is present):

```bash
uv run python -m avmap_tools.import_osm --bbox "41.385,2.158,41.398,2.174" --out maps/barcelona_eixample
uv run python -m avmap_tools.import_osm --bbox "41.385,2.158,41.398,2.174" --single-lane --out maps/barcelona_eixample_sim
```

Each writes `<out>.osm`, `<out>_projector_info.yaml` and validates the map. Delete or rename
the `.overpass.xml` file to fetch fresh data from the Overpass API instead (results will
differ as OpenStreetMap is edited). Any other city works the same way; add `--drive-on left`
for left-hand traffic.

## 4. Autoware evaluation

Needs Docker Desktop on WSL2 with at least 6 CPUs and 4 GB free for the headless run (the
visual run uses up to 14 CPUs and 5.5 GB). The scripts pin the exact image used for every
recorded result:

```
ghcr.io/autowarefoundation/autoware:latest-runtime@sha256:3662e297d7a41fffb54a16c6e494adc118b4162e659e0aea3b2081e46bcf91d6
```

```powershell
docker pull ghcr.io/autowarefoundation/autoware:latest-runtime@sha256:3662e297d7a41fffb54a16c6e494adc118b4162e659e0aea3b2081e46bcf91d6
# golden-route comparison (headless); use the full map so lanelet ids match the comparator
$env:AV_SIM_MAP="maps/barcelona_eixample.osm"; powershell -File apps/autoware_eval/run_eval.ps1
# visual drive in rviz through WSLg (single-lane map, drives to ARRIVED)
Remove-Item Env:AV_SIM_MAP; powershell -File apps/autoware_eval/run_viz.ps1
powershell -File apps/autoware_eval/open_rviz.ps1   # reopen rviz any time
docker rm -f autoware_viz                            # tear down
```

Method, the parameter overrides in `apps/autoware_eval/overrides/`, and every operational
quirk (localisation initialisation service, latched routes, DDS discovery on aged containers)
are documented in `apps/autoware_eval/EVAL.md`.

## 5. Container and deployment

```bash
docker build -t av-demo .
docker run --rm -p 8000:8000 av-demo          # http://localhost:8000
```

Fly.io (app `av-route-demo`, region `bom`, 512 MB shared CPU, auto-stop when idle):

```bash
fly auth login
fly launch --copy-config --no-deploy          # first time only; keeps fly.toml
fly deploy --ha=false                         # one machine
```

Alternative self-hosting behind a Cloudflare quick tunnel:
`docker compose -f infra/docker-compose.yml up -d --build` (see `infra/README.md`). A named
tunnel needs a `TUNNEL_TOKEN` in `infra/.env`, which is never committed.

## 6. Lane detection

```bash
uv run python -m avlane.detect --input dashcam.mp4 --output-dir out/
```

Writes `out/lanes.json` (metric lane width, curvature, offset, boundaries per frame) and an
annotated video. The test suite checks accuracy against synthetic frames with known geometry.

## 7. Measurements and figures

```bash
uv run python tools/eval/bench_planning.py                       # sections 1-5 of docs/RESULTS.md
uv run python tools/eval/load_test.py                            # section 6 (Windows)
uv run --with matplotlib python tools/eval/make_figures.py       # docs/figures/*.png
```

Results go to `tools/eval/results/` (ignored by git). `docs/RESULTS.md` records the values
used in the report and presentations, with dates.

## 8. Documentation map

| Document | Contents |
|---|---|
| `SPEC.md` | original specification and feasibility study |
| `ARCHITECTURE.md` | architecture, interface contracts, phased plan, acceptance criteria |
| `docs/adr/` | decision records: Lanelet2 as canonical format, shared planning core, hosting |
| `docs/PROJECT_REPORT.md` | detailed working report: phases, hard problems, results |
| `docs/RESULTS.md` | every measured number and how it was measured |
| `apps/autoware_eval/EVAL.md` | Autoware evaluation method, results and quirks |
| `maps/README.md` | map artefacts, provenance and licence |
| `docs/figures/` | figures used in the report and presentations |
