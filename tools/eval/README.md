# Evaluation scripts

Scripts that produced every measurement in `docs/RESULTS.md` and every figure in
`docs/figures/`. Run them from the repository root after `uv sync --all-packages`.

| Script | Produces | Run |
|---|---|---|
| `bench_planning.py` | map structure, build cost, planning latency, A* vs Dijkstra, heuristic consistency, profile, cached-geometry what-if, simulator accuracy, memory | `uv run python tools/eval/bench_planning.py` |
| `load_test.py` | drive-cap race, HTTP planning throughput at 1/4/16/64 concurrency, WebSocket drives at 1/50/200/500 | `uv run python tools/eval/load_test.py` |
| `make_figures.py` | the 14 report figures in `docs/figures/` | `uv run --with matplotlib python tools/eval/make_figures.py` |

Notes:

- Results are written to `tools/eval/results/` (git-ignored) and printed.
- `load_test.py` starts the service twice on ports 8791 and 8792 and reads process CPU and
  memory through the Win32 API, so it runs on Windows.
- Timings depend on the machine and its load. Counts, ratios, route agreement and tracking
  error are deterministic for a given seed and map.
- `make_figures.py` measures the cross-track error figure on every run; the latency,
  search, concurrency and coverage figures plot the recorded values from `docs/RESULTS.md`.
