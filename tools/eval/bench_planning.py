"""Planning benchmark on the committed Barcelona map.

Measures, in order: map artefact and graph structure, map build cost, service load time and
memory, road-network GeoJSON cost, planning latency over random routes, A* against a
zero-heuristic (Dijkstra) baseline, heuristic consistency for both cost models, a hot-path
profile, the effect of caching lanelet geometry, simulator tracking accuracy, telemetry
serialisation cost and route-store memory.

Run from the repository root:

    uv run python tools/eval/bench_planning.py

Results are printed and written to tools/eval/results/bench_results.json.
"""

from __future__ import annotations

import cProfile
import gc
import io
import itertools
import json
import math
import pstats
import random
import statistics as stats
import time
import tracemalloc
from pathlib import Path

from avmap_tools import build_graph, read_osm, validate_map

import avcore.planner as planner_mod
from avcore import Distance, EdgeKind, Point2D, TravelTime, filter_lanelets, plan_route
from avcore.models import Lanelet, polyline_length
from webdemo_backend.mapservice import MapService
from webdemo_backend.schemas import CostKind, DriveState, TelemetryFrame
from webdemo_backend.simulator import SimState, VehicleSimulator

REPO = Path(__file__).resolve().parents[2]
ART = REPO / "maps" / "barcelona_eixample.osm"
SIM_ART = REPO / "maps" / "barcelona_eixample_sim.osm"
RESULTS = Path(__file__).resolve().parent / "results"
OUT: dict[str, object] = {}


def rec(key, value):
    OUT[key] = value
    print(f"{key} = {value}", flush=True)


def pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, math.ceil(p / 100 * len(s)) - 1))]


def ms(x):
    return round(x * 1e3, 2)


def dist(xs):
    return {
        "p50": ms(pct(xs, 50)),
        "p95": ms(pct(xs, 95)),
        "p99": ms(pct(xs, 99)),
        "max": ms(max(xs)),
        "mean": ms(stats.mean(xs)),
    }


class ZeroHeuristic:
    """Wraps a cost model with h = 0, turning A* into Dijkstra's algorithm."""

    def __init__(self, inner):
        self.inner = inner

    def entry_cost(self, ll):
        return self.inner.entry_cost(ll)

    def edge_cost(self, a, b, kind):
        return self.inner.edge_cost(a, b, kind)

    def heuristic(self, node, goal, vmax):
        return 0.0


def section_map_structure():
    text = ART.read_text(encoding="utf-8")
    rec("artifact_bytes", len(text.encode("utf-8")))
    t0 = time.perf_counter()
    lanelets, origin = read_osm(text)
    rec("read_osm_s", round(time.perf_counter() - t0, 3))
    rec("lanelets_total", len(lanelets))
    rec("connectors", sum(1 for ll in lanelets if ll.is_connector))
    rec("origin", [round(origin.lat, 6), round(origin.lng, 6)])
    t0 = time.perf_counter()
    report = validate_map(lanelets)
    rec("validate_map_s", round(time.perf_counter() - t0, 3))
    rec("validate_ok", report.ok)
    t0 = time.perf_counter()
    graph = build_graph(lanelets)
    rec("build_graph_s", round(time.perf_counter() - t0, 3))
    edges = [e for lid in graph for e in graph.edges_from(lid)]
    rec("graph_V", len(graph))
    rec("graph_E", len(edges))
    rec("lateral_edges", sum(1 for e in edges if e.kind is not EdgeKind.SUCCESSOR))
    t0 = time.perf_counter()
    weak = filter_lanelets(graph)
    rec("filter_weak_s", round(time.perf_counter() - t0, 3))
    rec("weak_core", len(weak))
    t0 = time.perf_counter()
    strong = filter_lanelets(graph, require_strong=True)
    rec("filter_strong_s", round(time.perf_counter() - t0, 3))
    rec("strong_core", len(strong))
    rec("dead_end_lanelets", sum(1 for lid in graph if not graph.edges_from(lid)))
    cl = [len(ll.centerline) for ll in lanelets]
    rec("centerline_pts_mean", round(stats.mean(cl), 1))
    rec("centerline_pts_max", max(cl))
    street = [ll.length_m for ll in lanelets if not ll.is_connector]
    rec("street_lanelet_len_p50_m", round(pct(street, 50), 1))
    rec("street_lanelet_len_max_m", round(max(street), 1))
    sim_ll, _ = read_osm(SIM_ART.read_text(encoding="utf-8"))
    sim_g = build_graph(sim_ll)
    rec("sim_lanelets", len(sim_ll))
    rec(
        "sim_lateral_edges",
        sum(1 for lid in sim_g for e in sim_g.edges_from(lid) if e.kind is not EdgeKind.SUCCESSOR),
    )
    rec("sim_strong_core", len(filter_lanelets(sim_g, require_strong=True)))


def section_service():
    t0 = time.perf_counter()
    svc = MapService.from_osm_artifact(str(ART))
    rec("mapservice_load_s", round(time.perf_counter() - t0, 3))
    del svc
    gc.collect()
    tracemalloc.start()
    svc = MapService.from_osm_artifact(str(ART))
    gc.collect()
    cur, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    rec("mapservice_retained_MB_traced", round(cur / 1e6, 1))
    rec("mapservice_load_peak_MB_traced", round(peak / 1e6, 1))
    rec("snap_segments", len(svc._snap_segments))
    rec("service_lanelets", svc.lanelet_count)
    t0 = time.perf_counter()
    gj = svc.road_network_geojson()
    t1 = time.perf_counter()
    blob = json.dumps(gj)
    t2 = time.perf_counter()
    rec("geojson_build_ms", ms(t1 - t0))
    rec("geojson_json_dumps_ms", ms(t2 - t1))
    rec("geojson_bytes_json_dumps", len(blob))
    return svc


def random_pairs(svc, n, seed):
    ids = sorted(svc._lanelets)
    rng = random.Random(seed)

    def mid(lid):
        c = svc._lanelets[lid].centerline
        return svc.projector.to_latlng(c[len(c) // 2])

    return [(mid(rng.choice(ids)), mid(rng.choice(ids))) for _ in range(n)]


def section_latency(svc):
    graph = svc._graph
    pairs = random_pairs(svc, 300, 42)
    for a, b in pairs[:20]:
        svc.plan(a, b, CostKind.TRAVEL_TIME)
    model = TravelTime()
    snap_t, astar_t, total_t, dist_t, routes, snapped = [], [], [], [], [], []
    for a, b in pairs:
        t0 = time.perf_counter()
        sa, sb = svc.snap(a), svc.snap(b)
        t1 = time.perf_counter()
        plan_route(graph, sa, sb, model)
        t2 = time.perf_counter()
        snap_t.append(t1 - t0)
        astar_t.append(t2 - t1)
        snapped.append((sa, sb))
    for a, b in pairs:
        t0 = time.perf_counter()
        _, stored = svc.plan(a, b, CostKind.TRAVEL_TIME)
        total_t.append(time.perf_counter() - t0)
        routes.append(stored)
    for a, b in pairs:
        t0 = time.perf_counter()
        svc.plan(a, b, CostKind.DISTANCE)
        dist_t.append(time.perf_counter() - t0)
    rec("snap_x2_ms", dist(snap_t))
    rec("astar_traveltime_ms", dist(astar_t))
    rec("plan_total_traveltime_ms", dist(total_t))
    rec("plan_total_distance_ms", dist(dist_t))
    d = [s.route.distance_m for s in routes]
    n = [len(s.route.lanelet_ids) for s in routes]
    lc = [s.route.lane_changes for s in routes]
    rec("route_distance_m", {"p50": round(pct(d, 50)), "max": round(max(d))})
    rec("route_lanelets", {"p50": pct(n, 50), "max": max(n)})
    rec("route_lane_changes", {"p50": pct(lc, 50), "max": max(lc)})
    t0 = time.perf_counter()
    for _ in range(50):
        max(graph.lanelet(lid).speed_limit_mps for lid in graph)
    rec("per_call_max_speed_scan_ms", ms((time.perf_counter() - t0) / 50))
    return snapped, routes


def graph_cost(graph, route_ids, model):
    total = model.entry_cost(graph.lanelet(route_ids[0]))
    for x, y in itertools.pairwise(route_ids):
        total += min(
            model.edge_cost(graph.lanelet(x), graph.lanelet(y), e.kind)
            for e in graph.edges_from(x)
            if e.target == y
        )
    return total


def section_astar_vs_dijkstra(svc, snapped):
    graph = svc._graph
    model = TravelTime()
    real_edges_from = graph.edges_from
    calls = [0]

    def counting_edges_from(lid):
        calls[0] += 1
        return real_edges_from(lid)

    exp_a, exp_d, t_a, t_d, mismatch = [], [], [], [], 0
    for sa, sb in snapped[:150]:
        graph.edges_from = counting_edges_from
        calls[0] = 0
        t0 = time.perf_counter()
        ra = plan_route(graph, sa, sb, model)
        t_a.append(time.perf_counter() - t0)
        exp_a.append(calls[0])
        calls[0] = 0
        t0 = time.perf_counter()
        rd = plan_route(graph, sa, sb, ZeroHeuristic(model))
        t_d.append(time.perf_counter() - t0)
        exp_d.append(calls[0])
        graph.edges_from = real_edges_from
        ca = graph_cost(graph, ra.lanelet_ids, model)
        cd = graph_cost(graph, rd.lanelet_ids, model)
        if abs(ca - cd) > 1e-6 * max(1.0, cd):
            mismatch += 1
    graph.edges_from = real_edges_from
    rec("astar_expansions", {"p50": pct(exp_a, 50), "max": max(exp_a)})
    rec("dijkstra_expansions", {"p50": pct(exp_d, 50), "max": max(exp_d)})
    rec(
        "astar_over_dijkstra_expansions_mean",
        round(stats.mean(a / b for a, b in zip(exp_a, exp_d, strict=True) if b), 3),
    )
    rec("astar_ms_mean", ms(stats.mean(t_a)))
    rec("dijkstra_ms_mean", ms(stats.mean(t_d)))
    rec("cost_mismatch_astar_vs_dijkstra", f"{mismatch}/150")


def section_consistency(svc):
    """Is A* optimal for each cost model? Compared with Dijkstra on 400 random pairs."""
    graph = svc._graph
    pairs = random_pairs(svc, 400, 7)
    snapped = [(svc.snap(a), svc.snap(b)) for a, b in pairs]
    for name, model in (("traveltime", TravelTime()), ("distance", Distance())):
        worse, worst_abs, worst_rel = 0, 0.0, 0.0
        for sa, sb in snapped:
            ca = graph_cost(graph, plan_route(graph, sa, sb, model).lanelet_ids, model)
            cd = graph_cost(
                graph, plan_route(graph, sa, sb, ZeroHeuristic(model)).lanelet_ids, model
            )
            if ca > cd + 1e-6 * max(1.0, cd):
                worse += 1
                worst_abs = max(worst_abs, ca - cd)
                worst_rel = max(worst_rel, (ca - cd) / cd)
        rec(
            f"suboptimal_{name}",
            {
                "routes": f"{worse}/400",
                "worst_extra": round(worst_abs, 2),
                "worst_extra_pct": round(100 * worst_rel, 2),
            },
        )


def section_profile(svc, snapped):
    graph = svc._graph
    model = TravelTime()
    prof = cProfile.Profile()
    prof.enable()
    for sa, sb in snapped[:100]:
        plan_route(graph, sa, sb, model)
    prof.disable()
    buf = io.StringIO()
    pstats.Stats(prof, stream=buf).sort_stats("tottime").print_stats(10)
    lines = buf.getvalue().splitlines()
    start = next(i for i, line in enumerate(lines) if "ncalls" in line)
    print("PROFILE (100 plan_route calls, sorted by tottime)")
    print("\n".join(lines[start : start + 11]))


def section_memoised(svc, snapped):
    """What-if: cache per-lanelet length and end point (analysis only, not shipped)."""
    graph = svc._graph
    model = TravelTime()
    length_cache, end_cache = {}, {}
    orig_length, orig_endpoint = Lanelet.length_m, planner_mod._endpoint

    def cached_length(self):
        v = length_cache.get(self.id)
        if v is None:
            v = length_cache[self.id] = polyline_length(self.centerline)
        return v

    def cached_endpoint(ll):
        v = end_cache.get(ll.id)
        if v is None:
            v = end_cache[ll.id] = ll.centerline[-1]
        return v

    Lanelet.length_m = property(cached_length)
    planner_mod._endpoint = cached_endpoint
    try:
        for sa, sb in snapped[:20]:
            plan_route(graph, sa, sb, model)
        times = []
        for sa, sb in snapped:
            t0 = time.perf_counter()
            plan_route(graph, sa, sb, model)
            times.append(time.perf_counter() - t0)
    finally:
        Lanelet.length_m, planner_mod._endpoint = orig_length, orig_endpoint
    rec(
        "astar_memoised_ms",
        {"p50": ms(pct(times, 50)), "p95": ms(pct(times, 95)), "mean": ms(stats.mean(times))},
    )


def section_simulator(svc, routes):
    cte_all, per_route, states, reasons = [], [], {}, {}
    steps_total, wall_total = 0, 0.0
    last = None
    for stored in routes[:120]:
        r = stored.route
        sim = VehicleSimulator(r.centerline, stored.speed_limits_mps)
        ctes, steps = [], 0
        t0 = time.perf_counter()
        while True:
            f = sim.step(0.1)
            steps += 1
            if f.state is not SimState.DRIVING:
                break
            ctes.append(f.cte_m)
        wall_total += time.perf_counter() - t0
        steps_total += steps
        last = f
        states[f.state.value] = states.get(f.state.value, 0) + 1
        if f.reason:
            reasons[f.reason] = reasons.get(f.reason, 0) + 1
        cte_all.extend(ctes)
        if ctes and f.t > 0:
            per_route.append((stats.mean(ctes), max(ctes), r.distance_m, f.t))
    rec("sim_routes", sum(states.values()))
    rec("sim_terminal_states", states)
    rec("sim_abort_reasons", reasons)
    rec(
        "cte_m_pooled",
        {
            "mean": round(stats.mean(cte_all), 3),
            "p50": round(pct(cte_all, 50), 3),
            "p95": round(pct(cte_all, 95), 3),
            "p99": round(pct(cte_all, 99), 3),
            "max": round(max(cte_all), 3),
            "samples": len(cte_all),
        },
    )
    rec("cte_m_per_route_max_p50", round(pct([p[1] for p in per_route], 50), 3))
    rec("sim_step_cost_us", round(wall_total / steps_total * 1e6, 1))
    rec("sim_mean_speed_kmh", round(3.6 * stats.mean(p[2] / p[3] for p in per_route), 1))

    t0 = time.perf_counter()
    for _ in range(20000):
        c = svc.projector.to_latlng(Point2D(last.x, last.y))
        payload = TelemetryFrame(
            t=round(last.t, 3),
            lat=c.lat,
            lng=c.lng,
            heading_deg=(90.0 - math.degrees(last.heading_rad)) % 360.0,
            speed_mps=round(last.speed_mps, 3),
            cte_m=round(last.cte_m, 3),
            progress=round(last.progress, 5),
            state=DriveState.DRIVING,
        ).model_dump_json()
    rec("telemetry_serialize_us", round((time.perf_counter() - t0) / 20000 * 1e6, 1))
    rec("telemetry_frame_bytes", len(payload))


def section_route_store(svc):
    pairs = random_pairs(svc, 200, 42)
    svc._routes.clear()
    gc.collect()
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    for a, b in pairs:
        svc.plan(a, b, CostKind.TRAVEL_TIME)
    gc.collect()
    used = tracemalloc.get_traced_memory()[0] - base
    tracemalloc.stop()
    per_entry = used / len(svc._routes)
    rec("route_store_kB_per_entry_traced", round(per_entry / 1e3, 1))
    rec("route_store_MB_at_cap_1000_est", round(per_entry * 1000 / 1e6, 1))


def main() -> None:
    section_map_structure()
    gc.collect()
    svc = section_service()
    snapped, routes = section_latency(svc)
    section_astar_vs_dijkstra(svc, snapped)
    section_consistency(svc)
    section_profile(svc, snapped)
    section_memoised(svc, snapped)
    section_simulator(svc, routes)
    section_route_store(svc)
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "bench_results.json").write_text(json.dumps(OUT, indent=1, default=str))
    print("BENCH_OK")


if __name__ == "__main__":
    main()
