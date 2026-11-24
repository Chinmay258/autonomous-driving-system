"""Generate the report figures from the committed Barcelona map and fresh measurements.

Run from the repository root:

    uv run --with matplotlib python tools/eval/make_figures.py

Figures are written to docs/figures/. The architecture, lane-expansion, graph, lane-change,
pure-pursuit and schedule figures are drawn; the junction and route figures come from the
real map; the latency, search, concurrency and coverage figures plot the values recorded in
docs/RESULTS.md; the cross-track-error figure is measured on every run.
"""

from __future__ import annotations

import math
import random
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

from avcore import EdgeKind, Lanelet, LaneletId, Point2D, RoutingGraph, TravelTime, plan_route
from webdemo_backend.mapservice import MapService
from webdemo_backend.schemas import CostKind
from webdemo_backend.simulator import SimState, VehicleSimulator

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "docs" / "figures"
INK, LANE, ROUTE, CONN, GREEN = "#1b2733", "#9aa7b4", "#1f5fa8", "#c77700", "#2e7d32"
NL = "\n"

plt.switch_backend("Agg")


def save(fig, name):
    fig.savefig(OUT / name, facecolor="white")
    plt.close(fig)
    print("wrote", name, flush=True)


def heading_delta(ll):
    c = ll.centerline
    a = math.atan2(c[1].y - c[0].y, c[1].x - c[0].x)
    b = math.atan2(c[-1].y - c[-2].y, c[-1].x - c[-2].x)
    return math.atan2(math.sin(b - a), math.cos(b - a))


def fig_architecture():
    fig, ax = plt.subplots(figsize=(6.6, 4.6))
    ax.set_axis_off()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 62)
    ax.grid(False)

    def box(x, y, w, h, title, lines, fc="#eef3fb", ec=ROUTE):
        ax.add_patch(
            FancyBboxPatch(
                (x, y), w, h, boxstyle="round,pad=0.6", linewidth=1.1, facecolor=fc, edgecolor=ec
            )
        )
        ax.text(
            x + w / 2,
            y + h - 2.6,
            title,
            ha="center",
            va="top",
            fontsize=10.2,
            fontweight="bold",
            color=INK,
        )
        for i, line in enumerate(lines):
            ax.text(
                x + 2.5, y + h - 6.8 - i * 3.6, line, ha="left", va="top", fontsize=8.6, color=INK
            )

    def arrow(x1, y1, x2, y2, label="", rad=0.0, lx=None, ly=None, ha="center"):
        ax.add_patch(
            FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle="-|>",
                mutation_scale=9,
                linewidth=1.0,
                color="#445061",
                connectionstyle="arc3,rad=" + str(rad),
            )
        )
        if label:
            ax.text(
                lx if lx is not None else (x1 + x2) / 2,
                ly if ly is not None else (y1 + y2) / 2 + 1.0,
                label,
                ha=ha,
                va="center",
                fontsize=8.2,
                color="#445061",
                bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.8},
            )

    box(
        2,
        50,
        42,
        10,
        "OpenStreetMap  (Overpass API)",
        ["raw highway ways + tags, cached locally"],
        fc="#f4f6f8",
        ec="#7a8794",
    )
    box(
        2,
        33,
        42,
        14,
        "avmap_tools  -  map production",
        [
            "import_osm_roads(): per-lane expansion,",
            "junction clustering, turn connectors, U-turns",
            "write_osm() / validate_map()",
        ],
    )
    box(
        2,
        17,
        19.5,
        12,
        "barcelona_eixample.osm",
        ["Lanelet2 XML", "2,545 lanelets"],
        fc="#fff5e3",
        ec=CONN,
    )
    box(24.5, 17, 19.5, 12, "..._sim.osm", ["single-lane", "1,144 lanelets"], fc="#fff5e3", ec=CONN)
    box(
        55,
        33,
        43,
        27,
        "avcore  -  shared planning core",
        [
            "models: Lanelet, RouteResult",
            "graphbuild -> RoutingGraph",
            "filtering: sanity + largest SCC",
            "planner: A* (Distance | TravelTime)",
            "routeops: arclength trim / slice",
        ],
    )
    box(
        55,
        17,
        43,
        12,
        "webdemo_backend  -  FastAPI",
        ["POST /plan, WS /ws/drive (10 Hz)", "MapService + VehicleSimulator"],
        fc="#eef7ef",
        ec=GREEN,
    )
    box(55, 2, 20, 11, "Browser", ["MapLibre GL JS", "route, live car"], fc="#eef7ef", ec=GREEN)
    box(
        78,
        2,
        20,
        11,
        "Docker / Fly.io",
        ["python:3.11-slim", "512 MB instance"],
        fc="#eef7ef",
        ec=GREEN,
    )
    box(
        2,
        2,
        42,
        11,
        "Autoware planning simulator",
        ["ROS 2: mission -> behaviour -> control", "golden-route probe and comparator"],
        fc="#f4f6f8",
        ec="#7a8794",
    )

    arrow(23, 50, 23, 47.4)
    arrow(14, 33, 14, 29.4)
    arrow(33, 33, 33, 29.4)
    arrow(38, 17, 38, 13.4, "prep_map.py", lx=39.5, ly=15.2, ha="left")
    arrow(21.5, 23, 55, 43, "read_osm()", rad=0.10, lx=47.0, ly=31.0)
    arrow(76, 33, 76, 29.4)
    arrow(64, 17, 64, 13.4, "HTTPS / WSS", lx=62.5, ly=15.2, ha="right")
    arrow(88, 17, 88, 13.4, "image", lx=89.5, ly=15.2, ha="left")
    arrow(44, 7.5, 55, 36, "comparator" + NL + "imports avcore", rad=-0.30, lx=50.5, ly=26.0)
    save(fig, "architecture.png")


def fig_lane_expansion():
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.7))
    xs = np.linspace(0, 60, 200)
    ys = 6 * np.sin(xs / 22)
    dx, dy = np.gradient(xs), np.gradient(ys)
    n = np.hypot(dx, dy)
    nx, ny = -dy / n, dx / n
    axes[0].plot(xs, ys, color=INK, lw=1.6)
    axes[0].set_title(
        "(a) OpenStreetMap way: one centre line" + NL + "lanes=4, oneway=no, maxspeed=50"
    )
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    w = 3.5
    for k in (-2, -1, 0, 1, 2):
        style = (
            {"color": "#c9ced6", "lw": 1.0} if k else {"color": INK, "lw": 1.2, "ls": (0, (6, 4))}
        )
        axes[1].plot(xs + nx * k * w, ys + ny * k * w, **style)
    for k, col in ((-1.5, GREEN), (-0.5, GREEN), (0.5, ROUTE), (1.5, ROUTE)):
        axes[1].plot(xs + nx * k * w, ys + ny * k * w, color=col, lw=2.2, alpha=0.65)
    axes[1].annotate(
        "",
        xy=(xs[150] + nx[150] * 1.5 * w, ys[150] + ny[150] * 1.5 * w),
        xytext=(xs[120] + nx[120] * 1.5 * w, ys[120] + ny[120] * 1.5 * w),
        arrowprops={"arrowstyle": "-|>", "color": ROUTE, "lw": 1.4},
    )
    axes[1].annotate(
        "",
        xy=(xs[40] - nx[40] * 1.5 * w, ys[40] - ny[40] * 1.5 * w),
        xytext=(xs[70] - nx[70] * 1.5 * w, ys[70] - ny[70] * 1.5 * w),
        arrowprops={"arrowstyle": "-|>", "color": GREEN, "lw": 1.4},
    )
    axes[1].set_title(
        "(b) Four lanelets, 3.5 m wide" + NL + "dashed line = shared divider (lane change allowed)"
    )
    save(fig, "lane_expansion.png")


def fig_junction(lanelets):
    r = 45.0
    mids = [(ll, ll.centerline[len(ll.centerline) // 2]) for ll in lanelets.values()]
    turners = [(ll, m) for ll, m in mids if ll.is_connector and abs(heading_delta(ll)) > 1.2]
    best, c = None, None
    for _, m in turners[:400]:
        near = [o for o, q in mids if abs(q.x - m.x) <= r and abs(q.y - m.y) <= r]
        n_conn = sum(1 for o in near if o.is_connector)
        n_lane = len(near) - n_conn
        # A readable illustration: a four-arm crossing, not a five-way tangle.
        score = abs(n_conn - 10) + abs(n_lane - 8)
        if best is None or score < best:
            best, c = score, m
    fig, ax = plt.subplots(figsize=(4.6, 4.6))
    seen = {"straight": False, "left": False, "right": False}
    for ll in lanelets.values():
        pts = ll.centerline
        mid = pts[len(pts) // 2]
        if abs(mid.x - c.x) > r or abs(mid.y - c.y) > r:
            continue
        xs = [p.x - c.x for p in pts]
        ys = [p.y - c.y for p in pts]
        if not ll.is_connector:
            ax.plot(xs, ys, color=LANE, lw=2.0, solid_capstyle="round")
            i = len(pts) // 2
            j = min(i + 1, len(xs) - 1)
            ax.annotate(
                "",
                xy=(xs[j], ys[j]),
                xytext=(xs[i], ys[i]),
                arrowprops={"arrowstyle": "-|>", "color": "#6b7885", "lw": 0.9},
            )
        else:
            d = heading_delta(ll)
            kind = "straight" if abs(d) < math.pi / 6 else ("left" if d > 0 else "right")
            col = {"straight": GREEN, "left": ROUTE, "right": CONN}[kind]
            ax.plot(xs, ys, color=col, lw=1.5, label=None if seen[kind] else kind + " connector")
            seen[kind] = True
    ax.set_aspect("equal")
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    ax.set_title(
        "Junction cluster: street lanelets (grey) and"
        + NL
        + "turn connectors generated under lane discipline"
    )
    save(fig, "junction.png")


def fig_graph_edges():
    fig, ax = plt.subplots(figsize=(6.2, 2.4))
    ax.set_axis_off()
    ax.grid(False)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 30)
    pos = {"A": (12, 20), "B": (40, 20), "C": (68, 20), "D": (12, 8), "E": (40, 8), "F": (68, 8)}
    for name, (x, y) in pos.items():
        ax.add_patch(Circle((x, y), 4.2, facecolor="#eef3fb", edgecolor=ROUTE, lw=1.1))
        ax.text(x, y, name, ha="center", va="center", fontsize=8.5, fontweight="bold", color=INK)

    def edge(a, b, label, rad, color):
        (x1, y1), (x2, y2) = pos[a], pos[b]
        ax.add_patch(
            FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle="-|>",
                mutation_scale=10,
                lw=1.1,
                color=color,
                shrinkA=15,
                shrinkB=15,
                connectionstyle="arc3,rad=" + str(rad),
            )
        )
        ax.text(
            (x1 + x2) / 2,
            (y1 + y2) / 2 + (2.4 if rad >= 0 else -4.0),
            label,
            ha="center",
            fontsize=7.2,
            color=color,
        )

    edge("A", "B", "successor", 0, INK)
    edge("B", "C", "successor", 0, INK)
    edge("D", "E", "successor", 0, INK)
    edge("E", "F", "successor", 0, INK)
    edge("A", "D", "RIGHT (lane change)", -0.35, CONN)
    edge("D", "A", "LEFT (lane change)", -0.35, ROUTE)
    ax.text(84, 20, "lane 1 (left)", fontsize=7.5, color="#445061", va="center")
    ax.text(84, 8, "lane 2 (right)", fontsize=7.5, color="#445061", va="center")
    ax.set_title("Edge kinds in the routing graph", fontsize=9.5)
    save(fig, "graph_edges.png")


def fig_lane_change():
    def box_ll(lid, x0, y0, length, width=3.5, speed=13.9):
        h = width / 2
        return Lanelet(
            id=LaneletId(lid),
            left_bound=(Point2D(x0, y0 + h), Point2D(x0 + length, y0 + h)),
            right_bound=(Point2D(x0, y0 - h), Point2D(x0 + length, y0 - h)),
            speed_limit_mps=speed,
        )

    g = RoutingGraph()
    layout = {1: (0, 0, 60), 2: (60, 0, 60), 3: (0, 3.5, 60), 4: (60, 3.5, 60)}
    for lid, (x0, y0, ln) in layout.items():
        g.add_lanelet(box_ll(lid, x0, y0, ln))
    g.add_edge(LaneletId(1), LaneletId(2), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(3), LaneletId(4), EdgeKind.SUCCESSOR)
    g.add_edge(LaneletId(1), LaneletId(3), EdgeKind.LEFT)
    g.add_edge(LaneletId(2), LaneletId(4), EdgeKind.LEFT)
    result = plan_route(g, LaneletId(1), LaneletId(4), TravelTime())
    fig, ax = plt.subplots(figsize=(6.6, 2.2))
    for lid in (1, 2, 3, 4):
        ll = g.lanelet(LaneletId(lid))
        for bound in (ll.left_bound, ll.right_bound):
            ax.plot([p.x for p in bound], [p.y for p in bound], color="#c9ced6", lw=0.8)
        ax.plot(
            [p.x for p in ll.centerline],
            [p.y for p in ll.centerline],
            color=LANE,
            lw=1.0,
            ls=(0, (4, 3)),
        )
    ax.plot(
        [p.x for p in result.centerline],
        [p.y for p in result.centerline],
        color=ROUTE,
        lw=2.4,
        solid_capstyle="round",
        label="planned path",
    )
    end = result.centerline[-1]
    ax.scatter([end.x], [end.y], color="#b3261e", s=22, zorder=5)
    ax.set_aspect("equal")
    ax.set_xlim(88, 126)
    ax.set_ylim(-3.5, 7.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="lower left", fontsize=7.5)
    ax.set_title(
        "Lane change stitched as one tangent-aligned cubic diagonal (zoom on the manoeuvre)",
        fontsize=9,
    )
    save(fig, "lane_change.png")


def fig_pure_pursuit():
    fig, ax = plt.subplots(figsize=(4.4, 3.0))
    t = np.linspace(0, 1, 200)
    px, py = 40 * t, 9 * np.sin(2.0 * t)
    ax.plot(px, py, color=LANE, lw=2.0)
    cx, cy = 6.0, -0.7
    ax.scatter([cx], [cy], color=INK, s=26, zorder=5)
    ax.text(cx - 0.6, cy - 2.4, "vehicle", fontsize=7.5, color=INK)
    look = 12.0
    d = np.hypot(px - cx, py - cy)
    i = int(np.argmin(np.abs(d - look)))
    ax.add_patch(Circle((cx, cy), look, fill=False, color=CONN, ls=(0, (4, 3)), lw=1.0))
    ax.plot([cx, px[i]], [cy, py[i]], color=CONN, lw=1.2)
    ax.scatter([px[i]], [py[i]], color=CONN, s=26, zorder=5)
    ax.text(px[i] + 0.6, py[i] + 0.8, "look-ahead point", fontsize=7.5, color=CONN)
    ax.annotate(
        "",
        xy=(cx + 7, cy + 0.2),
        xytext=(cx, cy),
        arrowprops={"arrowstyle": "-|>", "color": INK, "lw": 1.2},
    )
    ax.text(cx + 3.4, cy - 1.8, "heading", fontsize=7.5, color=INK)
    ax.text(cx + 5.2, cy + 4.2, "alpha", fontsize=8.5, color="#b3261e")
    ax.text(0.5, 8.0, r"$\delta=\arctan\left(2L_{wb}\sin\alpha\,/\,\ell_d\right)$", fontsize=9.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Pure-pursuit steering geometry", fontsize=9.5)
    save(fig, "pure_pursuit.png")


def fig_schedule():
    phases = [
        ("P0  Study, survey, architecture, CI", 0, 2),
        ("P1  Planning core (graph, A*)", 2, 2),
        ("P2  Map tooling, real-city import", 4, 4),
        ("P3  Lane detection", 7, 1),
        ("P4  Web service and simulator", 8, 2),
        ("P5  Browser front end", 9, 2),
        ("P6  Autoware evaluation", 10, 2),
        ("P7  Deploy, measure, report", 12, 2),
    ]
    fig, ax = plt.subplots(figsize=(6.6, 2.8))
    for i, (_, start, dur) in enumerate(phases):
        ax.barh(i, dur, left=start, color=ROUTE if i % 2 == 0 else GREEN, alpha=0.85, height=0.6)
    ax.set_yticks(range(len(phases)))
    ax.set_yticklabels([p[0] for p in phases], fontsize=8)
    ax.invert_yaxis()
    ax.set_xticks([w + 0.5 for w in range(14)])
    ax.set_xticklabels([str(w + 1) for w in range(14)], fontsize=7.5)
    ax.set_xlim(0, 14)
    ax.set_xlabel("week of the semester (January to April 2026)")
    ax.axvline(6.6, color=CONN, ls=(0, (4, 3)), lw=1.0)
    ax.text(6.7, 0.2, "Phase-1 review", fontsize=7.5, color=CONN)
    ax.axvline(12.6, color="#b3261e", ls=(0, (4, 3)), lw=1.0)
    ax.text(12.7, 0.2, "Phase-2 review", fontsize=7.5, color="#b3261e")
    ax.set_title("Project work schedule", fontsize=9.5)
    save(fig, "schedule.png")


def fig_route_map(svc, lanelets, rng):
    best = None
    ids = list(lanelets)
    for _ in range(60):
        ca = lanelets[rng.choice(ids)].centerline
        cb = lanelets[rng.choice(ids)].centerline
        p = svc.projector.to_latlng(ca[len(ca) // 2])
        q = svc.projector.to_latlng(cb[len(cb) // 2])
        try:
            _, stored = svc.plan(p, q, CostKind.TRAVEL_TIME)
        except Exception:
            continue
        r = stored.route
        score = r.distance_m + 400 * r.lane_changes
        if best is None or score > best[0]:
            best = (score, r)
    route = best[1]
    fig, ax = plt.subplots(figsize=(6.0, 6.0))
    for ll in lanelets.values():
        pts = ll.centerline
        ax.plot(
            [p.x for p in pts],
            [p.y for p in pts],
            color="#b9c0c9" if not ll.is_connector else "#d5dae0",
            lw=0.55,
        )
    ax.plot([p.x for p in route.centerline], [p.y for p in route.centerline], color=ROUTE, lw=2.0)
    start, goal = route.centerline[0], route.centerline[-1]
    ax.scatter([start.x], [start.y], color=GREEN, s=34, zorder=5, label="start")
    ax.scatter([goal.x], [goal.y], color="#b3261e", s=34, zorder=5, label="goal")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(
        f"Planned lane-level route: {len(route.lanelet_ids)} lanelets, {route.distance_m:,.0f} m, "
        f"{route.lane_changes} lane changes,{NL}ETA {route.eta_s:,.0f} s over the "
        "2,117-lanelet routable core",
        fontsize=9,
    )
    save(fig, "route_map.png")


def measure_tracking(svc, lanelets, rng):
    """Cross-track error of the simulated vehicle over 80 random routes."""
    ids = sorted(lanelets)
    routes = []
    for _ in range(80):
        ca = lanelets[rng.choice(ids)].centerline
        cb = lanelets[rng.choice(ids)].centerline
        p = svc.projector.to_latlng(ca[len(ca) // 2])
        q = svc.projector.to_latlng(cb[len(cb) // 2])
        routes.append(svc.plan(p, q, CostKind.TRAVEL_TIME)[1])
    ctes, arrived = [], 0
    for stored in routes:
        sim = VehicleSimulator(stored.route.centerline, stored.speed_limits_mps)
        while True:
            f = sim.step(0.1)
            if f.state is not SimState.DRIVING:
                break
            ctes.append(f.cte_m)
        arrived += f.state is SimState.ARRIVED
    print("arrived", arrived, "of", len(routes), flush=True)
    return ctes


def fig_latency():
    fig, ax = plt.subplots(figsize=(4.4, 2.6))
    stages = ["snap" + NL + "(2 clicks)", "A*" + NL + "search", "stitch +" + NL + "trim", "total"]
    vals = [34.8, 8.4, 2.6, 45.8]
    bars = ax.bar(stages, vals, color=[CONN, ROUTE, GREEN, "#445061"], alpha=0.9)
    ax.bar_label(bars, fmt="%.1f ms", fontsize=7.5, padding=2)
    ax.set_ylabel("median time (ms)")
    ax.set_ylim(0, 55)
    ax.set_title("Where the median planning request spends its time", fontsize=9.5)
    save(fig, "latency.png")


def fig_expansions():
    fig, ax = plt.subplots(figsize=(4.2, 2.6))
    labels = ["A* (straight-line" + NL + "heuristic)", "Dijkstra" + NL + "(h = 0)"]
    bars = ax.bar(labels, [359, 1004], color=[ROUTE, LANE], alpha=0.9, width=0.55)
    ax.bar_label(bars, fmt="%d nodes", fontsize=8, padding=2)
    ax.set_ylabel("median nodes expanded")
    ax.set_ylim(0, 1200)
    ax.set_title(
        "Search effort on 150 real routes" + NL + "(identical route cost in 150/150 cases)",
        fontsize=9,
    )
    save(fig, "expansions.png")


def fig_concurrency():
    fig, ax1 = plt.subplots(figsize=(4.6, 2.7))
    conc = [1, 4, 16, 64]
    p50 = [44.7, 183.4, 771.0, 3192.2]
    rps = [20.94, 21.77, 19.86, 19.38]
    ax1.plot(conc, p50, marker="o", color=ROUTE, lw=1.5)
    ax1.set_xscale("log", base=2)
    ax1.set_yscale("log")
    ax1.set_xticks(conc)
    ax1.set_xticklabels([str(c) for c in conc])
    ax1.set_xlabel("concurrent /plan requests")
    ax1.set_ylabel("median latency (ms)", color=ROUTE)
    ax2 = ax1.twinx()
    ax2.grid(False)
    ax2.plot(conc, rps, marker="s", color=CONN, lw=1.5, ls=(0, (4, 3)))
    ax2.set_ylabel("throughput (requests/s)", color=CONN)
    ax2.set_ylim(0, 26)
    ax1.set_title(
        "Throughput saturates near 20 plans/s while latency grows"
        + NL
        + "in proportion to offered concurrency (single process)",
        fontsize=9,
    )
    save(fig, "concurrency.png")


def fig_ws():
    fig, ax1 = plt.subplots(figsize=(4.6, 2.7))
    drives = [1, 50, 200, 500]
    hz = [9.03, 9.18, 8.97, 8.93]
    cpu = [0.01, 0.09, 0.28, 0.68]
    ax1.plot(drives, hz, marker="o", color=GREEN, lw=1.5)
    ax1.axhline(10.0, color=LANE, ls=(0, (4, 3)), lw=1.0)
    ax1.text(1.5, 10.15, "nominal 10 Hz", fontsize=7, color="#6b7885")
    ax1.set_xscale("log")
    ax1.set_xticks(drives)
    ax1.set_xticklabels([str(d) for d in drives])
    ax1.set_ylim(0, 11.5)
    ax1.set_xlabel("concurrent simulated drives")
    ax1.set_ylabel("telemetry rate (frames/s)", color=GREEN)
    ax2 = ax1.twinx()
    ax2.grid(False)
    ax2.plot(drives, cpu, marker="s", color=CONN, lw=1.5, ls=(0, (4, 3)))
    ax2.set_ylabel("server CPU (cores)", color=CONN)
    ax2.set_ylim(0, 0.8)
    ax1.set_title(
        "Telemetry rate holds while server CPU grows" + NL + "linearly with concurrent drives",
        fontsize=9,
    )
    save(fig, "ws.png")


def fig_cte(ctes):
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
    axes[0].hist(ctes, bins=60, color=ROUTE, alpha=0.85, edgecolor="white", linewidth=0.3)
    axes[0].set_yscale("log")
    axes[0].set_xlabel("cross-track error (m)")
    axes[0].set_ylabel("telemetry frames")
    axes[0].set_title("(a) Distribution of path-tracking error")
    xs = np.sort(np.array(ctes))
    axes[1].plot(xs, np.arange(1, len(xs) + 1) / len(xs) * 100, color=ROUTE, lw=1.6)
    for pct in (50, 95, 99):
        q = np.percentile(xs, pct)
        axes[1].axvline(q, color=CONN, ls=(0, (3, 3)), lw=0.9)
        axes[1].text(
            q, 30, f" p{pct} = {q:.2f} m", fontsize=7, color=CONN, rotation=90, va="bottom"
        )
    axes[1].set_xlabel("cross-track error (m)")
    axes[1].set_ylabel("frames below (%)")
    axes[1].set_xlim(0, max(1.0, float(np.percentile(xs, 99.9))))
    axes[1].set_title("(b) Cumulative distribution")
    save(fig, "cte.png")


def fig_coverage():
    fig, ax = plt.subplots(figsize=(5.4, 2.5))
    pkgs = ["avcore", "avmap_tools", "avlane", "webdemo_backend"]
    tests = [56, 43, 9, 36]
    cov = [97.6, 87.7, 73.6, 94.6]
    x = np.arange(len(pkgs))
    b1 = ax.bar(x - 0.2, tests, width=0.38, color=ROUTE, alpha=0.9)
    ax.bar_label(b1, fontsize=7.5, padding=2)
    ax.set_ylabel("number of tests", color=ROUTE)
    ax.set_ylim(0, 70)
    ax.set_xticks(x)
    ax.set_xticklabels(pkgs, fontsize=8)
    ax2 = ax.twinx()
    ax2.grid(False)
    b2 = ax2.bar(x + 0.2, cov, width=0.38, color=GREEN, alpha=0.85)
    ax2.bar_label(b2, fmt="%.0f%%", fontsize=7.5, padding=2)
    ax2.set_ylabel("line coverage (%)", color=GREEN)
    ax2.set_ylim(0, 115)
    ax.set_title("Test suite: 144 tests, 90.08 % overall line coverage", fontsize=9.5)
    save(fig, "tests.png")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "DejaVu Serif"],
            "font.size": 9,
            "axes.titlesize": 10,
            "figure.dpi": 220,
            "savefig.bbox": "tight",
            "axes.grid": True,
            "grid.alpha": 0.25,
        }
    )
    print("loading map ...", flush=True)
    t0 = time.perf_counter()
    svc = MapService.from_osm_artifact(str(REPO / "maps" / "barcelona_eixample.osm"))
    print(f"map loaded in {time.perf_counter() - t0:.2f} s", flush=True)
    lanelets = svc._lanelets
    rng = random.Random(11)

    fig_architecture()
    fig_lane_expansion()
    fig_junction(lanelets)
    fig_graph_edges()
    fig_lane_change()
    fig_pure_pursuit()
    fig_schedule()
    fig_route_map(svc, lanelets, rng)
    ctes = measure_tracking(svc, lanelets, rng)
    fig_latency()
    fig_expansions()
    fig_concurrency()
    fig_ws()
    fig_cte(ctes)
    fig_coverage()
    print(
        f"cross-track error: mean {np.mean(ctes):.3f} m, p95 {np.percentile(ctes, 95):.3f} m, "
        f"max {np.max(ctes):.3f} m over {len(ctes)} frames"
    )


if __name__ == "__main__":
    main()
