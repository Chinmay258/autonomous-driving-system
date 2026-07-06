"""Host-side golden comparison: our A* vs Autoware's mission planner.

Subcommands:
  scenarios                       -> deterministic start/goal lanelet id pairs
  ours --start-id S --goal-id G   -> our planner's lanelet id sequence (JSON)
  compare --start-id S --goal-id G --golden '<GOLDEN_ROUTE json>'
                                  -> verdict comparing both planners

Run from repo root with:  uv run python apps/autoware_eval/compare_routes.py ...
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from avmap_tools import build_graph, read_osm

from avcore import LaneletId, TravelTime, filter_lanelets, plan_route

REPO = Path(__file__).resolve().parents[2]
ARTIFACT = REPO / "maps" / "barcelona_eixample.osm"


def load_graph():
    lanelets, origin = read_osm(ARTIFACT.read_text(encoding="utf-8"))
    return filter_lanelets(build_graph(lanelets)), origin


def pick_scenarios(graph) -> list[dict[str, int]]:
    """Deterministic scenario pairs spread across the map."""
    ids = sorted(graph)
    mids = {
        lid: graph.lanelet(lid).centerline[len(graph.lanelet(lid).centerline) // 2] for lid in ids
    }
    anchor = ids[0]
    by_dist = sorted(ids, key=lambda lid: mids[anchor].distance_to(mids[lid]))
    return [
        {"name": "cross_map", "start": int(anchor), "goal": int(by_dist[-1])},
        {"name": "mid_range", "start": int(anchor), "goal": int(by_dist[len(by_dist) // 2])},
        {"name": "short_hop", "start": int(anchor), "goal": int(by_dist[len(by_dist) // 10])},
    ]


def our_route(graph, start: int, goal: int) -> list[int]:
    result = plan_route(graph, LaneletId(start), LaneletId(goal), TravelTime())
    return [int(lid) for lid in result.lanelet_ids]


def compare(ours: list[int], golden: dict) -> dict:
    preferred: list[int] = golden["preferred"]
    alternatives: list[list[int]] = golden["alternatives"]
    strict = ours == preferred
    # Section-tolerant: Autoware emits one segment per road section with all
    # parallel lanes as primitives; our per-lane sequence must thread through
    # those sections in order.
    i = 0
    threaded = 0
    for section in alternatives:
        while i < len(ours) and ours[i] in section:
            threaded += 1
            i += 1
    coverage = threaded / len(ours) if ours else 0.0
    return {
        "strict_match": strict,
        "our_steps": len(ours),
        "autoware_sections": len(preferred),
        "threaded_steps": threaded,
        "section_coverage": round(coverage, 3),
        "same_endpoints": bool(
            preferred and ours and ours[0] in alternatives[0] and ours[-1] in alternatives[-1]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scenarios")
    p_ours = sub.add_parser("ours")
    p_ours.add_argument("--start-id", type=int, required=True)
    p_ours.add_argument("--goal-id", type=int, required=True)
    p_cmp = sub.add_parser("compare")
    p_cmp.add_argument("--start-id", type=int, required=True)
    p_cmp.add_argument("--goal-id", type=int, required=True)
    p_cmp.add_argument("--golden", required=True, help="GOLDEN_ROUTE JSON payload")
    args = parser.parse_args()

    graph, origin = load_graph()
    if args.cmd == "scenarios":
        print(
            json.dumps(
                {
                    "origin": {"lat": origin.lat, "lng": origin.lng},
                    "scenarios": pick_scenarios(graph),
                }
            )
        )
        return 0
    if args.cmd == "ours":
        print(json.dumps(our_route(graph, args.start_id, args.goal_id)))
        return 0
    ours = our_route(graph, args.start_id, args.goal_id)
    verdict = compare(ours, json.loads(args.golden))
    print(json.dumps({"ours": ours, **verdict}, indent=2))
    return 0 if verdict["same_endpoints"] else 1


if __name__ == "__main__":
    sys.exit(main())
