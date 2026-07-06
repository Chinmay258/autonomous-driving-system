import pytest
from avmap_tools import build_graph, polylines_to_lanelets

from avcore import EdgeKind, LaneletId


def test_three_boundaries_make_two_adjacent_lanes() -> None:
    boundaries = [
        [(-3.7, 0.0), (-3.7, 30.0)],
        [(0.0, 0.0), (0.0, 30.0)],
        [(3.7, 0.0), (3.7, 30.0)],
    ]
    lanelets = polylines_to_lanelets(boundaries)
    assert len(lanelets) == 2
    assert lanelets[0].right_bound == lanelets[1].left_bound  # shared divider
    graph = build_graph(lanelets)
    kinds = {e.kind for e in graph.edges_from(LaneletId(1))}
    assert EdgeKind.RIGHT in kinds  # lane change derived automatically


def test_lane_geometry_sane() -> None:
    lanelets = polylines_to_lanelets([[(-1.85, 0.0), (-1.6, 30.0)], [(1.85, 0.0), (2.1, 30.0)]])
    assert lanelets[0].mean_width_m == pytest.approx(3.7, abs=0.1)
    assert lanelets[0].length_m == pytest.approx(30.0, abs=0.5)


def test_rejects_single_boundary() -> None:
    with pytest.raises(ValueError):
        polylines_to_lanelets([[(0.0, 0.0), (0.0, 30.0)]])
