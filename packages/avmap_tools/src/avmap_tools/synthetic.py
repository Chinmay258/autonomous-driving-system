"""Synthetic grid town: two-way streets, right-hand traffic, turn connectors.

Deterministic generator for a Manhattan-style town used as the demo map and
as the Autoware evaluation map until a real OSM-derived map replaces it. All
streets are two-way single-lane; every intersection gets straight/left/right
connectors (no U-turns), which makes the routing graph strongly connected.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from avcore import Lanelet, LaneletId, Point2D

LANE_WIDTH_M = 3.5
HALF_LANE = LANE_WIDTH_M / 2.0
INTERSECTION_HALF_M = 8.0  # street segments stop this far from the crossing point
STREET_SPEED_MPS = 13.9  # 50 km/h
TURN_SPEED_MPS = 8.3  # 30 km/h


@dataclass
class _Builder:
    next_id: int = 1
    lanelets: list[Lanelet] = field(default_factory=list)

    def lane(self, start: Point2D, end: Point2D, *, speed: float = STREET_SPEED_MPS) -> Lanelet:
        """Straight lane from start to end; bounds offset by the left-hand normal."""
        dx, dy = end.x - start.x, end.y - start.y
        norm = (dx * dx + dy * dy) ** 0.5
        # Unit normal pointing to the driver's left.
        nx, ny = -dy / norm, dx / norm
        lanelet = Lanelet(
            id=LaneletId(self.next_id),
            left_bound=(
                Point2D(start.x + nx * HALF_LANE, start.y + ny * HALF_LANE),
                Point2D(end.x + nx * HALF_LANE, end.y + ny * HALF_LANE),
            ),
            right_bound=(
                Point2D(start.x - nx * HALF_LANE, start.y - ny * HALF_LANE),
                Point2D(end.x - nx * HALF_LANE, end.y - ny * HALF_LANE),
            ),
            speed_limit_mps=speed,
        )
        self.next_id += 1
        self.lanelets.append(lanelet)
        return lanelet

    def connector(self, source: Lanelet, target: Lanelet) -> Lanelet:
        """Bridge from source's end cross-section to target's start cross-section."""
        lanelet = Lanelet(
            id=LaneletId(self.next_id),
            left_bound=(source.left_bound[-1], target.left_bound[0]),
            right_bound=(source.right_bound[-1], target.right_bound[0]),
            speed_limit_mps=TURN_SPEED_MPS,
            is_connector=True,
        )
        self.next_id += 1
        self.lanelets.append(lanelet)
        return lanelet


def synthetic_town(blocks_x: int = 3, blocks_y: int = 3, *, block_m: float = 80.0) -> list[Lanelet]:
    """Generate the town. Intersections sit at (c*block_m, r*block_m)."""
    if blocks_x < 1 or blocks_y < 1:
        raise ValueError("need at least a 1x1 block grid")
    if block_m <= 2 * INTERSECTION_HALF_M + 2 * LANE_WIDTH_M:
        raise ValueError("block_m too small for intersection geometry")

    b = _Builder()
    gap = INTERSECTION_HALF_M

    # Street segments. Right-hand traffic: eastbound rides south of the axis,
    # westbound north; northbound east of the axis, southbound west.
    h_east: dict[tuple[int, int], Lanelet] = {}
    h_west: dict[tuple[int, int], Lanelet] = {}
    v_north: dict[tuple[int, int], Lanelet] = {}
    v_south: dict[tuple[int, int], Lanelet] = {}

    for r in range(blocks_y + 1):
        y = r * block_m
        for c in range(blocks_x):
            x0, x1 = c * block_m + gap, (c + 1) * block_m - gap
            h_east[(r, c)] = b.lane(Point2D(x0, y - HALF_LANE), Point2D(x1, y - HALF_LANE))
            h_west[(r, c)] = b.lane(Point2D(x1, y + HALF_LANE), Point2D(x0, y + HALF_LANE))
    for c in range(blocks_x + 1):
        x = c * block_m
        for r in range(blocks_y):
            y0, y1 = r * block_m + gap, (r + 1) * block_m - gap
            v_north[(c, r)] = b.lane(Point2D(x + HALF_LANE, y0), Point2D(x + HALF_LANE, y1))
            v_south[(c, r)] = b.lane(Point2D(x - HALF_LANE, y1), Point2D(x - HALF_LANE, y0))

    # Intersection connectors: straight + left + right for every incoming lane.
    for ci in range(blocks_x + 1):
        for ri in range(blocks_y + 1):
            east_in = h_east.get((ri, ci - 1))
            west_in = h_west.get((ri, ci))
            north_in = v_north.get((ci, ri - 1))
            south_in = v_south.get((ci, ri))
            east_out = h_east.get((ri, ci))
            west_out = h_west.get((ri, ci - 1))
            north_out = v_north.get((ci, ri))
            south_out = v_south.get((ci, ri - 1))

            movements = (
                (east_in, (east_out, north_out, south_out)),
                (west_in, (west_out, south_out, north_out)),
                (north_in, (north_out, west_out, east_out)),
                (south_in, (south_out, east_out, west_out)),
            )
            for incoming, outgoings in movements:
                if incoming is None:
                    continue
                for outgoing in outgoings:
                    if outgoing is not None:
                        b.connector(incoming, outgoing)

    return b.lanelets
