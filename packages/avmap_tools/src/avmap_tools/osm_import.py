"""Real-city import: raw OpenStreetMap highways -> per-lane lanelets.

Every OSM way of a drivable class is split at intersection nodes, each split
segment is expanded into one lanelet per marked lane (using `lanes`,
`lanes:forward/backward`, `oneway`, `maxspeed` tags with per-class defaults),
and intersections get connector lanelets with **strict lane discipline**:

- straight movements preserve lane index (lane k -> lane k),
- right turns are allowed only from the rightmost lane into the rightmost lane,
- left turns only from the leftmost lane into the leftmost lane,
- U-turns are never generated.

Adjacent same-direction lanes share their divider polyline object, so
``build_graph`` derives LEFT/RIGHT lane-change edges automatically; opposite
directions never share a divider and therefore never get lane-change edges.

Turn-restriction relations are not yet honored (tracked for a later phase).
Pure stdlib; no osmnx/networkx.
"""

from __future__ import annotations

import itertools
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from avcore import Lanelet, LaneletId, LatLng, Point2D
from avmap_tools.projection import LocalProjector

LANE_WIDTH_M = 3.5
# Deep enough setback that turn fillets get a driveable radius even on the
# acute crossings (Autoware's behavior planner emits degenerate paths on
# arcs much tighter than ~4-5 m).
INTERSECTION_TRIM_M = 12.0
TURN_FILLET_RADIUS_M = 5.0
TURN_FILLET_MIN_RADIUS_M = 2.5
MIN_SEGMENT_LENGTH_M = 4.0
TURN_SPEED_MPS = 8.3
UTURN_SPEED_MPS = 5.5
UTURN_MAX_GAP_M = 15.0  # U-turns only back onto the same street's other side
STRAIGHT_MAX_RAD = math.pi / 6  # |delta| below this is a straight continuation
UTURN_MIN_RAD = 5 * math.pi / 6  # |delta| above this is a U-turn

_DRIVABLE = re.compile(
    r"^(motorway|trunk|primary|secondary|tertiary|residential|unclassified)(_link)?$"
)
_CLASS_SPEED_MPS = {
    "motorway": 27.8,
    "motorway_link": 13.9,
    "trunk": 22.2,
    "trunk_link": 13.9,
    "primary": 16.7,
    "primary_link": 11.1,
    "secondary": 13.9,
    "secondary_link": 11.1,
    "tertiary": 11.1,
    "tertiary_link": 11.1,
    "residential": 8.3,
    "unclassified": 8.3,
}
_CLASS_LANES_TOTAL = {"motorway": 3, "trunk": 2, "primary": 2, "secondary": 2}


@dataclass(frozen=True, slots=True)
class _Way:
    id: int
    nodes: tuple[int, ...]
    tags: dict[str, str]


@dataclass(frozen=True, slots=True)
class _DirectedLanes:
    """One driving direction of one street segment, expanded into lanes."""

    lanes: tuple[Lanelet, ...]  # index 0 = leftmost in driving direction
    start_node: int
    end_node: int
    speed_mps: float

    def heading_out(self) -> float:
        a, b = self.lanes[0].centerline[-2], self.lanes[0].centerline[-1]
        return math.atan2(b.y - a.y, b.x - a.x)

    def heading_in(self) -> float:
        a, b = self.lanes[0].centerline[0], self.lanes[0].centerline[1]
        return math.atan2(b.y - a.y, b.x - a.x)


def parse_maxspeed(value: str | None, highway_class: str) -> float:
    """maxspeed tag ('50', '50 km/h', '30 mph') -> m/s, with class fallback."""
    fallback = _CLASS_SPEED_MPS.get(highway_class, 8.3)
    if not value:
        return fallback
    match = re.match(r"^\s*(\d+(?:\.\d+)?)\s*(mph)?", value)
    if not match:
        return fallback
    speed = float(match.group(1))
    return speed * 0.44704 if match.group(2) else speed / 3.6


def lane_counts(tags: dict[str, str]) -> tuple[int, int]:
    """(forward, backward) lane counts from OSM tags + class defaults."""
    highway = tags.get("highway", "")
    base = highway.removesuffix("_link")
    oneway = tags.get("oneway", "no") in {"yes", "1", "true"}
    default_total = _CLASS_LANES_TOTAL.get(base, 1) * (1 if oneway else 2)
    try:
        total = max(1, int(tags["lanes"]))
    except (KeyError, ValueError):
        total = default_total
    if oneway:
        return total, 0
    try:
        fwd = max(1, int(tags["lanes:forward"]))
    except (KeyError, ValueError):
        fwd = max(1, total - total // 2)
    try:
        bwd = max(1, int(tags["lanes:backward"]))
    except (KeyError, ValueError):
        bwd = max(1, total - fwd)
    return fwd, bwd


def _offset_polyline(points: list[Point2D], d: float) -> tuple[Point2D, ...]:
    """Parallel offset by d along the left-hand normal (miter joints, capped)."""
    seg_normals: list[tuple[float, float]] = []
    for a, b in itertools.pairwise(points):
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        seg_normals.append((-dy / length, dx / length))
    out: list[Point2D] = []
    for i, p in enumerate(points):
        if i == 0:
            nx, ny = seg_normals[0]
            scale = 1.0
        elif i == len(points) - 1:
            nx, ny = seg_normals[-1]
            scale = 1.0
        else:
            ax, ay = seg_normals[i - 1]
            bx, by = seg_normals[i]
            sx, sy = ax + bx, ay + by
            norm = math.hypot(sx, sy)
            if norm < 1e-9:  # 180-degree spike; fall back to segment normal
                nx, ny = bx, by
                scale = 1.0
            else:
                nx, ny = sx / norm, sy / norm
                scale = 1.0 / max(0.5, nx * bx + ny * by)  # miter, capped at 2x
        out.append(Point2D(p.x + nx * d * scale, p.y + ny * d * scale))
    return tuple(out)


def _polyline_trim(points: list[Point2D], trim_start: float, trim_end: float) -> list[Point2D]:
    """Cut arclength off both ends (assumes total length > trims + epsilon)."""

    def cut_front(pts: list[Point2D], amount: float) -> list[Point2D]:
        if amount <= 0:
            return pts
        remaining = amount
        for i in range(len(pts) - 1):
            seg = pts[i].distance_to(pts[i + 1])
            if remaining < seg:
                t = remaining / seg
                first = Point2D(
                    pts[i].x + t * (pts[i + 1].x - pts[i].x),
                    pts[i].y + t * (pts[i + 1].y - pts[i].y),
                )
                return [first, *pts[i + 1 :]]
            remaining -= seg
        return pts[-2:]

    trimmed = cut_front(points, trim_start)
    trimmed = list(reversed(cut_front(list(reversed(trimmed)), trim_end)))
    return trimmed


def _dedupe(points: list[Point2D]) -> list[Point2D]:
    out: list[Point2D] = []
    for p in points:
        if not out or out[-1].distance_to(p) > 1e-6:
            out.append(p)
    return out


DENSIFY_STEP_M = 10.0


def _densify(points: list[Point2D], max_step: float = DENSIFY_STEP_M) -> list[Point2D]:
    """Insert intermediate vertices so no segment exceeds ``max_step``.

    Real Lanelet2 maps carry dense geometry; Autoware's behavior planner
    resamples from map vertices and degenerates on our raw 2-point,
    hundreds-of-meters straight bounds (reference path collapsed to ~10 m).
    """
    out = [points[0]]
    for a, b in itertools.pairwise(points):
        span = a.distance_to(b)
        steps = max(1, math.ceil(span / max_step))
        for i in range(1, steps + 1):
            t = i / steps
            out.append(Point2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)))
    return out


def _heading(a: Point2D, b: Point2D) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def _folds(bound: list[Point2D], center: list[Point2D]) -> bool:
    """True if any bound segment runs against its centerline segment (a fold:
    the offset self-intersected because the arc radius dipped below the
    offset distance). lanelet2 'fixes' such bounds by inverting them, which
    silently breaks succession — so folded connectors must never be written."""
    for i in range(min(len(bound), len(center)) - 1):
        bx, by = bound[i + 1].x - bound[i].x, bound[i + 1].y - bound[i].y
        cx, cy = center[i + 1].x - center[i].x, center[i + 1].y - center[i].y
        if bx * cx + by * cy <= 0:
            return True
    return False


def _uturn_arc(a: Point2D, heading_a: float, b: Point2D, samples: int = 14) -> list[Point2D]:
    """Teardrop U-turn: a circular sweep through the junction.

    A cubic hairpin between antiparallel lanes 3.5 m apart has an apex radius
    of ~half the chord, which folds the inner offset bound. A circle of
    radius >= 3 m centered ahead of the median keeps the inner bound radius
    positive everywhere — the shape a real U-turn drives.
    """
    chord = a.distance_to(b)
    radius = max(3.0, chord / 2.0 + 1.2)
    fx, fy = math.cos(heading_a), math.sin(heading_a)
    forward = math.sqrt(max(radius * radius - (chord / 2.0) ** 2, 0.25))
    cx = (a.x + b.x) / 2.0 + fx * forward
    cy = (a.y + b.y) / 2.0 + fy * forward
    ang_a = math.atan2(a.y - cy, a.x - cx)
    ang_b = math.atan2(b.y - cy, b.x - cx)
    # Pick the rotation whose tangent at `a` points along the entry heading.
    tangent_ccw = (-math.sin(ang_a), math.cos(ang_a))
    ccw = (tangent_ccw[0] * fx + tangent_ccw[1] * fy) > 0
    sweep = (ang_b - ang_a) % (2 * math.pi) if ccw else -((ang_a - ang_b) % (2 * math.pi))
    pts = [
        Point2D(
            cx + radius * math.cos(ang_a + sweep * i / samples),
            cy + radius * math.sin(ang_a + sweep * i / samples),
        )
        for i in range(samples + 1)
    ]
    return _dedupe([a, *pts[1:-1], b])


def _corner_fillet(
    a: Point2D,
    heading_a: float,
    b: Point2D,
    heading_b: float,
    *,
    target_radius: float = TURN_FILLET_RADIUS_M,
    min_radius: float = TURN_FILLET_MIN_RADIUS_M,
    samples: int = 12,
) -> list[Point2D] | None:
    """Road-engineering corner: circular arc tangent to entry AND exit rays.

    straight(a->ta) + arc(ta->tb, radius r) + straight(tb->b), with r shrunk
    to fit the available leg lengths (never below ``min_radius``). Returns
    None when the ray geometry doesn't admit a tangent corner.
    """
    delta = _signed_angle(heading_a, heading_b)
    interior = math.pi - abs(delta)
    if interior < 0.15:  # ~U-turn; handled by _uturn_arc
        return None
    dax, day = math.cos(heading_a), math.sin(heading_a)
    dbx, dby = math.cos(heading_b), math.sin(heading_b)
    det = dax * dby - day * dbx
    if abs(det) < 1e-9:
        return None
    t = ((b.x - a.x) * dby - (b.y - a.y) * dbx) / det
    u = (dax * (b.y - a.y) - day * (b.x - a.x)) / det
    if t <= 0.5 or u <= 0.5:
        return None
    tan_half = math.tan(interior / 2.0)
    radius = min(target_radius, 0.9 * t * tan_half, 0.9 * u * tan_half)
    if radius < min_radius:
        return None
    leg = radius / tan_half
    corner = Point2D(a.x + t * dax, a.y + t * day)
    tangent_in = Point2D(corner.x - dax * leg, corner.y - day * leg)
    tangent_out = Point2D(corner.x + dbx * leg, corner.y + dby * leg)
    side = 1.0 if delta > 0 else -1.0
    center_x = tangent_in.x - day * side * radius
    center_y = tangent_in.y + dax * side * radius
    ang_in = math.atan2(tangent_in.y - center_y, tangent_in.x - center_x)
    ang_out = math.atan2(tangent_out.y - center_y, tangent_out.x - center_x)
    sweep = _signed_angle(ang_in, ang_out)
    if side > 0 and sweep < 0:
        sweep += 2 * math.pi
    if side < 0 and sweep > 0:
        sweep -= 2 * math.pi
    arc = [
        Point2D(
            center_x + radius * math.cos(ang_in + sweep * i / samples),
            center_y + radius * math.sin(ang_in + sweep * i / samples),
        )
        for i in range(samples + 1)
    ]
    return _dedupe([a, *arc, b])


def _cubic_fillet(
    a: Point2D,
    heading_a: float,
    b: Point2D,
    heading_b: float,
    samples: int = 10,
    reach_mult: float = 1.0,
) -> list[Point2D]:
    """Road fillet: cubic bezier with controls along the entry/exit tangents.

    Tangent-continuous at both ends for any turn angle — straights stay
    straight, acute crossings sweep clean arcs, U-turns form a tight loop —
    with no special-case control-point logic to misbehave. The tangent reach
    grows with turn sharpness so acute turns get a driveable radius instead
    of a pointy apex.
    """
    gap = a.distance_to(b)
    turn = abs(_signed_angle(heading_a, heading_b))
    if turn < STRAIGHT_MAX_RAD:
        # Near-straight: keep controls well inside the chord — a reach floor
        # here makes control points cross on short gaps, kinking the curve so
        # its offset bounds fold into a bowtie (which lanelet2 then "fixes"
        # by inverting a bound, breaking succession).
        reach = 0.3 * gap * reach_mult
    else:
        reach = min(24.0, max(2.0, 0.45 * gap * (1.0 + turn / math.pi)) * reach_mult)
    c1 = Point2D(a.x + math.cos(heading_a) * reach, a.y + math.sin(heading_a) * reach)
    c2 = Point2D(b.x - math.cos(heading_b) * reach, b.y - math.sin(heading_b) * reach)
    pts = []
    for i in range(samples + 1):
        t = i / samples
        omt = 1.0 - t
        pts.append(
            Point2D(
                omt**3 * a.x + 3 * omt**2 * t * c1.x + 3 * omt * t**2 * c2.x + t**3 * b.x,
                omt**3 * a.y + 3 * omt**2 * t * c1.y + 3 * omt * t**2 * c2.y + t**3 * b.y,
            )
        )
    return _dedupe(pts)


@dataclass
class _Importer:
    drive_on: str = "right"
    next_id: int = 1
    lanelets: list[Lanelet] = field(default_factory=list)

    def _new_id(self) -> LaneletId:
        lid = LaneletId(self.next_id)
        self.next_id += 1
        return lid

    def expand_segment(
        self, geometry: list[Point2D], n_lanes: int, speed: float, *, centered: bool
    ) -> tuple[Lanelet, ...] | None:
        """One driving direction -> n lanelets. Geometry runs in driving direction.

        centered=True (oneway street): carriageway straddles the way centerline.
        centered=False (two-way): lanes occupy the right half (or left half when
        drive_on='left').
        """
        geometry = _dedupe(geometry)
        if len(geometry) < 2:
            return None
        geometry = _densify(geometry)
        side = -1.0 if self.drive_on == "right" else 1.0
        # Boundary j (j = 0..n) offset in lane widths from the way centerline;
        # boundary 0 is the leftmost marking in driving direction.
        if centered:
            offsets = [(n_lanes / 2.0 - j) * LANE_WIDTH_M * -side for j in range(n_lanes + 1)]
        else:
            offsets = [side * j * LANE_WIDTH_M for j in range(n_lanes + 1)]
        boundaries = [_offset_polyline(geometry, d) for d in offsets]
        lanes: list[Lanelet] = []
        for k in range(n_lanes):
            left, right = boundaries[k], boundaries[k + 1]
            if self.drive_on == "left" and not centered:
                left, right = boundaries[k + 1], boundaries[k]
            lanes.append(
                Lanelet(
                    id=self._new_id(),
                    left_bound=left,
                    right_bound=right,
                    speed_limit_mps=speed,
                )
            )
        return tuple(lanes)

    def connector(self, source: Lanelet, target: Lanelet, speed: float) -> None:
        """Curved junction connector (cubic fillet along the turn arc).

        Even near-touching ends get a micro-connector: lanelet2/Autoware
        succession needs exactly shared bound nodes, which only the connector
        provides (our own graph builder is tolerant; lanelet2 is not).
        """
        a, b = source.centerline[-1], target.centerline[0]
        gap = a.distance_to(b)
        if gap < 0.02:
            return  # genuinely coincident; shared nodes already exist
        heading_a = _heading(source.centerline[-2], source.centerline[-1])
        heading_b = _heading(target.centerline[0], target.centerline[1])
        turn = abs(_signed_angle(heading_a, heading_b))
        half = LANE_WIDTH_M / 2.0

        def build(center: list[Point2D]) -> tuple[list[Point2D], list[Point2D], bool]:
            left = list(_offset_polyline(center, half))
            right = list(_offset_polyline(center, -half))
            folded = _folds(left, center) or _folds(right, center)
            # Snap bound endpoints onto the street lanes' exact bound nodes:
            # lanelet2 derives succession from shared points, so the connector
            # must reuse them, not approximate them.
            left[0], left[-1] = source.left_bound[-1], target.left_bound[0]
            right[0], right[-1] = source.right_bound[-1], target.right_bound[0]
            return left, right, folded

        if turn >= UTURN_MIN_RAD:
            left, right, folded = build(_uturn_arc(a, heading_a, b))
        elif gap < 3.0 and turn < STRAIGHT_MAX_RAD:
            left, right, folded = build([a, b])
        else:
            # Prefer the tangent circular corner (guaranteed radius floor);
            # fall back to cubic easing, widened until no offset bound folds.
            folded = True
            corner = (
                _corner_fillet(a, heading_a, b, heading_b) if turn >= STRAIGHT_MAX_RAD else None
            )
            if corner is not None:
                left, right, folded = build(corner)
            if folded:
                for attempt in range(4):
                    center = _cubic_fillet(a, heading_a, b, heading_b, reach_mult=1.5**attempt)
                    left, right, folded = build(center)
                    if not folded:
                        break
            if folded:
                left, right, folded = build([a, b])  # straight degrade for turns
        if folded:
            return  # unbuildable without corrupting the map
        self.lanelets.append(
            Lanelet(
                id=self._new_id(),
                left_bound=tuple(_dedupe(left)),
                right_bound=tuple(_dedupe(right)),
                speed_limit_mps=speed,
                is_connector=True,
            )
        )


def import_osm_roads(
    xml_text: str, *, drive_on: str = "right", origin: LatLng | None = None
) -> tuple[list[Lanelet], LatLng]:
    """Parse raw OSM XML and synthesize the per-lane lanelet map."""
    if drive_on not in {"right", "left"}:
        raise ValueError("drive_on must be 'right' or 'left'")
    root = ET.fromstring(xml_text)

    coords: dict[int, LatLng] = {
        int(el.attrib["id"]): LatLng(lat=float(el.attrib["lat"]), lng=float(el.attrib["lon"]))
        for el in root.findall("node")
    }
    ways: list[_Way] = []
    for el in root.findall("way"):
        tags = {t.attrib["k"]: t.attrib["v"] for t in el.findall("tag")}
        highway = tags.get("highway", "")
        if not _DRIVABLE.match(highway) or tags.get("area") == "yes":
            continue
        node_ids = tuple(
            int(nd.attrib["ref"]) for nd in el.findall("nd") if int(nd.attrib["ref"]) in coords
        )
        if len(node_ids) < 2:
            continue
        if tags.get("oneway") == "-1":  # reversed oneway: normalize
            node_ids = node_ids[::-1]
            tags = {**tags, "oneway": "yes"}
        ways.append(_Way(id=int(el.attrib["id"]), nodes=node_ids, tags=tags))

    if not ways:
        raise ValueError("no drivable ways found in OSM extract")

    used = [nid for way in ways for nid in way.nodes]
    if origin is None:
        lats = [coords[n].lat for n in set(used)]
        lngs = [coords[n].lng for n in set(used)]
        origin = LatLng(lat=(min(lats) + max(lats)) / 2, lng=(min(lngs) + max(lngs)) / 2)
    projector = LocalProjector(origin)
    points = {nid: projector.to_local(coords[nid]) for nid in set(used)}

    node_way_count: dict[int, int] = {}
    for way in ways:
        for nid in set(way.nodes):
            node_way_count[nid] = node_way_count.get(nid, 0) + 1
    junctions = {nid for nid, count in node_way_count.items() if count >= 2}

    importer = _Importer(drive_on=drive_on)
    incoming: dict[int, list[_DirectedLanes]] = {}
    outgoing: dict[int, list[_DirectedLanes]] = {}

    for way in ways:
        # Split at internal junction nodes.
        cut_indices = (
            [0]
            + [i for i in range(1, len(way.nodes) - 1) if way.nodes[i] in junctions]
            + [len(way.nodes) - 1]
        )
        fwd_n, bwd_n = lane_counts(way.tags)
        speed = parse_maxspeed(way.tags.get("maxspeed"), way.tags.get("highway", ""))
        oneway = bwd_n == 0

        for a, b in itertools.pairwise(cut_indices):
            seg_nodes = way.nodes[a : b + 1]
            geometry = _dedupe([points[n] for n in seg_nodes])
            if len(geometry) < 2:
                continue
            length = sum(p.distance_to(q) for p, q in itertools.pairwise(geometry))
            if length < MIN_SEGMENT_LENGTH_M:
                # Sub-junction sliver (e.g. traffic-island node pairs); the
                # intersection connectors of its end nodes cover the gap.
                continue
            trim_start = INTERSECTION_TRIM_M if seg_nodes[0] in junctions else 0.0
            trim_end = INTERSECTION_TRIM_M if seg_nodes[-1] in junctions else 0.0
            if length - trim_start - trim_end < MIN_SEGMENT_LENGTH_M:
                spare = max(0.0, length - MIN_SEGMENT_LENGTH_M)
                scale = spare / (trim_start + trim_end) if trim_start + trim_end > 0 else 0.0
                trim_start *= scale
                trim_end *= scale
            geometry = _dedupe(_polyline_trim(geometry, trim_start, trim_end))
            if len(geometry) < 2:
                continue

            for reverse, n_lanes in ((False, fwd_n), (True, bwd_n)):
                if n_lanes == 0:
                    continue
                geom = list(reversed(geometry)) if reverse else list(geometry)
                lanes = importer.expand_segment(geom, n_lanes, speed, centered=oneway)
                if lanes is None:
                    continue
                importer.lanelets.extend(lanes)
                start_node = seg_nodes[-1] if reverse else seg_nodes[0]
                end_node = seg_nodes[0] if reverse else seg_nodes[-1]
                directed = _DirectedLanes(
                    lanes=lanes, start_node=start_node, end_node=end_node, speed_mps=speed
                )
                outgoing.setdefault(start_node, []).append(directed)
                incoming.setdefault(end_node, []).append(directed)

    _build_connectors(importer, incoming, outgoing)
    return importer.lanelets, origin


def _signed_angle(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _build_connectors(
    importer: _Importer,
    incoming: dict[int, list[_DirectedLanes]],
    outgoing: dict[int, list[_DirectedLanes]],
) -> None:
    for node, arrivals in sorted(incoming.items()):
        departures = outgoing.get(node, [])
        for arr in arrivals:
            for dep in departures:
                delta = _signed_angle(arr.heading_out(), dep.heading_in())
                if abs(delta) >= UTURN_MIN_RAD:
                    # U-turn: leftmost lane onto the opposite carriageway of
                    # the same street only (gap-gated), at crawl speed.
                    a = arr.lanes[0].centerline[-1]
                    b = dep.lanes[0].centerline[0]
                    if a.distance_to(b) <= UTURN_MAX_GAP_M:
                        importer.connector(arr.lanes[0], dep.lanes[0], UTURN_SPEED_MPS)
                    continue
                n_in, n_out = len(arr.lanes), len(dep.lanes)
                if abs(delta) < STRAIGHT_MAX_RAD:
                    speed = min(arr.speed_mps, dep.speed_mps)
                    for k in range(n_in):
                        importer.connector(arr.lanes[k], dep.lanes[min(k, n_out - 1)], speed)
                elif delta > 0:  # left turn: leftmost lane only
                    speed = min(arr.speed_mps, dep.speed_mps, TURN_SPEED_MPS)
                    importer.connector(arr.lanes[0], dep.lanes[0], speed)
                else:  # right turn: rightmost lane only
                    speed = min(arr.speed_mps, dep.speed_mps, TURN_SPEED_MPS)
                    importer.connector(arr.lanes[n_in - 1], dep.lanes[n_out - 1], speed)
