"""Map + planning service for the web demo.

Owns the map artifact (synthetic town for now, OSM-derived later — same
interface), the routing graph, coordinate projection, click snapping, and a
bounded in-memory route store. All planning goes through avcore (ADR-0002).
"""

from __future__ import annotations

import secrets
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

from avmap_tools import LocalProjector, build_graph, read_osm, synthetic_town, validate_map

from avcore import (
    Distance,
    LaneletId,
    LatLng,
    MapValidationError,
    Point2D,
    RouteResult,
    TravelTime,
    filter_lanelets,
    plan_route,
)
from avcore.graph import RoutingGraph
from avcore.models import Lanelet
from webdemo_backend.schemas import CostKind

# Must exceed half an Eixample block diagonal (~70 m): a click in the middle
# of a large city block should still snap to the nearest surrounding street.
MAX_SNAP_DISTANCE_M = 100.0
MAX_STORED_ROUTES = 1000
DEFAULT_ORIGIN = LatLng(lat=43.7384, lng=7.4246)  # Monaco


class SnapError(Exception):
    """Click could not be matched to a drivable lanelet."""


@dataclass(frozen=True, slots=True)
class StoredRoute:
    route: RouteResult
    speed_limits_mps: tuple[float, ...]  # parallel to route.centerline


class MapService:
    def __init__(self, lanelets: list[Lanelet], origin: LatLng) -> None:
        validate_map(lanelets).raise_if_failed()
        # Restrict to the sane, largest connected component: real-map imports
        # can carry bbox-clipped stubs that would strand the vehicle.
        self._graph: RoutingGraph = filter_lanelets(build_graph(lanelets))
        if len(self._graph) == 0:
            raise MapValidationError("no drivable connected lanelets after filtering")
        self._lanelets = {lid: self._graph.lanelet(lid) for lid in self._graph}
        self.projector = LocalProjector(origin)
        self.origin = origin
        # Snap index: centerline segments (vertex-only snapping fails on long
        # straight blocks where vertices are ~100 m apart).
        self._snap_segments: list[tuple[Point2D, Point2D, LaneletId]] = [
            (a, b, ll.id)
            for ll in self._lanelets.values()
            for a, b in zip(ll.centerline, ll.centerline[1:], strict=False)
        ]
        self._routes: OrderedDict[str, StoredRoute] = OrderedDict()

    @classmethod
    def from_synthetic_town(cls, blocks: int = 3, origin: LatLng = DEFAULT_ORIGIN) -> MapService:
        return cls(synthetic_town(blocks, blocks), origin)

    @classmethod
    def from_osm_artifact(cls, path: str) -> MapService:
        """Load the canonical Lanelet2 artifact produced by avmap_tools."""
        lanelets, origin = read_osm(Path(path).read_text(encoding="utf-8"))
        return cls(lanelets, origin)

    @property
    def lanelet_count(self) -> int:
        return len(self._lanelets)

    def snap(self, coord: LatLng) -> LaneletId:
        """Nearest lanelet (by centerline segment distance) within snap radius."""
        point = self.projector.to_local(coord)
        best_id: LaneletId | None = None
        best_d = MAX_SNAP_DISTANCE_M
        for a, b, lanelet_id in self._snap_segments:
            abx, aby = b.x - a.x, b.y - a.y
            seg_len_sq = abx * abx + aby * aby
            if seg_len_sq == 0:
                d = a.distance_to(point)
            else:
                t = ((point.x - a.x) * abx + (point.y - a.y) * aby) / seg_len_sq
                t = max(0.0, min(1.0, t))
                d = Point2D(a.x + t * abx, a.y + t * aby).distance_to(point)
            if d < best_d:
                best_d = d
                best_id = lanelet_id
        if best_id is None:
            raise SnapError(f"no lanelet within {MAX_SNAP_DISTANCE_M:.0f} m of click")
        return best_id

    def plan(self, start: LatLng, goal: LatLng, cost: CostKind) -> tuple[str, StoredRoute]:
        """Snap both ends, plan, store, and return (route_id, stored route).

        Raises SnapError or UnreachableGoalError for the API layer to map to
        wire error codes.
        """
        start_id = self.snap(start)
        goal_id = self.snap(goal)
        model = Distance() if cost is CostKind.DISTANCE else TravelTime()
        route = plan_route(self._graph, start_id, goal_id, model)
        stored = StoredRoute(route=route, speed_limits_mps=self._speeds_for(route))
        route_id = f"r_{secrets.token_hex(8)}"
        self._routes[route_id] = stored
        while len(self._routes) > MAX_STORED_ROUTES:
            self._routes.popitem(last=False)
        return route_id, stored

    def _speeds_for(self, route: RouteResult) -> tuple[float, ...]:
        """Per-centerline-point speed limits, aligned with the stitched centerline."""
        speeds: list[float] = []
        seen = 0
        for lanelet_id in route.lanelet_ids:
            lanelet = self._lanelets[lanelet_id]
            for point in lanelet.centerline:
                if seen < len(route.centerline) and route.centerline[seen] == point:
                    speeds.append(lanelet.speed_limit_mps)
                    seen += 1
        # Stitching dedupes joint points; pad defensively if alignment drifted.
        while len(speeds) < len(route.centerline):
            speeds.append(speeds[-1] if speeds else 8.3)
        return tuple(speeds)

    def get_route(self, route_id: str) -> StoredRoute | None:
        return self._routes.get(route_id)

    def road_network_geojson(self) -> dict[str, object]:
        """All lanelet centerlines as a FeatureCollection (frontend base layer)."""
        features: list[dict[str, object]] = [
            {
                "type": "Feature",
                "properties": {"id": int(ll.id), "speed_limit_mps": ll.speed_limit_mps},
                "geometry": {
                    "type": "LineString",
                    "coordinates": self.to_geojson_coords(ll.centerline),
                },
            }
            for ll in self._lanelets.values()
        ]
        return {"type": "FeatureCollection", "features": features}

    def to_geojson_coords(self, points: tuple[Point2D, ...]) -> list[tuple[float, float]]:
        coords = []
        for point in points:
            ll = self.projector.to_latlng(point)
            coords.append((ll.lng, ll.lat))
        return coords
