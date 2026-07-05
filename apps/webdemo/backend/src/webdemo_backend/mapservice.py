"""Map + planning service for the web demo.

Owns the map artifact (synthetic town for now, OSM-derived later — same
interface), the routing graph, coordinate projection, click snapping, and a
bounded in-memory route store. All planning goes through avcore (ADR-0002).
"""

from __future__ import annotations

import secrets
from collections import OrderedDict
from dataclasses import dataclass

from avmap_tools import LocalProjector, build_graph, synthetic_town, validate_map

from avcore import (
    Distance,
    LaneletId,
    LatLng,
    Point2D,
    RouteResult,
    TravelTime,
    plan_route,
)
from avcore.graph import RoutingGraph
from avcore.models import Lanelet
from webdemo_backend.schemas import CostKind

MAX_SNAP_DISTANCE_M = 60.0
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
        self._lanelets = {ll.id: ll for ll in lanelets}
        self._graph: RoutingGraph = build_graph(lanelets)
        self.projector = LocalProjector(origin)
        self.origin = origin
        # Snap index: every centerline vertex of every lanelet.
        self._snap_points: list[tuple[Point2D, LaneletId]] = [
            (point, ll.id) for ll in lanelets for point in ll.centerline
        ]
        self._routes: OrderedDict[str, StoredRoute] = OrderedDict()

    @classmethod
    def from_synthetic_town(cls, blocks: int = 3, origin: LatLng = DEFAULT_ORIGIN) -> MapService:
        return cls(synthetic_town(blocks, blocks), origin)

    @property
    def lanelet_count(self) -> int:
        return len(self._lanelets)

    def snap(self, coord: LatLng) -> LaneletId:
        """Nearest lanelet to a clicked coordinate, within the snap radius."""
        point = self.projector.to_local(coord)
        best_id: LaneletId | None = None
        best_d = MAX_SNAP_DISTANCE_M
        for candidate, lanelet_id in self._snap_points:
            d = candidate.distance_to(point)
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

    def to_geojson_coords(self, points: tuple[Point2D, ...]) -> list[tuple[float, float]]:
        coords = []
        for point in points:
            ll = self.projector.to_latlng(point)
            coords.append((ll.lng, ll.lat))
        return coords
