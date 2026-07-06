"""avcore — the shared planning brain.

Pure logic: map model, routing graph, filtering, A* planner. This package must
never import web frameworks, ROS/Autoware, or OpenCV (ADR-0002). Both the
Autoware evaluation track and the live web demo import from here, which is what
guarantees the deployed demo runs the same algorithm validated in simulation.
"""

from avcore.errors import (
    AvCoreError,
    MapValidationError,
    UnknownLaneletError,
    UnreachableGoalError,
)
from avcore.filtering import filter_lanelets, max_curvature, rank_routes
from avcore.graph import Edge, EdgeKind, RoutingGraph
from avcore.models import Lanelet, LaneletId, LaneletSubtype, LatLng, Point2D, RouteResult
from avcore.planner import CostModel, Distance, TravelTime, plan_route
from avcore.routeops import trim_route

__all__ = [
    "AvCoreError",
    "CostModel",
    "Distance",
    "Edge",
    "EdgeKind",
    "Lanelet",
    "LaneletId",
    "LaneletSubtype",
    "LatLng",
    "MapValidationError",
    "Point2D",
    "RouteResult",
    "RoutingGraph",
    "TravelTime",
    "UnknownLaneletError",
    "UnreachableGoalError",
    "filter_lanelets",
    "max_curvature",
    "plan_route",
    "rank_routes",
    "trim_route",
]
