"""Local metric <-> WGS84 projection.

Equirectangular approximation anchored at an origin: exact enough (<< 1 cm
error) for the few-km maps this project uses, dependency-free, and trivially
invertible. Autoware gets the matching projector info via
``write_projector_info_yaml``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from avcore import LatLng, Point2D

METERS_PER_DEG_LAT = 111_320.0


@dataclass(frozen=True, slots=True)
class LocalProjector:
    origin: LatLng

    @property
    def _meters_per_deg_lng(self) -> float:
        return METERS_PER_DEG_LAT * math.cos(math.radians(self.origin.lat))

    def to_latlng(self, point: Point2D) -> LatLng:
        return LatLng(
            lat=self.origin.lat + point.y / METERS_PER_DEG_LAT,
            lng=self.origin.lng + point.x / self._meters_per_deg_lng,
        )

    def to_local(self, coord: LatLng) -> Point2D:
        return Point2D(
            x=(coord.lng - self.origin.lng) * self._meters_per_deg_lng,
            y=(coord.lat - self.origin.lat) * METERS_PER_DEG_LAT,
        )
