"""Frozen wire contracts for the live demo (ARCHITECTURE.md §5.1-5.2).

These pydantic models ARE the contract: the frontend, tests, and docs all
derive from them. Breaking changes require a version bump + ADR.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class LatLng(_Strict):
    lat: float = Field(ge=-90.0, le=90.0)
    lng: float = Field(ge=-180.0, le=180.0)


class CostKind(StrEnum):
    DISTANCE = "distance"
    TRAVEL_TIME = "travel_time"


class PlanRequest(_Strict):
    start: LatLng
    goal: LatLng
    cost: CostKind = CostKind.TRAVEL_TIME


class LineString(_Strict):
    type: Literal["LineString"] = "LineString"
    coordinates: list[tuple[float, float]] = Field(min_length=2)  # [lng, lat] per GeoJSON


class PlanResponse(_Strict):
    route_id: str
    lanelet_ids: list[int]
    geometry: LineString
    distance_m: float = Field(ge=0)
    eta_s: float = Field(ge=0)
    lane_changes: int = Field(ge=0)


class PlanErrorCode(StrEnum):
    OUT_OF_COVERAGE = "OUT_OF_COVERAGE"
    UNREACHABLE_GOAL = "UNREACHABLE_GOAL"
    INVALID_INPUT = "INVALID_INPUT"
    SNAP_FAILED = "SNAP_FAILED"


class PlanError(_Strict):
    error: PlanErrorCode
    detail: str


class DriveState(StrEnum):
    DRIVING = "driving"
    PAUSED = "paused"
    ARRIVED = "arrived"
    ABORTED = "aborted"


class TelemetryFrame(_Strict):
    """Server -> client, ~10 Hz. Terminal frames carry state arrived/aborted."""

    t: float = Field(ge=0)
    lat: float
    lng: float
    heading_deg: float = Field(ge=0, lt=360)
    speed_mps: float = Field(ge=0)
    cte_m: float
    progress: float = Field(ge=0, le=1)
    state: DriveState
    reason: str | None = None


class DriveCommand(_Strict):
    """Client -> server over the drive WebSocket."""

    cmd: Literal["start", "pause", "resume", "cancel"]
