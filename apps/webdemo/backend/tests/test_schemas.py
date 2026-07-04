import pytest
from pydantic import ValidationError

from webdemo_backend.schemas import (
    CostKind,
    DriveCommand,
    DriveState,
    LatLng,
    LineString,
    PlanErrorCode,
    PlanRequest,
    PlanResponse,
    TelemetryFrame,
)

MONACO = {"lat": 43.7384, "lng": 7.4246}


class TestPlanRequest:
    def test_valid_request_with_default_cost(self) -> None:
        req = PlanRequest.model_validate({"start": MONACO, "goal": {"lat": 43.74, "lng": 7.429}})
        assert req.cost is CostKind.TRAVEL_TIME

    @pytest.mark.parametrize("lat,lng", [(91, 0), (-91, 0), (0, 181), (0, -181)])
    def test_out_of_range_coordinates_rejected(self, lat: float, lng: float) -> None:
        with pytest.raises(ValidationError):
            LatLng(lat=lat, lng=lng)

    def test_unknown_fields_rejected(self) -> None:
        with pytest.raises(ValidationError):
            PlanRequest.model_validate({"start": MONACO, "goal": MONACO, "sql": "drop table"})


class TestPlanResponse:
    def test_round_trip(self) -> None:
        resp = PlanResponse(
            route_id="r_1",
            lanelet_ids=[101, 104],
            geometry=LineString(coordinates=[(7.4246, 43.7384), (7.429, 43.74)]),
            distance_m=812.4,
            eta_s=96.3,
            lane_changes=1,
        )
        assert PlanResponse.model_validate_json(resp.model_dump_json()) == resp

    def test_degenerate_geometry_rejected(self) -> None:
        with pytest.raises(ValidationError):
            LineString(coordinates=[(7.4246, 43.7384)])

    def test_error_codes_are_stable(self) -> None:
        assert {c.value for c in PlanErrorCode} == {
            "OUT_OF_COVERAGE",
            "UNREACHABLE_GOAL",
            "INVALID_INPUT",
            "SNAP_FAILED",
        }


class TestTelemetry:
    def test_valid_frame(self) -> None:
        frame = TelemetryFrame(
            t=12.3,
            lat=43.739,
            lng=7.426,
            heading_deg=78.4,
            speed_mps=8.1,
            cte_m=0.12,
            progress=0.34,
            state=DriveState.DRIVING,
        )
        assert frame.reason is None

    @pytest.mark.parametrize("progress", [-0.1, 1.1])
    def test_progress_bounds(self, progress: float) -> None:
        with pytest.raises(ValidationError):
            TelemetryFrame(
                t=0,
                lat=0,
                lng=0,
                heading_deg=0,
                speed_mps=0,
                cte_m=0,
                progress=progress,
                state=DriveState.DRIVING,
            )

    def test_drive_command_literal(self) -> None:
        assert DriveCommand(cmd="cancel").cmd == "cancel"
        with pytest.raises(ValidationError):
            DriveCommand.model_validate({"cmd": "warp"})
