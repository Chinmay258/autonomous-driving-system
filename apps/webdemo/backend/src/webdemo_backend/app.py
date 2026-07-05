"""FastAPI application: POST /plan + WS /ws/drive (contracts in schemas.py)."""

from __future__ import annotations

import asyncio
import math
import os
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

from avcore import LatLng as CoreLatLng
from avcore import Point2D, UnreachableGoalError
from webdemo_backend.mapservice import MapService, SnapError
from webdemo_backend.schemas import (
    DriveCommand,
    DriveState,
    LineString,
    PlanError,
    PlanErrorCode,
    PlanRequest,
    PlanResponse,
    TelemetryFrame,
)
from webdemo_backend.simulator import SimFrame, SimState, VehicleSimulator

TICK_S = 0.1

_SIM_TO_DRIVE = {
    SimState.DRIVING: DriveState.DRIVING,
    SimState.ARRIVED: DriveState.ARRIVED,
    SimState.ABORTED: DriveState.ABORTED,
}


def create_app() -> FastAPI:
    app = FastAPI(title="AV Route Planning Demo", version="0.1.0")
    service = MapService.from_synthetic_town(blocks=int(os.environ.get("AV_TOWN_BLOCKS", "3")))
    realtime = os.environ.get("AV_SIM_REALTIME", "1") == "1"
    app.state.mapservice = service

    def plan_error(code: PlanErrorCode, detail: str, status: int = 422) -> JSONResponse:
        return JSONResponse(
            status_code=status,
            content=PlanError(error=code, detail=detail).model_dump(),
        )

    @app.get("/healthz")
    def healthz() -> dict[str, object]:
        return {"status": "ok", "lanelets": service.lanelet_count}

    @app.get("/map/roads")
    def map_roads() -> dict[str, object]:
        return service.road_network_geojson()

    static_dir = Path(__file__).parent / "static"

    @app.get("/", include_in_schema=False)
    def index() -> FileResponse:
        return FileResponse(static_dir / "index.html")

    @app.post("/plan", response_model=PlanResponse, responses={422: {"model": PlanError}})
    def plan(request: PlanRequest) -> PlanResponse | JSONResponse:
        try:
            route_id, stored = service.plan(
                CoreLatLng(lat=request.start.lat, lng=request.start.lng),
                CoreLatLng(lat=request.goal.lat, lng=request.goal.lng),
                request.cost,
            )
        except SnapError as exc:
            return plan_error(PlanErrorCode.SNAP_FAILED, str(exc))
        except UnreachableGoalError as exc:
            return plan_error(PlanErrorCode.UNREACHABLE_GOAL, str(exc))
        route = stored.route
        return PlanResponse(
            route_id=route_id,
            lanelet_ids=[int(lid) for lid in route.lanelet_ids],
            geometry=LineString(coordinates=service.to_geojson_coords(route.centerline)),
            distance_m=route.distance_m,
            eta_s=route.eta_s,
            lane_changes=route.lane_changes,
        )

    def to_telemetry(frame: SimFrame, *, state: DriveState | None = None) -> TelemetryFrame:
        coord = service.projector.to_latlng(Point2D(frame.x, frame.y))
        heading = (90.0 - math.degrees(frame.heading_rad)) % 360.0  # compass, north-up
        return TelemetryFrame(
            t=round(frame.t, 3),
            lat=coord.lat,
            lng=coord.lng,
            heading_deg=heading,
            speed_mps=round(frame.speed_mps, 3),
            cte_m=round(frame.cte_m, 3),
            progress=round(frame.progress, 5),
            state=state if state is not None else _SIM_TO_DRIVE[frame.state],
            reason=frame.reason,
        )

    @app.websocket("/ws/drive")
    async def drive(ws: WebSocket) -> None:
        await ws.accept()
        stored = service.get_route(ws.query_params.get("route_id", ""))
        if stored is None:
            await ws.close(code=4404, reason="unknown route_id")
            return

        try:
            first = DriveCommand.model_validate(await ws.receive_json())
        except WebSocketDisconnect:
            return
        if first.cmd != "start":
            await ws.close(code=4400, reason="expected start command")
            return

        sim = VehicleSimulator(stored.route.centerline, stored.speed_limits_mps)
        paused = False
        cancelled = False

        async def read_commands() -> None:
            nonlocal paused, cancelled
            try:
                while True:
                    cmd = DriveCommand.model_validate(await ws.receive_json()).cmd
                    if cmd == "pause":
                        paused = True
                    elif cmd == "resume":
                        paused = False
                    elif cmd == "cancel":
                        cancelled = True
                        return
            except WebSocketDisconnect:
                cancelled = True

        reader = asyncio.create_task(read_commands())
        try:
            while True:
                if cancelled:
                    await ws.send_text(
                        to_telemetry(sim.last_frame, state=DriveState.ABORTED).model_dump_json()
                    )
                    break
                if paused:
                    frame_json = to_telemetry(
                        sim.last_frame, state=DriveState.PAUSED
                    ).model_dump_json()
                else:
                    frame_json = to_telemetry(sim.step(TICK_S)).model_dump_json()
                await ws.send_text(frame_json)
                if sim.last_frame.state is not SimState.DRIVING:
                    break
                # Non-realtime still needs a real (timer-based) yield so the
                # command reader task is never starved by the frame loop.
                await asyncio.sleep(TICK_S if realtime else 0.001)
        except WebSocketDisconnect:
            pass
        finally:
            reader.cancel()

    return app
