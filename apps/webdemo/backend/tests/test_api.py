import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from avcore import LatLng, Point2D
from webdemo_backend.app import create_app
from webdemo_backend.mapservice import MapService, SnapError

ORIGIN = LatLng(lat=43.7384, lng=7.4246)


@pytest.fixture(scope="module")
def client(module_monkeypatch) -> TestClient:
    module_monkeypatch.setenv("AV_TOWN_BLOCKS", "2")
    module_monkeypatch.setenv("AV_SIM_REALTIME", "0")
    return TestClient(create_app())


@pytest.fixture(scope="module")
def module_monkeypatch():
    from _pytest.monkeypatch import MonkeyPatch

    mp = MonkeyPatch()
    yield mp
    mp.undo()


@pytest.fixture(scope="module")
def service(client: TestClient) -> MapService:
    return client.app.state.mapservice  # type: ignore[no-any-return]


def town_coord(service: MapService, x: float, y: float) -> dict[str, float]:
    ll = service.projector.to_latlng(Point2D(x, y))
    return {"lat": ll.lat, "lng": ll.lng}


class TestMapService:
    def test_snap_far_away_raises(self, service: MapService) -> None:
        with pytest.raises(SnapError):
            service.snap(LatLng(lat=0.0, lng=0.0))

    def test_snap_on_road(self, service: MapService) -> None:
        lanelet_id = service.snap(service.projector.to_latlng(Point2D(40.0, -1.75)))
        assert int(lanelet_id) >= 1


class TestHealth:
    def test_healthz(self, client: TestClient) -> None:
        response = client.get("/healthz")
        assert response.status_code == 200
        assert response.json()["lanelets"] > 0


class TestPlanEndpoint:
    def test_plan_happy_path(self, client: TestClient, service: MapService) -> None:
        body = {
            "start": town_coord(service, 20.0, -1.75),
            "goal": town_coord(service, 140.0, 158.25),
        }
        response = client.post("/plan", json=body)
        assert response.status_code == 200, response.text
        data = response.json()
        assert data["route_id"].startswith("r_")
        assert len(data["geometry"]["coordinates"]) >= 2
        assert data["distance_m"] > 100
        assert data["eta_s"] > 0

    def test_plan_out_of_coverage(self, client: TestClient) -> None:
        response = client.post(
            "/plan", json={"start": {"lat": 0, "lng": 0}, "goal": {"lat": 0, "lng": 0}}
        )
        assert response.status_code == 422
        assert response.json()["error"] == "SNAP_FAILED"

    def test_plan_invalid_body(self, client: TestClient) -> None:
        response = client.post("/plan", json={"start": {"lat": 999, "lng": 0}})
        assert response.status_code == 422  # pydantic validation


class TestDriveWebSocket:
    def _plan(self, client: TestClient, service: MapService) -> str:
        response = client.post(
            "/plan",
            json={
                "start": town_coord(service, 20.0, -1.75),
                "goal": town_coord(service, 100.0, 78.25),
            },
        )
        assert response.status_code == 200
        return str(response.json()["route_id"])

    def test_unknown_route_rejected(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/drive?route_id=r_nope") as ws:
            with pytest.raises(WebSocketDisconnect) as excinfo:
                ws.receive_json()
            assert excinfo.value.code == 4404

    def test_drive_to_arrival(self, client: TestClient, service: MapService) -> None:
        route_id = self._plan(client, service)
        frames = []
        with client.websocket_connect(f"/ws/drive?route_id={route_id}") as ws:
            ws.send_json({"cmd": "start"})
            while True:
                frame = ws.receive_json()
                frames.append(frame)
                if frame["state"] in ("arrived", "aborted"):
                    break
        assert frames[-1]["state"] == "arrived"
        assert frames[-1]["progress"] > 0.97
        assert all(f["speed_mps"] >= 0 for f in frames)
        assert max(abs(f["cte_m"]) for f in frames) < 3.0

    def test_pause_and_cancel(self, client: TestClient, service: MapService) -> None:
        # In no-realtime mode the frame flood can outrun command delivery, so
        # allow a few attempts; each attempt is a fresh drive on a long route.
        for _attempt in range(3):
            response = client.post(
                "/plan",
                json={
                    "start": town_coord(service, 20.0, -1.75),
                    "goal": town_coord(service, 140.0, 158.25),
                },
            )
            route_id = str(response.json()["route_id"])
            with client.websocket_connect(f"/ws/drive?route_id={route_id}") as ws:
                ws.send_json({"cmd": "start"})
                ws.send_json({"cmd": "pause"})  # queued before frames flood
                saw_paused = False
                for _ in range(8000):
                    state = ws.receive_json()["state"]
                    if state == "paused":
                        saw_paused = True
                        break
                    if state in ("arrived", "aborted"):
                        break
                if not saw_paused:
                    continue  # lost the race; retry
                ws.send_json({"cmd": "cancel"})
                for _ in range(8000):
                    if ws.receive_json()["state"] == "aborted":
                        return  # pause + cancel both observed
                raise AssertionError("cancel never reflected in telemetry")
        raise AssertionError("pause never reflected in telemetry across 3 attempts")
