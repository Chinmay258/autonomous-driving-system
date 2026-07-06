"""MapService loading the canonical Lanelet2 artifact (AV_MAP_PATH path)."""

from pathlib import Path

import pytest
from avmap_tools import synthetic_town, write_osm

from avcore import LatLng
from webdemo_backend.app import create_app
from webdemo_backend.mapservice import MapService
from webdemo_backend.schemas import CostKind

ORIGIN = LatLng(lat=43.7384, lng=7.4246)


@pytest.fixture(scope="module")
def artifact(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("maps") / "town.osm"
    path.write_text(write_osm(synthetic_town(2, 2), ORIGIN), encoding="utf-8")
    return path


class TestFromOsmArtifact:
    def test_loads_and_plans(self, artifact: Path) -> None:
        service = MapService.from_osm_artifact(str(artifact))
        assert service.lanelet_count > 50
        _, stored = service.plan(
            LatLng(lat=43.73838, lng=7.42485),
            LatLng(lat=43.73952, lng=7.42633),
            CostKind.TRAVEL_TIME,
        )
        assert stored.route.distance_m > 0
        assert len(stored.speed_limits_mps) == len(stored.route.centerline)

    def test_app_honors_av_map_path(self, artifact: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("AV_MAP_PATH", str(artifact))
        app = create_app()
        service = app.state.mapservice
        assert service.lanelet_count > 50

    def test_missing_artifact_raises(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("AV_MAP_PATH", "does/not/exist.osm")
        with pytest.raises(FileNotFoundError):
            create_app()
