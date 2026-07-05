import math
from itertools import pairwise

import pytest

from avcore import Point2D
from webdemo_backend.simulator import SimState, VehicleSimulator


def straight_path(length: float = 200.0, step: float = 10.0) -> tuple[Point2D, ...]:
    n = int(length / step) + 1
    return tuple(Point2D(i * step, 0.0) for i in range(n))


def l_shaped_path() -> tuple[Point2D, ...]:
    """200 m east, 90-degree corner (smoothed by 3 knots), 200 m north."""
    east = [Point2D(x, 0.0) for x in range(0, 191, 10)]
    corner = [Point2D(196.0, 1.0), Point2D(199.0, 4.0), Point2D(200.0, 10.0)]
    north = [Point2D(200.0, y) for y in range(20, 201, 10)]
    return tuple(east + corner + north)


def drive_to_terminal(sim: VehicleSimulator, max_steps: int = 20000):
    frames = []
    for _ in range(max_steps):
        frame = sim.step(0.1)
        frames.append(frame)
        if frame.state is not SimState.DRIVING:
            return frames
    raise AssertionError("simulation never terminated")


class TestStraightDrive:
    def test_arrives_with_low_cross_track_error(self) -> None:
        path = straight_path()
        sim = VehicleSimulator(path, tuple(13.9 for _ in path))
        frames = drive_to_terminal(sim)
        assert frames[-1].state is SimState.ARRIVED
        assert max(f.cte_m for f in frames) < 0.5
        assert frames[-1].progress > 0.97

    def test_respects_speed_limit(self) -> None:
        path = straight_path(400.0)
        limit = 10.0
        sim = VehicleSimulator(path, tuple(limit for _ in path))
        frames = drive_to_terminal(sim)
        assert max(f.speed_mps for f in frames) <= limit + 0.2

    def test_progress_is_monotonic(self) -> None:
        path = straight_path()
        sim = VehicleSimulator(path, tuple(13.9 for _ in path))
        frames = drive_to_terminal(sim)
        progresses = [f.progress for f in frames]
        assert all(b >= a - 1e-9 for a, b in pairwise(progresses))


class TestCornerDrive:
    def test_slows_for_corner_and_tracks_it(self) -> None:
        path = l_shaped_path()
        sim = VehicleSimulator(path, tuple(13.9 for _ in path))
        frames = drive_to_terminal(sim)
        assert frames[-1].state is SimState.ARRIVED
        # Speed in the corner region must dip well below the straightaway speed.
        corner_speeds = [f.speed_mps for f in frames if 180.0 < f.x < 210.0 and f.y < 30.0]
        assert corner_speeds, "vehicle never entered corner region"
        assert min(corner_speeds) < 9.0
        assert max(f.cte_m for f in frames) < 2.0

    def test_heading_ends_northbound(self) -> None:
        path = l_shaped_path()
        sim = VehicleSimulator(path, tuple(13.9 for _ in path))
        frames = drive_to_terminal(sim)
        final_heading_deg = math.degrees(frames[-1].heading_rad) % 360.0
        assert final_heading_deg == pytest.approx(90.0, abs=15.0)


class TestValidationAndTerminalBehavior:
    def test_rejects_degenerate_path(self) -> None:
        with pytest.raises(ValueError):
            VehicleSimulator((Point2D(0, 0),), (10.0,))

    def test_rejects_mismatched_speeds(self) -> None:
        with pytest.raises(ValueError):
            VehicleSimulator((Point2D(0, 0), Point2D(10, 0)), (10.0,))

    def test_terminal_frame_is_sticky(self) -> None:
        path = straight_path(30.0)
        sim = VehicleSimulator(path, tuple(13.9 for _ in path))
        drive_to_terminal(sim)
        again = sim.step(0.1)
        assert again.state is SimState.ARRIVED
        assert again == sim.last_frame
