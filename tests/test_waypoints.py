import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from waypoints import Waypoint, append_waypoint, load_waypoints, save_waypoints  # noqa: E402


def test_waypoint_requires_six_joints():
    with pytest.raises(ValueError):
        Waypoint(joint_positions=[0.0, 0.0, 0.0])


def test_save_and_load_round_trip(tmp_path):
    path = tmp_path / "waypoints.json"
    waypoints = [
        Waypoint(joint_positions=[0.0] * 6, name="home"),
        Waypoint(joint_positions=[0.1, -0.2, 1.1, 0.0, 0.5, 0.0], name="approach"),
    ]
    save_waypoints(path, waypoints)
    assert load_waypoints(path) == waypoints


def test_append_waypoint_creates_file(tmp_path):
    path = tmp_path / "waypoints.json"
    append_waypoint(path, Waypoint(joint_positions=[0.0] * 6, name="home"))
    append_waypoint(path, Waypoint(joint_positions=[1.0] * 6, name="second"))
    assert [wp.name for wp in load_waypoints(path)] == ["home", "second"]
