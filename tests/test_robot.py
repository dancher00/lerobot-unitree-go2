import time

import numpy as np
import pytest

from lerobot_robot_unitree_go2 import UnitreeGo2, UnitreeGo2Config
from lerobot_robot_unitree_go2.unitree import MockGo2Backend


@pytest.fixture
def robot() -> UnitreeGo2:
    instance = UnitreeGo2(UnitreeGo2Config(mock=True, watchdog_timeout_s=1.0))
    instance.connect()
    yield instance
    instance.disconnect()


def test_mock_connect_disconnect() -> None:
    robot = UnitreeGo2(UnitreeGo2Config(mock=True))
    assert not robot.is_connected
    robot.connect()
    assert robot.is_connected
    robot.disconnect()
    assert not robot.is_connected


def test_observation_shape(robot: UnitreeGo2) -> None:
    observation = robot.get_observation()
    assert observation["front"].shape == (480, 640, 3)
    assert observation["front"].dtype == np.uint8
    assert [observation[name] for name in list(robot.observation_features)[:6]] == pytest.approx(
        [0.0] * 6
    )


def test_action_shape_and_clipping_mutates_recorded_action(robot: UnitreeGo2) -> None:
    action = {"base.vx": 8.0, "base.vy": -8.0, "base.wz": 8.0}
    sent = robot.send_action(action)
    assert list(sent) == ["base.vx", "base.vy", "base.wz"]
    assert sent == {"base.vx": 0.5, "base.vy": -0.3, "base.wz": 0.8}
    assert action == sent
    backend = robot.backend
    assert isinstance(backend, MockGo2Backend)
    assert backend.command_history[-1] == (0.5, -0.3, 0.8)


def test_missing_action_key_rejected(robot: UnitreeGo2) -> None:
    with pytest.raises(ValueError, match="missing"):
        robot.send_action({"base.vx": 0.0, "base.vy": 0.0})


def test_mock_velocity_response(robot: UnitreeGo2) -> None:
    robot.send_action({"base.vx": 0.2, "base.vy": 0.1, "base.wz": 0.3})
    time.sleep(0.02)
    observation = robot.get_observation()
    assert 0 < observation["base.vx"] < 0.2
    assert observation["base.yaw"] > 0


def test_emergency_stop(robot: UnitreeGo2) -> None:
    robot.send_action({"base.vx": 0.2, "base.vy": 0.1, "base.wz": 0.3})
    robot.emergency_stop("test")
    backend = robot.backend
    assert isinstance(backend, MockGo2Backend)
    assert backend.command_history[-1] == (0.0, 0.0, 0.0)


def test_extended_mock_observation_matches_declared_schema() -> None:
    robot = UnitreeGo2(UnitreeGo2Config(mock=True, extended_state=True))
    robot.connect()
    try:
        observation = robot.get_observation()
        assert set(observation) == set(robot.observation_features)
        assert len([key for key in observation if ".joint." in key]) == 12
    finally:
        robot.disconnect()
