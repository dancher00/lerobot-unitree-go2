import numpy as np
import pytest
from lerobot.datasets import CODEBASE_VERSION, LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.utils.feature_utils import (
    build_dataset_frame,
    combine_feature_dicts,
    hw_to_dataset_features,
)

from lerobot_robot_unitree_go2 import UnitreeGo2, UnitreeGo2Config


@pytest.mark.integration
def test_mock_episode_round_trip(tmp_path) -> None:
    robot = UnitreeGo2(UnitreeGo2Config(mock=True))
    features = combine_feature_dicts(
        hw_to_dataset_features(robot.observation_features, "observation", use_video=False),
        hw_to_dataset_features(robot.action_features, "action", use_video=False),
    )
    root = tmp_path / "go2_dataset"
    dataset = LeRobotDataset.create(
        "test/go2_dataset",
        fps=20,
        root=root,
        robot_type=robot.name,
        features=features,
        use_videos=False,
    )
    robot.connect()
    try:
        for _ in range(3):
            observation = robot.get_observation()
            action = robot.send_action({"base.vx": 0.1, "base.vy": 0.0, "base.wz": 0.0})
            dataset.add_frame(
                {
                    **build_dataset_frame(features, observation, "observation"),
                    **build_dataset_frame(features, action, "action"),
                    "task": "Approach the red cylinder",
                }
            )
        dataset.save_episode()
        dataset.finalize()
    finally:
        robot.disconnect()

    reopened = LeRobotDataset("test/go2_dataset", root=root)
    assert reopened.meta.info.codebase_version == CODEBASE_VERSION == "v3.0"
    assert reopened.num_episodes == 1
    assert len(reopened) == 3
    frame = reopened[0]
    assert tuple(frame["observation.images.front"].shape[-2:]) == (480, 640)
    assert np.asarray(frame["observation.state"]).shape == (6,)
    assert np.asarray(frame["action"]).shape == (3,)
    assert "timestamp" in frame


def test_current_recorder_pipeline_records_clipped_command() -> None:
    robot = UnitreeGo2(UnitreeGo2Config(mock=True))
    teleop_processor, robot_processor, _ = make_default_processors()
    robot.connect()
    try:
        observation = robot.get_observation()
        requested = {"base.vx": 9.0, "base.vy": -9.0, "base.wz": 9.0}
        action_values = teleop_processor((requested, observation))
        action_to_send = robot_processor((action_values, observation))
        sent = robot.send_action(action_to_send)
        assert action_values == sent == {"base.vx": 0.5, "base.vy": -0.3, "base.wz": 0.8}
    finally:
        robot.disconnect()
