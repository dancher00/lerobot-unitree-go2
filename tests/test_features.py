from lerobot.utils.feature_utils import hw_to_dataset_features

from lerobot_robot_unitree_go2 import UnitreeGo2, UnitreeGo2Config


def test_lerobot_feature_schema_compatibility() -> None:
    robot = UnitreeGo2(UnitreeGo2Config(mock=True))
    observation = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
    action = hw_to_dataset_features(robot.action_features, "action", use_video=True)

    assert observation["observation.images.front"]["shape"] == (480, 640, 3)
    assert observation["observation.state"]["shape"] == (6,)
    assert observation["observation.state"]["names"] == [
        "base.vx",
        "base.vy",
        "base.wz",
        "base.roll",
        "base.pitch",
        "base.yaw",
    ]
    assert action["action"]["shape"] == (3,)
    assert action["action"]["names"] == ["base.vx", "base.vy", "base.wz"]


def test_plugin_factory_discovers_implementation() -> None:
    from lerobot.robots import make_robot_from_config

    robot = make_robot_from_config(UnitreeGo2Config(mock=True))
    assert isinstance(robot, UnitreeGo2)
