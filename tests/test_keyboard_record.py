import pytest

from lerobot_robot_unitree_go2.keyboard_record import build_parser, build_record_args


def test_keyboard_record_builds_current_lerobot_arguments() -> None:
    args = build_parser().parse_args(
        [
            "--interface",
            "en7",
            "--serial",
            "123456",
            "--repo-id",
            "test/go2_walk",
            "--task",
            "Walk forward",
            "--episodes",
            "2",
            "--push-to-hub",
        ]
    )
    record_args = build_record_args(args)

    assert "--robot.type=unitree_go2" in record_args
    assert "--robot.network_interface=en7" in record_args
    assert 'serial_number_or_name: "123456"' in next(
        item for item in record_args if item.startswith("--robot.cameras=")
    )
    assert "--teleop.type=unitree_go2_keyboard" in record_args
    assert "--teleop.vx=0.3" in record_args
    assert "--dataset.single_task=Walk forward" in record_args
    assert "--dataset.num_episodes=2" in record_args
    assert "--dataset.push_to_hub=true" in record_args


def test_keyboard_record_rejects_invalid_speed() -> None:
    args = build_parser().parse_args(
        ["--serial", "123", "--repo-id", "test/go2", "--task", "Walk", "--vx", "0"]
    )
    with pytest.raises(ValueError, match="vx, vy, and wz"):
        build_record_args(args)
