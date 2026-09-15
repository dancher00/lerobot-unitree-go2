import pytest

from lerobot_robot_unitree_go2.teleoperators.gamepad import apply_deadzone


def test_deadzone() -> None:
    assert apply_deadzone(0.1, 0.12) == 0.0
    assert apply_deadzone(-0.1, 0.12) == 0.0
    assert apply_deadzone(1.0, 0.12) == 1.0
    assert apply_deadzone(-1.0, 0.12) == -1.0
    assert apply_deadzone(0.56, 0.12) == pytest.approx(0.5)
