import pytest

from lerobot_robot_unitree_go2 import UnitreeGo2Config
from lerobot_robot_unitree_go2.teleoperators import (
    UnitreeGo2GamepadTeleopConfig,
    UnitreeGo2KeyboardTeleopConfig,
)


def test_mock_config_defaults_are_valid() -> None:
    config = UnitreeGo2Config(mock=True)
    assert config.control_frequency == 20
    assert config.max_vx == 0.5


@pytest.mark.parametrize(
    ("field", "value"),
    [("control_frequency", 0), ("max_vx", 0.0), ("watchdog_timeout_s", -1.0)],
)
def test_invalid_config_rejected(field: str, value: float) -> None:
    with pytest.raises(ValueError):
        UnitreeGo2Config(mock=True, **{field: value})


def test_real_config_requires_front_camera() -> None:
    with pytest.raises(ValueError, match="front"):
        UnitreeGo2Config(mock=False)


def test_teleop_config_validation() -> None:
    with pytest.raises(ValueError):
        UnitreeGo2KeyboardTeleopConfig(vx=0)
    with pytest.raises(ValueError):
        UnitreeGo2GamepadTeleopConfig(deadzone=1.0)
